########## INIT ####################################################################################

##### Imports #####

### Standard ###
import time
now = time.time
from time import sleep

### Special ###
import numpy as np

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence

### Local ###
from magpie_control.BT import Move_Arm, Open_Gripper, Close_Gripper, Gripper_Aperture_OK, Set_Gripper
from aspire.symbols import extract_pose_as_homog, env_var



########## CONSTANTS & COMPONENTS ##################################################################

### Init data structs & Keys ###
MP2BB          = dict()  # Hack the BB object into the built-in namespace
HAND_OBJ_KEY   = "handHas"
PAUSE_KEY      = "robotPaused"
PROTO_PICK_ROT = np.array( [[ -1.0,  0.0,  0.0, ],
                            [  0.0,  1.0,  0.0, ],
                            [  0.0,  0.0, -1.0, ]] )
 # Link 6 to gripper tip
_GRASP_OFFSET_X = -1.15/100.0 + 0.0075
_GRASP_OFFSET_Y =  1.30/100.0 - 0.0075
_GRASP_OFFSET_Z = 0.110 + 0.110 + 0.010
TCP_XFORM = np.array([
    [1, 0, 0, _GRASP_OFFSET_X ],
    [0, 1, 0, _GRASP_OFFSET_Y ],
    [0, 0, 1, _GRASP_OFFSET_Z ],
    [0, 0, 0, 1               ],
])

### Set important BB items ###
MP2BB[ PAUSE_KEY ] = False


########## BASE CLASS ##############################################################################


class BasicBehavior( Behaviour ):
    """ Abstract class for repetitive housekeeping """
    
    def __init__( self, name = None, ctrl = None ):
        """ Set name to the child class name unless otherwise specified """
        if name is None:
            super().__init__( name = str( self.__class__.__name__  ) )
        else:
            super().__init__( name = name )
        self.ctrl  = ctrl
        self.msg   = ""
        self.logger.debug( f"[{self.name}::__init__()]" )
        if self.ctrl is None:
            self.logger.warning( f"{self.name} is NOT conntected to a robot controller!" )
        self.count = 0
        

    def setup(self):
        """ Virtual setup for base class """
        self.logger.debug( f"[{self.name}::setup()]" )          
        
        
    def initialise( self ):
        """ Run first time behaviour is ticked or not RUNNING.  Will be run again after SUCCESS/FAILURE. """
        self.status = Status.RUNNING # Do not let the behavior idle in INVALID
        self.logger.debug( f"[{self.name}::initialise()]" ) 
        self.count = 0         

        
    def terminate( self, new_status ):
        """ Log how the behavior terminated """
        self.status = new_status
        self.logger.debug( f"[{self.name}::terminate().terminate()][{self.status}->{new_status}]" )
        
        
    def update( self ):
        """ Return true in all cases """
        self.status = Status.SUCCESS
        return self.status
    

    def stall( self, Nwait ):
        """ Run at least `Nwait` ticks """
        rtnStat = Status.INVALID
        if self.count < Nwait:
            rtnStat = Status.RUNNING
        else:
            rtnStat = Status.SUCCESS
        self.count += 1
        return rtnStat
    


########## BASIC BEHAVIORS #########################################################################

### Constants ###
LIBBT_TS_S       = 0.25
DEFAULT_TRAN_ERR = 0.010 # 0.002
DEFAULT_ORNT_ERR = 3*np.pi/180.0

def pass_msg_up( bt : BasicBehavior, failBelow = False ):
    if bt.parent is not None:
        if bt.status == Status.FAILURE:
            if (bt.parent.status != Status.FAILURE) or (len( bt.parent.msg ) == 0):
                setattr( bt.parent, "msg", bt.msg )
                pass_msg_up( bt.parent, True )
            else:
                pass_msg_up( bt.parent )
        elif failBelow:
            setattr( bt.parent, "msg", bt.msg )
            pass_msg_up( bt.parent, True )


def connect_BT_to_robot( bt : BasicBehavior, robot ):
    """ Assign `robot` controller to `bt` and recursively to all nodes below """
    if hasattr( bt, 'ctrl' ):
        bt.ctrl = robot
    if len( bt.children ):
        for child in bt.children:
            connect_BT_to_robot( child, robot )



########## HELPER FUNCTIONS ########################################################################

def display_PDLS_plan( plan ):
    print( f"\nPlan output from PDDLStream:" )
    if plan is not None:
        for i, action in enumerate( plan ):
            # print( dir( action ) )
            print( f"\t{i+1}: { action.__class__.__name__ }, {action.name}" )
            for j, arg in enumerate( action.args ):
                print( f"\t\tArg {j}:\t{type( arg )}, {arg}" )
    else:
        print( plan )



########## BT-PLANNER INTERFACE ####################################################################

class BT_Runner:
    """ Run a BT with checks """

    def __init__( self, root, tickHz = 4.0, limit_s = 20.0 ):
        """ Set root node reference and running parameters """
        self.root   = root
        self.status = Status.INVALID
        self.freq   = tickHz
        self.period = 1.0 / tickHz
        self.msg    = ""
        self.Nlim   = int( limit_s * tickHz )
        self.i      = 0
        self.tLast  = now()


    def setup_BT_for_running( self ):
        """ Prep BT for running """
        self.root.setup_with_descendants()


    def display_BT( self ):
        """ Draw the BT along with the status of all the nodes """
        print( py_trees.display.unicode_tree( root = self.root, show_status = True ) )


    def p_ended( self ):
        """ Has the BT ended? """
        return self.status in ( Status.FAILURE, Status.SUCCESS )
    

    def set_fail( self, msg = "DEFAULT MSG: STOPPED" ):
        """ Handle external signals to halt BT execution """
        self.status = Status.FAILURE
        self.msg    = msg


    def per_sleep( self ):
        """ Sleep for the remainder of the period """
        # NOTE: Run this AFTER BT and associated work have finished
        tNow = now()
        elap = (tNow - self.tLast)
        if (elap < self.period):
            sleep( self.period - elap )
        self.tLast = now()


    def tick_once( self ):
        """ Run one simulation step """
        ## Advance BT ##
        if not self.p_ended():
            self.root.tick_once()
        self.status = self.root.status
        self.i += 1
        ## Check Conditions ##
        if (self.i >= self.Nlim) and (not self.p_ended()):
            self.set_fail( "BT TIMEOUT" )
        if self.p_ended():
            pass_msg_up( self.root )
            if (len( self.msg ) == 0) and hasattr( self.root, 'msg' ):
                self.msg = self.root.msg
            self.display_BT() 
        return self.root.tip().name



########## BLOCKS DOMAIN HELPER FUNCTIONS ##########################################################


def grasp_pose_from_obj_pose( anyPose ):
    """ Return the homogeneous coords given [Px,Py,Pz,Ow,Ox,Oy,Oz] """
    rtnPose = extract_pose_as_homog( anyPose )
    offVec = TCP_XFORM[0:3,3]
    rtnPose[0:3,0:3] = PROTO_PICK_ROT
    rtnPose[0:3,3]  += offVec
    return rtnPose


def grasp_pose_from_posn( posn ):
    """ Return the homogeneous coords given [Px,Py,Pz] """
    rtnPose = np.eye(4)
    rtnPose[0:3,3] = posn
    offVec = TCP_XFORM[0:3,3]
    rtnPose[0:3,0:3] = PROTO_PICK_ROT
    rtnPose[0:3,3]  += offVec
    return rtnPose



########## BLOCKS DOMAIN BEHAVIOR TREES ############################################################

class GroundedAction( Sequence ):
    """ This is the parent class for all actions available to the planner """

    def __init__( self, args = None, robot = None, name = "Grounded Sequence" ):
        """ Init BT """
        super().__init__( name = name, memory = True )
        self.args    = args if (args is not None) else list() # Symbols involved in this action
        self.symbols = list() #  Symbol on which this behavior relies
        self.msg     = "" # ---- Message: Reason this action failed -or- OTHER
        self.ctrl    = robot # - Agent that executes

    def __repr__( self ):
        """ Get the name, Assume child classes made it sufficiently descriptive """
        return str( self.name )
    


class MoveFree( GroundedAction ):
    """ Move the unburdened effector to the given location """
    def __init__( self, args, robot = None, name = None, suppressGrasp = False ):

        # ?poseBgn ?poseEnd
        poseBgn, poseEnd = args
        if poseBgn is None:
            poseBgn = robot.get_tcp_pose()

        if not suppressGrasp:
            poseBgn = grasp_pose_from_obj_pose( extract_pose_as_homog( poseBgn ) )
            poseEnd = grasp_pose_from_obj_pose( extract_pose_as_homog( poseEnd ) )
        else:
            poseBgn = extract_pose_as_homog( poseBgn )
            poseEnd = extract_pose_as_homog( poseEnd )

        if name is None:
            name = f"Move Free from {poseBgn} --to-> {poseEnd}"

        super().__init__( args, robot, name )
        
        psnMid1 = np.array( poseBgn[0:3,3] )
        psnMid2 = np.array( poseEnd[0:3,3] )
        psnMid1[2] = env_var("_Z_SAFE")
        psnMid2[2] = env_var("_Z_SAFE")
        poseMd1 = np.eye(4) 
        poseMd2 = np.eye(4)
        poseMd1[0:3,0:3] = PROTO_PICK_ROT
        poseMd2[0:3,0:3] = PROTO_PICK_ROT
        poseMd1[0:3,3] = psnMid1
        poseMd2[0:3,3] = psnMid2
        
        transportMotn = Sequence( name = "Move Arm Safely", memory = True )
        transportMotn.add_children( [
            Move_Arm( poseMd1, ctrl = robot, linSpeed = env_var("_ROBOT_FREE_SPEED") ),
            Move_Arm( poseMd2, ctrl = robot, linSpeed = env_var("_ROBOT_FREE_SPEED") ),
            Move_Arm( poseEnd, ctrl = robot, linSpeed = env_var("_ROBOT_FREE_SPEED") ),
        ] )

        self.add_child( transportMotn )

        # self.poseEnd = extract_pose_as_homog( poseEnd )
                
        # self.add_child(
        #     Move_Arm( self.poseEnd, ctrl = robot, linSpeed = env_var("_ROBOT_FREE_SPEED") )
        # )



class Pick( GroundedAction ):
    """ Add object to the gripper payload """
    def __init__( self, args, robot = None, name = None ):

        # ?label ?pose ?prevSupport
        label, pose, prevSupport = args
        
        if name is None:
            name = f"Pick {label} at {pose.pose} from {prevSupport}"
        super().__init__( args, robot, name )

        self.add_child( 
            Close_Gripper( ctrl = robot  )
        )



class Unstack( GroundedAction ):
    """ Add object to the gripper payload """
    def __init__( self, args, robot = None, name = None ):

        # ?label ?pose ?prevSupport
        label, pose, prevSupport = args
        
        if name is None:
            name = f"Unstack {label} at {pose.pose} from {prevSupport}"
        super().__init__( args, robot, name )

        self.add_child( 
            Close_Gripper( ctrl = robot  )
        )



class MoveHolding( GroundedAction ):
    """ Move the burdened effector to the given location """
    def __init__( self, args, robot = None, name = None ):

        # ?poseBgn ?poseEnd ?label
        poseBgn, poseEnd, label = args

        if name is None:
            name = f"Move Holding {label} --to-> {poseEnd}"
        super().__init__( args, robot, name )

        poseBgn = grasp_pose_from_obj_pose( extract_pose_as_homog( poseBgn ) )
        poseEnd = grasp_pose_from_obj_pose( extract_pose_as_homog( poseEnd ) )
        psnMid1 = np.array( poseBgn[0:3,3] )
        psnMid2 = np.array( poseEnd[0:3,3] )
        psnMid1[2] = env_var("_Z_SAFE")
        psnMid2[2] = env_var("_Z_SAFE")
        poseMd1 = np.eye(4) 
        poseMd2 = np.eye(4)
        poseMd1[0:3,0:3] = PROTO_PICK_ROT
        poseMd2[0:3,0:3] = PROTO_PICK_ROT
        poseMd1[0:3,3] = psnMid1
        poseMd2[0:3,3] = psnMid2
    
        checkedMotion = Sequence( name = "Move Without Dropping", memory = False )
        dropChecker   = Gripper_Aperture_OK( 
            env_var("_BLOCK_SCALE"), 
            margin_m = env_var("_BLOCK_SCALE")*0.50, 
            name = "Check Holding", ctrl = robot  
        )
        transportMotn = Sequence( name = "Move Object", memory = True )
        transportMotn.add_children( [
            Move_Arm( poseMd1, ctrl = robot, linSpeed = env_var("_ROBOT_HOLD_SPEED") ),
            Move_Arm( poseMd2, ctrl = robot, linSpeed = env_var("_ROBOT_HOLD_SPEED") ),
            Move_Arm( poseEnd, ctrl = robot, linSpeed = env_var("_ROBOT_HOLD_SPEED") ),
        ] )
        checkedMotion.add_children([
            dropChecker,
            transportMotn
        ])

        self.add_child( checkedMotion )



class Place( GroundedAction ):
    """ Let go of gripper payload """
    def __init__( self, args, robot = None, name = None ):

        # ?label ?pose ?support
        label, pose, support = args
        
        if name is None:
            name = f"Place {label} at {pose.pose} onto {support}"
        super().__init__( args, robot, name )

        # self.add_child( 
        #     Open_Gripper( ctrl = robot  )
        # )

        self.add_children([ 
            Set_Gripper( env_var("_BLOCK_SCALE")*2.0, name = "Release Object", ctrl = robot ),
            Close_Gripper( ctrl = robot ),
            Gripper_Aperture_OK( 
                env_var("_BLOCK_SCALE"), 
                margin_m = env_var("_BLOCK_SCALE")*0.50, 
                name = "Check Placed", ctrl = robot  
            ),
            Open_Gripper( ctrl = robot ),
        ])


class Stack( GroundedAction ):
    """ Let go of gripper payload """
    def __init__( self, args, robot = None, name = None ):

        # ?labelUp ?poseUp ?labelDn
        labelUp, poseUp, labelDn = args
        
        if name is None:
            name = f"Stack {labelUp} at {poseUp.pose} onto {labelDn}"
        super().__init__( args, robot, name )

        # self.add_child( 
        #     Open_Gripper( ctrl = robot  )
        # )

        self.add_children([ 
            Set_Gripper( env_var("_BLOCK_SCALE")*2.0, name = "Release Object", ctrl = robot ),
            Close_Gripper( ctrl = robot ),
            Gripper_Aperture_OK( 
                env_var("_BLOCK_SCALE"), 
                margin_m = env_var("_BLOCK_SCALE")*0.50, 
                name = "Check Placed", ctrl = robot  
            ),
            Open_Gripper( ctrl = robot ),
        ])



########## PLANS ###################################################################################

class Plan( Sequence ):
    """ Special BT `Sequence` with assigned priority, cost, and confidence """

    def __init__( self ):
        """ Set default priority """
        super().__init__( name = "PDDLStream Plan", memory = True )
        self.msg    = "" # --------------- Message: Reason this plan failed -or- OTHER
        self.ctrl   = None
    
    def __len__( self ):
        """ Return the number of children """
        return len( self.children )

    def append( self, action ):
        """ Add an action """
        self.add_child( action )
    
    def __repr__( self ):
        """ String representation of the plan """
        return f"<{self.name}, Status: {self.status}>"



########## PDLS --TO-> BT ##########################################################################

def get_ith_BT_action_from_PDLS_plan( pdlsPlan, i, robot, planner, sensePeriod_s ):
    """ Fetch the `i`th item from `pdlsPlan` and parameterize a BT that operates on the environment """
    actName  = pdlsPlan[i].name
    actArgs  = pdlsPlan[i].args
    btAction = None
    print( f"Planner Type: {type( planner )}" )
    if actName == "move_free":
        btAction = MoveFree( actArgs, robot = robot )
        # btAction = Interleaved_MoveFree_and_PerceiveScene( 
        #     MoveFree( actArgs, robot = robot ), 
        #     planner, 
        #     sensePeriod_s, 
        #     initSenseStep = False
        # )
    elif actName == "pick":
        btAction = Pick( actArgs, robot = robot )
    elif actName == "unstack":
        btAction = Unstack( actArgs, robot = robot )
    elif actName == "move_holding":
        btAction = MoveHolding( actArgs, robot = robot )
    elif actName == "place":
        btAction = Place( actArgs, robot = robot )
    elif actName == "stack":
        btAction = Stack( actArgs, robot = robot )
    else:
        raise NotImplementedError( f"There is no BT procedure defined for a PDDL action named {actName}!" )
    print( f"Action {i+1}, {actName} --> {btAction.name}, planned!" )
    return btAction


def get_BT_plan_until_block_change( pdlsPlan, planner, sensePeriod_s, robot ):
    """ Translate the PDLS plan to one that can be executed by the robot """
    rtnBTlst = []
    print( f"Planner Type: {type( planner )}" )
    if pdlsPlan is not None:
        for i in range( len( pdlsPlan ) ):
            btAction = get_ith_BT_action_from_PDLS_plan( pdlsPlan, i, robot, planner, sensePeriod_s )
            rtnBTlst.append( btAction )
            if btAction.__class__ in ( Place, Stack ):
                break
    rtnPlan = Plan()
    rtnPlan.add_children( rtnBTlst )
    return rtnPlan


def get_BT_plan( pdlsPlan, planner, sensePeriod_s, robot ):
    """ Translate the PDLS plan to one that can be executed by the robot """
    rtnBTlst = []
    print( f"Planner Type: {type( planner )}" )
    if pdlsPlan is not None:
        for i in range( len( pdlsPlan ) ):
            btAction = get_ith_BT_action_from_PDLS_plan( pdlsPlan, i, robot, planner, sensePeriod_s )
            rtnBTlst.append( btAction )
    rtnPlan = Plan()
    rtnPlan.add_children( rtnBTlst )
    return rtnPlan