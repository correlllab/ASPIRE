########## INIT ####################################################################################

##### Imports #####

### Standard ###
import time
now = time.time
from time import sleep

### Special ###
import numpy as np

### Local ###
from magpie_control.BT import ( Move_Arm, Open_Gripper, Close_Gripper, Gripper_Aperture_OK, 
                                Set_Gripper, SequenceP, )
from aspire.symbols import extract_pose_as_homog, env_var



########## CONSTANTS & COMPONENTS ##################################################################

### Init data structs & Keys ###
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

class GroundedAction( SequenceP ):
    """ This is the parent class for all actions available to the planner """

    def __init__( self, args = None, robot = None, name = "Grounded SequenceP" ):
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
        
        transportMotn = SequenceP( name = "Move Arm Safely", memory = True )
        transportMotn.add_children( [
            Move_Arm( poseMd1, ctrl = robot, linSpeed = env_var("_ROBOT_FREE_SPEED"), linAccel = env_var("_ROBOT_LIN_ACCEL") ),
            Move_Arm( poseMd2, ctrl = robot, linSpeed = env_var("_ROBOT_FREE_SPEED"), linAccel = env_var("_ROBOT_LIN_ACCEL") ),
            Move_Arm( poseEnd, ctrl = robot, linSpeed = env_var("_ROBOT_FREE_SPEED"), linAccel = env_var("_ROBOT_LIN_ACCEL") ),
        ] )

        self.add_child( transportMotn )

       
class MoveFree_w_Pause( GroundedAction ):
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
        
        transportMotn = SequenceP( name = "Move Arm Safely", memory = True )
        transportMotn.add_children( [
            Move_Arm( poseMd1, ctrl = robot, linSpeed = env_var("_ROBOT_FREE_SPEED"), linAccel = env_var("_ROBOT_LIN_ACCEL") ),
            Move_Arm( poseMd2, ctrl = robot, linSpeed = env_var("_ROBOT_FREE_SPEED"), linAccel = env_var("_ROBOT_LIN_ACCEL") ),
            Move_Arm( poseEnd, ctrl = robot, linSpeed = env_var("_ROBOT_FREE_SPEED"), linAccel = env_var("_ROBOT_LIN_ACCEL") ),
        ] )

        self.add_child( transportMotn )



class Pick( GroundedAction ):
    """ Add object to the gripper payload """
    def __init__( self, args, robot = None, name = None ):

        # ?label ?pose ?prevSupport ?idUp ?idDn
        # label, pose, prevSupport, idUp, idDn = args
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

        # ?label ?pose ?prevSupport ?idUp ?idDn
        # label, pose, prevSupport, idUp, idDn = args
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
        # poseBgn, poseEnd, label, id_ = args
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
    
        checkedMotion = SequenceP( name = "Move Without Dropping", memory = False )
        dropChecker   = Gripper_Aperture_OK( 
            env_var("_BLOCK_SCALE"), 
            margin_m = env_var("_BLOCK_SCALE")*0.50, 
            name = "Check Holding", ctrl = robot  
        )
        transportMotn = SequenceP( name = "Move Object", memory = True )
        transportMotn.add_children( [
            Move_Arm( poseMd1, ctrl = robot, linSpeed = env_var("_ROBOT_HOLD_SPEED"), linAccel = env_var("_ROBOT_LIN_ACCEL") ),
            Move_Arm( poseMd2, ctrl = robot, linSpeed = env_var("_ROBOT_HOLD_SPEED"), linAccel = env_var("_ROBOT_LIN_ACCEL") ),
            Move_Arm( poseEnd, ctrl = robot, linSpeed = env_var("_ROBOT_HOLD_SPEED"), linAccel = env_var("_ROBOT_LIN_ACCEL") ),
        ] )
        checkedMotion.add_children([
            dropChecker,
            transportMotn
        ])

        self.add_child( checkedMotion )



class Place( GroundedAction ):
    """ Let go of gripper payload """
    def __init__( self, args, robot = None, name = None ):

        # ?label ?pose ?support ?idUp ?idDn
        # label, pose, support, idUp, idDn = args
        label, pose, support= args
        
        if name is None:
            name = f"Place {label} at {pose.pose} onto {support}"
        super().__init__( args, robot, name )


        self.add_children([ 
            # Set_Gripper( env_var("_BLOCK_SCALE")*2.0, name = "Release Object", ctrl = robot ),
            # Set_Gripper( env_var("_BLOCK_SCALE")*4.0, name = "Release Object", ctrl = robot ),
            Open_Gripper( ctrl = robot ),
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

        # ?labelUp ?poseUp ?labelDn ?idUp ?idDn
        # labelUp, poseUp, labelDn, idUp, idDn = args
        labelUp, poseUp, labelDn = args
        
        if name is None:
            name = f"Stack {labelUp} at {poseUp.pose} onto {labelDn}"
        super().__init__( args, robot, name )

        self.add_children([ 
            # Set_Gripper( env_var("_BLOCK_SCALE")*2.0, name = "Release Object", ctrl = robot ),
            Open_Gripper( ctrl = robot ),
            Close_Gripper( ctrl = robot ),
            Gripper_Aperture_OK( 
                env_var("_BLOCK_SCALE"), 
                margin_m = env_var("_BLOCK_SCALE")*0.50, 
                name = "Check Placed", ctrl = robot  
            ),
            Open_Gripper( ctrl = robot ),
        ])



########## PLANS ###################################################################################

class Plan( SequenceP ):
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



class PlanParser:
    """ Virtual class to parse PDLS plans into BTs """

    def placeholder( self, plan ):
        """ SHOULD NOT BE USED! """
        return list()


    def __init__( self ):
        """ EMPTY """
        self.outPlan = None


    def display_PDLS_plan( self, plan ):
        """ SHOULD NOT BE USED! """
        raise NotImplementedError( "`display_PDLS_plan` HAS NOT BEEN IMPLEMENTED!" )


    def parse_PDLS_plan( self, plan ):
        """ SHOULD NOT BE USED! """
        raise NotImplementedError( "`parse_PDLS_plan` HAS NOT BEEN IMPLEMENTED!" )
    

    def parse_PDLS_action( self, plan ):
        """ SHOULD NOT BE USED! """
        raise NotImplementedError( "`parse_PDLS_action` HAS NOT BEEN IMPLEMENTED!" )
