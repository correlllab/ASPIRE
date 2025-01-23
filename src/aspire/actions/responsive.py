########## INIT ####################################################################################

##### Imports #####

### Standard ###
from datetime import datetime
from time import sleep

### Special ###
import numpy as np

from py_trees.common import Status
from py_trees.composites import Sequence

### Local ###
from magpie_control.poses import translation_diff, vec_unit
from aspire.actions.pdls_behaviors import BasicBehavior
from aspire.symbols import env_var
from magpie_control.BT import Move_Arm, Move_Arm_w_Pause


##### Globals && Constants #####
# _DUMMYPOSE = np.eye(4)



########## RESPONSIVE BT HELPER FUNCTIONS ##########################################################


def linear_direction_from_A_to_B( poseA, poseB ):
    """ Return the linear direction vector that points from `poseA` --to-> `poseB` """
    return vec_unit( np.subtract( poseB[0:3,3], poseA[0:3,3] ) )


def translate_pose_along_direction( pose, direction, distance ):
    """ Translate `pose` along `direction` by magnitude `distance` """
    rtnPose = np.array( pose )
    rtnPose[0:3,3] += np.multiply( direction, distance )
    return rtnPose


def line_intersect_plane( rayOrg, rayDir, planePnt, planeNrm, pntParallel = False ):
    # URL:  Intersection point between line and plane: https://github.com/jwatson-CO-edu/scanviewer_ur5-intellisense/blob/46753ef2a14fe90e4ed741c31aa4685c656c3f83/MathGeo.cpp#L647
    # NOTE: Line is defined by a ray lying on line, though this function will return an intersection point on either side of the ray origin
    #       whichever side it occurs
    d = 0.0
    e = 0.00005
    # If the ray direction and plane normal are perpendicular, then the ray is parallel to the plane
    if ( np.abs( np.dot( rayDir, planeNrm ) ) < e ):
        # if the line segment between the plane point and the ray origin has no component in the plane normal direction, plane contains ray
        if ( np.abs( np.subtract( planePnt, rayOrg ).dot( planeNrm ) ) < e ):
            # If ray origin is an appropriate stand-in for the intersection of coplanar line, Return ray origin for sake of convenience
            # 2024-07-27: Use case for this branch is UNCLEAR
            if( pntParallel ):
                return rayOrg
            # else return no-intersection
            else:
                return None
        # else the ray is apart from and parallel to the plane, no intersection to ret
        else:
            return None 
    # Else the ray and plane intersect at exactly one point
    else:
        # 1. Calculate the distance along the ray that the intersection occurs
        d = ( np.subtract( planePnt, rayOrg ).dot( planeNrm ) ) / ( np.dot( rayDir, planeNrm ) )
        return np.add( rayOrg, np.multiply( rayDir, d ) )



########## RESPONSIVE PLANNER BEHAVIOR TREES #######################################################


class PerceiveScene( BasicBehavior ):
    """ Ask the Perception Pipeline to get a reading """

    def __init__( self, args, robot = None, name = None, planner = None ):

        assert planner is not None, "`PerceiveScene` REQUIRES a planner reference!"
        print( f"Planner Type: {type( planner )}" )
        self.planner  = planner
        self.args     = args
        self.needCool = False
        
        if name is None:
            name = f"PerceiveScene with planner {type(planner)}"
        super().__init__(  name = name, ctrl = robot )


    def initialise( self ):
        """ Actually Move """
        super().initialise()
        if self.ctrl.p_moving():
            timeStr = datetime.now().strftime("%H:%M:%S")
            print( f"\n WARN: `PerceiveScene.initialise`: Robot was MOVING at init time: {timeStr}!\n" )
            self.needCool = True
        else:
            self.needCool = False
    

    def update( self ):
        """ Return true if the target reached """
        if self.needCool:
            sleep( env_var("_MOVE_COOLDOWN_S") )
        if self.ctrl.p_moving():
            timeStr = datetime.now().strftime("%H:%M:%S")
            print( f"\n`PerceiveScene.initialise`: Robot was MOVING at UPDATE time: {timeStr}!\n" )
            self.status = Status.FAILURE
        else:
            self.planner.percFunc( 1 )
            sleep( 0.25 )
            if self.planner.chckFunc():
                self.status = Status.SUCCESS
            else:
                self.status = Status.FAILURE
        return self.status



########## SPARE PARTS #############################################################################


# class Interleaved_MoveFree_and_PerceiveScene( GroundedAction ):
#     """ Get a replacement sequence for `MoveFree` that stops for perception at the appropriate times """
#     # FUTURE: PROBABLY MORE SOPHISTICATED SENSORY PLANNING GOES HERE

#     def __init__( self, mfBT, planner, sensePeriod_s, name = None, initSenseStep = True ):
#         """ Init BT """
#         self._VERBOSE = True

#         targetP = mfBT.poseEnd.copy()
        
#         if name is None:
#             # name = f"`Interleaved_MoveFree_and_Perceive`, args: {mfBT.args}"
#             name = f"Interleaved_MoveFree_and_Perceive, {mfBT.name}"
#         super().__init__( mfBT.args, mfBT.ctrl, name )

#         # Init #
#         print( f"Planner Type: {type( planner )}" )
#         self.planner  = planner
#         print( env_var("_ROBOT_FREE_SPEED"), sensePeriod_s )
#         self.distMax  = env_var("_ROBOT_FREE_SPEED") * sensePeriod_s
#         self.zSAFE    = max( env_var("_Z_SAFE"), targetP[2,3] ) # Eliminate (some) silly vertical movements
#         self.bgnShot  = initSenseStep
#         self.mfBT     = mfBT
        
#         # Poses to be Modified at Ticktime #
#         self.targetP = targetP
#         self.pose1up = _DUMMYPOSE.copy()
#         self.pose2up = _DUMMYPOSE.copy()
        
#         # Empty sequences to be built at RUNTIME #
#         self.moveUp = Sequence( "Leg 1", memory = True )
#         self.moveJg = Sequence( "Leg 2", memory = True )
#         self.mvTrgt = Sequence( "Leg 3", memory = True )
        
#         # 2. Move direcly up from the starting pose
#         self.add_child( self.moveUp )
#         # 3. Translate to above the target
#         self.add_child( self.moveJg )
#         # 4. Move to the target pose
#         self.add_child( self.mvTrgt )


#     def initialise( self ):
#         """
#         ( Ticked first time ) or ( ticked not RUNNING ):
#         Generate child sequences with respect to intermittent sensing requirement
#         """

#         def check_and_correct_extreme_closeup( tcpPose ):
#             """ Check and correct `camPose` for insufficient PCD clearance, Return corrected pose """

#             camPose = np.dot( tcpPose, self.mfBT.ctrl.camXform )
#             zzMag   = camPose[2,2]

#             # 1. If downward-facing, then Check for correction
#             if (zzMag < 0.0):
#                 hndZdir = vec_unit( np.dot( camPose, np.array( [0.0, 0.0, -1.0, 1.0,] ) )[0:3] )
#                 camPosn = camPose[0:3,3]
#                 XYintrc = line_intersect_plane( camPosn, hndZdir, [0.0, 0.0, 0.0,], [0.0, 0.0, 1.0,], pntParallel = False )
#                 if XYintrc is None:
#                     raise ValueError( f"`Interleaved_MoveFree_and_PerceiveScene.initialise`: Plane intersection failed from {camPosn} along {hndZdir}" )
#                 else:
#                     if self._VERBOSE:
#                         print( f"\n`Interleaved_MoveFree_and_PerceiveScene.initialise`: Plan move opposite of ray from {camPosn} along {hndZdir}\n" )
#                 dShot = np.linalg.norm( np.subtract( XYintrc, camPosn ) )
#                 if dShot < env_var("_MIN_CAM_PCD_DIST_M"):
#                     dMove     = env_var("_MIN_CAM_PCD_DIST_M") - dShot
#                     backupDir = hndZdir
#                     assert backupDir[2] > 0.0, "Moving DOWN for a better shot is WRONG"
#                     backpMove = np.multiply( backupDir, dMove )
#                     bkTcpPose = np.array( tcpPose )
#                     bkTcpPose[0:3,3] += backpMove
#                     return bkTcpPose
#                 else:
#                     return np.array( tcpPose )
#             # N. Upward-facing poses do not need correction
#             else:
#                 return np.array( tcpPose )


#         # 0. Fetch pose NOW
#         nowPose = self.ctrl.get_tcp_pose()
#         epsilon = 0.00005

#         # 1. Optional: Start with a sensing action, but only when appropriate
#         if self.bgnShot:
#             truShot = check_and_correct_extreme_closeup( nowPose )
#             if translation_diff( truShot, nowPose ) <= epsilon:
#                 self.prepend_child( PerceiveScene( 
#                     self.mfBT.args, 
#                     robot = self.mfBT.ctrl, 
#                     name = "PerceiveScene 1", 
#                     planner = self.planner ) )
        
#         # 5. Compute intermediate poses
#         self.pose1up = nowPose.copy()
#         self.pose1up[2, 3] = self.zSAFE

#         self.pose2up = self.targetP.copy()
#         self.pose2up[2, 3] = self.zSAFE

#         if self._VERBOSE:
#             print( "\n##### Interleaved_MoveFree_and_PerceiveScene #####" )
#             print( "Begin:" )
#             print( nowPose )
#             print( "WP 1:" )
#             print( self.pose1up )
#             print( "WP 2:" )
#             print( self.pose2up )
#             print( "End:" )
#             print( self.targetP )
#             print( "##### Construct Intermittent Paths ... #####\n" )


#         # 6. Construct child sequences && Set them up
#         accumDist = 0.0
#         targtDist = 0.0
#         modloDist = self.distMax # 0.0
#         senseNum  = 1
#         moveNum   = 0
#         dstPoses  = [ self.pose1up, self.pose2up, self.targetP ]
#         seqMoves  = [ self.moveUp , self.moveJg , self.mvTrgt  ]
#         lastPose  = nowPose.copy()
        

#         if self._VERBOSE:
#             print( "\n##### Interleaved_MoveFree_and_PerceiveScene: Interleaved Motion #####" )

#         # A. For every waypoint in this jog action, do
#         for i in range( len(dstPoses) ):

#             # B. Compute leg parameters
#             dstPose_i = dstPoses[i]
#             seqMove_i = seqMoves[i]
#             legDist_i = translation_diff( lastPose, dstPose_i )
#             assert legDist_i > 0.0, "ERROR: Computed ZERO leg distance!"
#             legDirV_i = linear_direction_from_A_to_B( lastPose, dstPose_i )
#             targtDist += legDist_i

#             Nloop = 0

#             # C. While traversing this leg, do
#             while accumDist < targtDist:

#                 # D. Calc remaining distance and Step distance
#                 remnDist = translation_diff( lastPose, dstPose_i )
#                 stepDist = min( [modloDist, remnDist, self.distMax,] )
#                 # E. If step, then Move Action
#                 if stepDist > epsilon:
#                     stepPose = translate_pose_along_direction( lastPose, legDirV_i, stepDist )
#                     stepPose[0:3,0:3] = dstPose_i[0:3,0:3]

#                     seqMove_i.add_child(  Move_Arm( stepPose, ctrl = self.ctrl, linSpeed = env_var("_ROBOT_FREE_SPEED") )  )
#                     moveNum += 1
#                     if self._VERBOSE:
#                         print( f"WP {moveNum}:" )
#                         print( stepPose )

#                     modloDist -= stepDist
#                     accumDist += stepDist
#                     lastPose  = stepPose
#                 # F. Else Sense Action
#                 else:
#                     # G. Check for extreme closeup, 2024-07-27: Taking PCDs too close results in BAD OBJECT POSES
#                     # FUTURE: PROBABLY MORE SOPHISTICATED SENSORY PLANNING GOES HERE
#                     shotPose = check_and_correct_extreme_closeup( lastPose )
#                     # H. If too close, then back up, take shot, and return to planned pose
#                     if translation_diff( shotPose, lastPose ) > epsilon:
#                         seqMove_i.add_children([  
#                             Move_Arm( shotPose, name = "Camera Backup", ctrl = self.ctrl, linSpeed = env_var("_ROBOT_FREE_SPEED") ),
#                             PerceiveScene( self.args, 
#                                            self.ctrl, 
#                                            name = f"PerceiveScene {senseNum}", planner = self.planner ),
#                             Move_Arm( lastPose, name = "Camera Return", ctrl = self.ctrl, linSpeed = env_var("_ROBOT_FREE_SPEED") ),
#                         ])
#                     # I. Else no correction required
#                     else:
#                         seqMove_i.add_child(  
#                             PerceiveScene( self.args, 
#                                            self.ctrl, 
#                                            name = f"PerceiveScene {senseNum}", planner = self.planner )  
#                         )
#                     modloDist = self.distMax # No assumptions violated here

#                 Nloop += 1
#                 if Nloop >= 10:
#                     break # HACK: THIS IS PROBABLY VERY BAD ???
                        
#         # J. Prep sequence
#         for chld in self.children:
#             chld.setup_with_descendants()

#         if self._VERBOSE:
#             print( f"##### Constructed {moveNum} motions! #####\n" )