########## INIT ####################################################################################
import os
from random import random

import numpy as np

from magpie.poses import repair_pose

from aspire.env_config import set_blocks_env, env_var, env_sto

set_blocks_env()

_poseGrn = np.eye(4)
_poseGrn[0:3,3] = [ env_var("_MIN_X_OFFSET") +env_var("_X_WRK_SPAN")/2.0, env_var("_MIN_Y_OFFSET")+env_var("_Y_WRK_SPAN")/2.0, 0.5*env_var("_BLOCK_SCALE"), ]

from aspire.symbols import ObjPose, extract_pose_as_homog, euclidean_distance_between_symbols
from aspire.utils import diff_norm



########## ENVIRONMENT #############################################################################

def set_block_poses_env():
    """ Hard-coded poses for the blocks problem """

    env_sto( "_trgtGrn", ObjPose( _poseGrn ) ) 
    env_sto( "_N_XTRA_SPOTS", 3 ) 


    env_sto( "_temp_home", np.array( [[-1.000e+00, -1.190e-04,  2.634e-05, -2.540e-01],
                                      [-1.190e-04,  1.000e+00, -9.598e-06, -4.811e-01],
                                      [-2.634e-05, -9.601e-06, -1.000e+00,  4.022e-01],
                                      [ 0.000e+00,  0.000e+00,  0.000e+00,  1.000e+00],] ) )

    env_sto( "_GOOD_VIEW_POSE", repair_pose( np.array( [[-0.749, -0.513,  0.419, -0.428,],
                                                        [-0.663,  0.591, -0.46 , -0.273,],
                                                        [-0.012, -0.622, -0.783,  0.337,],
                                                        [ 0.   ,  0.   ,  0.   ,  1.   ,],] ) ) )

    env_sto( "_HIGH_VIEW_POSE", repair_pose( np.array( [[-0.709, -0.455,  0.539, -0.51 ],
                                                        [-0.705,  0.442, -0.554, -0.194],
                                                        [ 0.014, -0.773, -0.635,  0.332],
                                                        [ 0.   ,  0.   ,  0.   ,  1.   ],] ) ) )

    env_sto( "_HIGH_TWO_POSE", repair_pose( np.array( [[-0.351, -0.552,  0.756, -0.552],
                                                       [-0.936,  0.194, -0.293, -0.372],
                                                       [ 0.015, -0.811, -0.585,  0.283],
                                                       [ 0.   ,  0.   ,  0.   ,  1.   ],] ) ) )


########## HELPER FUNCTIONS ########################################################################


def rand_table_pose():
    """ Return a random pose in the direct viscinity if the robot """
    rtnPose = np.eye(4)
    rtnPose[0:3,3] = [ 
        env_var("_MIN_X_OFFSET") + 0.5*env_var("_X_WRK_SPAN") + 0.5*env_var("_X_WRK_SPAN")*random(), 
        env_var("_MIN_Y_OFFSET") + 0.5*env_var("_Y_WRK_SPAN") + 0.5*env_var("_Y_WRK_SPAN")*random(), 
        env_var("_BLOCK_SCALE")/2.0,
    ]
    return rtnPose



########## STREAM CREATORS && HELPER FUNCTIONS #####################################################

class BlockFunctions:

    def __init__( self, planner ):
        """ Attach to Planner """
        self.planner = planner

    ##### Stream Creators #################################################

    def get_above_pose_stream( self ):
        """ Return a function that returns poses """

        def stream_func( *args ):
            """ A function that returns poses """

            if os.environ["_VERBOSE"]:
                print( f"\nEvaluate ABOVE LABEL stream with args: {args}\n" )

            objcName = args[0]

            for sym in self.planner.symbols:
                if sym.label == objcName:
                    upPose = extract_pose_as_homog( sym )
                    upPose[2,3] += env_var("_BLOCK_SCALE")

                    rtnPose = self.planner.get_grounded_fact_pose_or_new( upPose )
                    print( f"FOUND a pose {rtnPose} supported by {objcName}!" )

                    yield (rtnPose,)

        return stream_func


    def get_placement_pose_stream( self ):
        """ Return a function that returns poses """

        def stream_func( *args ):
            """ A function that returns poses """

            if os.environ["_VERBOSE"]:
                print( f"\nEvaluate PLACEMENT POSE stream with args: {args}\n" )

            placed   = False
            testPose = None
            while not placed:
                testPose  = rand_table_pose()
                print( f"\t\tSample: {testPose}" )
                for sym in self.planner.symbols:
                    if euclidean_distance_between_symbols( testPose, sym ) < ( os.environ["_MIN_SEP"] ):
                        collide = True
                        break
                if not collide:
                    placed = True
            yield (ObjPose(testPose),)

        return stream_func


    def get_free_placement_test( self ):
        """ Return a function that checks placement poses """

        def test_func( *args ):
            """ a function that checks placement poses """
            print( f"\nEvaluate PLACEMENT test with args: {args}\n" )
            # ?pose
            chkPose   = args[0]
            print( f"Symbols: {self.planner.symbols}" )

            for sym in self.planner.symbols:
                if euclidean_distance_between_symbols( chkPose, sym ) < ( os.environ["_MIN_SEP"] ):
                    print( f"PLACEMENT test FAILURE\n" )
                    return False
            print( f"PLACEMENT test SUCCESS\n" )
            return True
        
        return test_func
    

    ##### Helper Functions ################################################

    def ground_relevant_predicates( self ):
        """ Scan the environment for evidence that the task is progressing, using current beliefs """
        rtnFacts = []
        ## Gripper Predicates ##
        if len( self.planner.grasp ):
            for graspedLabel in self.planner.grasp:
                # [hndl,pDif,bOrn,] = grasped
                # labl = self.world.get_handle_name( hndl )
                rtnFacts.append( ('Holding', graspedLabel,) )
        else:
            rtnFacts.append( ('HandEmpty',) )
        ## Obj@Loc Predicates ##
        # A. Check goals
        for g in self.planner.goal[1:]:
            if g[0] == 'GraspObj':
                pLbl = g[1]
                pPos = g[2]
                tObj = self.planner.get_labeled_symbol( pLbl )
                if (tObj is not None) and (euclidean_distance_between_symbols( pPos, tObj ) <= os.environ["_ACCEPT_POSN_ERR"]):
                    rtnFacts.append( g ) # Position goal met
        # B. No need to ground the rest

        ## Support Predicates && Blocked Status ##
        # Check if `sym_i` is supported by `sym_j`, blocking `sym_j`, NOTE: Table supports not checked
        supDices = set([])
        for i, sym_i in enumerate( self.planner.symbols ):
            for j, sym_j in enumerate( self.planner.symbols ):
                if i != j:
                    lblUp = sym_i.label
                    lblDn = sym_j.label
                    posUp = extract_pose_as_homog( sym_i )
                    posDn = extract_pose_as_homog( sym_j )
                    xySep = diff_norm( posUp[0:2,3], posDn[0:2,3] )
                    zSep  = posUp[2,3] - posDn[2,3] # Signed value
                    if ((xySep <= 1.65*os.environ["_BLOCK_SCALE"]) and (1.65*os.environ["_BLOCK_SCALE"] >= zSep >= 0.9*os.environ["_BLOCK_SCALE"])):
                        supDices.add(i)
                        rtnFacts.extend([
                            ('Supported', lblUp, lblDn,),
                            ('Blocked', lblDn,),
                            ('PoseAbove', self.get_grounded_fact_pose_or_new( posUp ), lblDn,),
                        ])
        for i, sym_i in enumerate( self.symbols ):
            if i not in supDices:
                rtnFacts.extend( [
                    ('Supported', sym_i.label, 'table',),
                    ('PoseAbove', self.get_grounded_fact_pose_or_new( sym_i ), 'table',),
                ] )
        ## Where the robot at? ##
        robotPose = ObjPose( self.robot.get_tcp_pose() )
        rtnFacts.extend([ 
            ('AtPose', robotPose,),
            ('WayPoint', robotPose,),
        ])
        ## Return relevant predicates ##
        return rtnFacts
    

    def allocate_table_swap_space( self, Nspots = None ):
        """ Find some open poses on the table for performing necessary swaps """
        if Nspots is None:
            Nspots = env_var("_N_XTRA_SPOTS")
        rtnFacts  = []
        freeSpots = []
        occuSpots = [extract_pose_as_homog( sym ) for sym in self.planner.symbols]
        while len( freeSpots ) < Nspots:
            nuPose = rand_table_pose()
            print( f"\t\tSample: {nuPose}" )
            collide = False
            for spot in occuSpots:
                if euclidean_distance_between_symbols( spot, nuPose ) < ( env_var("_MIN_SEP") ):
                    collide = True
                    break
            if not collide:
                freeSpots.append( ObjPose( nuPose ) )
                occuSpots.append( nuPose )
        for objPose in freeSpots:
            rtnFacts.extend([
                ('Waypoint', objPose,),
                ('Free', objPose,),
                ('PoseAbove', objPose, 'table'),
            ])
        return rtnFacts