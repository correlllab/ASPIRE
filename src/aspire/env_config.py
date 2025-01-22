########## INIT ####################################################################################
import os, json, pickle
import codecs

import numpy as np



########## SETTINGS ################################################################################

def env_sto( varKey, varVal ):
    """ Serialize and save the env var """
    # https://stackoverflow.com/a/30469744
    os.environ[ varKey ] = codecs.encode( pickle.dumps( varVal ), "base64").decode()


def env_var( varKey ):
    """ Load and parse the env var """
    try:
        # https://stackoverflow.com/a/30469744
        return pickle.loads(codecs.decode( os.environ[ varKey ].encode(), "base64"))
    except KeyError:
        print( f"There was no {varKey} in the environment!" )
        return None
    except json.JSONDecodeError:
        print( f"Bad data stored at {varKey}! Cannot decode!" )
        return None


def set_global_env():
    """ Non-specific options """
    np.set_printoptions(
        edgeitems =  16, # Number of items before ...
        linewidth = 200, 
        formatter = dict( float = lambda x: "%.5g" % x ) 
    )
    env_sto( "_VERBOSE", True )



########## GRAPHICS ################################################################################

def set_grahpics_env():
    """ Graphical Debugging Options """
    env_sto( "_USE_GRAPHICS", True ) 
    # env_sto( "_RECORD_SYM_SEQ", True )
    env_sto( "_BLOCK_ALPHA", 1.0 )
    env_sto( "_CLR_TABLE", {
        'red': [1.0, 0.0, 0.0,],
        'ylw': [1.0, 1.0, 0.0,],
        'blu': [0.0, 0.0, 1.0,],
        'grn': [0.0, 1.0, 0.0,],
        'orn': [1.0, 0.5, 0.0,],
        'vio': [0.5, 0.0, 1.0,],
        env_var("_NULL_NAME")[:3]: [1.0, 1.0, 1.0,],
    } )



########## OBJECTS #################################################################################

def set_object_env():
    """ Object Names and Scales """
    env_sto( "_NULL_NAME", "NOTHING" ) 
    env_sto( "_ONLY_RED", False ) 
    env_sto( "_ONLY_PRIMARY", False ) 
    env_sto( "_ONLY_SECONDARY", False ) 
    env_sto( "_ONLY_EXPERIMENT", True ) 
    

    env_sto( "_BLOCK_NAMES", [
        # 'redBlock', 
        'ylwBlock', 
        'bluBlock', 
        'grnBlock', 
        # 'ornBlock', 
        # 'vioBlock', 
        env_var("_NULL_NAME"),] )
        

    env_sto( "_POSE_DIM", 7 ) 
    env_sto( "_ACTUAL_NAMES", env_var( "_BLOCK_NAMES" )[:-1] ) 
    env_sto( "_N_CLASSES", len( env_var("_BLOCK_NAMES" )) ) 
    env_sto( "_N_ACTUAL", len( env_var("_ACTUAL_NAMES" )) ) 


    # env_sto( "_BLOCK_SCALE", 0.025 ) # Medium Wooden Blocks (YCB)
    env_sto( "_BLOCK_SCALE", 0.040 ) # 3D Printed Blocks

    env_sto( "_ACCEPT_POSN_ERR", 0.55*env_var( "_BLOCK_SCALE" ) ) # 0.50 # 0.65 # 0.75 # 0.85 # 1.00
    env_sto( "_Z_SAFE", 0.400 )
    env_sto( "_MIN_SEP", 0.85*env_var( "_BLOCK_SCALE" ) ) # 0.40 # 0.60 # 0.70 # 0.75



########## ROBOT ###################################################################################

def set_workspace_env():
    """ Set working envelope """

    # env_sto( "_Z_TABLE"     ,  0.032 ) 
    env_sto( "_Z_TABLE"     ,  0.000 ) 

    env_sto( "_SPACE_EXPAND",  0.050 ) 
    env_sto( "_MIN_X_OFFSET", -0.468 - env_var( "_SPACE_EXPAND" ) )
    env_sto( "_MAX_X_OFFSET", -0.103 + env_var( "_SPACE_EXPAND" ) )
    env_sto( "_MIN_Y_OFFSET", -0.625 - env_var( "_SPACE_EXPAND" ) ) 
    env_sto( "_MAX_Y_OFFSET", -0.272 + env_var( "_SPACE_EXPAND" ) )
    env_sto( "_MAX_Z_BOUND" , env_var( "_BLOCK_SCALE" )*4.0 )
    env_sto( "_X_WRK_SPAN"  , env_var( "_MAX_X_OFFSET" ) - env_var( "_MIN_X_OFFSET" ) )
    env_sto( "_Y_WRK_SPAN"  , env_var( "_MAX_Y_OFFSET" ) - env_var( "_MIN_Y_OFFSET" ) )


def set_robot_env():
    """ Set robot working parameters for this problem """
    set_workspace_env()
    env_sto( "_ROBOT_FREE_SPEED",  0.125 ) 
    env_sto( "_ROBOT_HOLD_SPEED",  0.125 )
    env_sto( "_MOVE_COOLDOWN_S",  0.5 )
    env_sto( "_BT_UPDATE_HZ"  ,  5.0 )
    env_sto( "_BT_ACT_TIMEOUT_S",  20.0 )




########## CAMERA ##################################################################################

def set_camera_env():
    """ Set camera view params """
    env_sto( "_D405_FOV_H_DEG",  84.0 )
    env_sto( "_D405_FOV_V_DEG",  58.0 )
    env_sto( "_D405_FOV_D_M",  0.920 )
    env_sto( "_MIN_CAM_PCD_DIST_M", 0.075 + env_var( "_BLOCK_SCALE" ) * len( env_var( "_ACTUAL_NAMES" ) ) )



########## >>>>> BLOCKS <<<<< ######################################################################

def set_blocks_env():
    """ Store all params for the blocks problem """
    set_global_env()
    set_object_env()
    set_robot_env()
    set_camera_env()
    set_grahpics_env()