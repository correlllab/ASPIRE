########## INIT ####################################################################################

##### Imports #####
### Standard ###
import time, os
now = time.time
from copy import deepcopy
from itertools import count

### Special ###
import numpy as np

### Local ###
from magpie_control.poses import translation_diff
from aspire.env_config import env_var



########## HELPER FUNCTIONS ########################################################################

def extract_np_array_pose( obj_or_arr ):
    """ Return only a copy of the row vector representing the 3D pose """
    if isinstance( obj_or_arr, (list, np.ndarray,) ):
        return np.array( obj_or_arr )
    elif isinstance( obj_or_arr, ObjPose ):
        return np.array( obj_or_arr.pose )
    elif isinstance( obj_or_arr, (GraspObj,) ):
        return extract_np_array_pose( obj_or_arr.pose )
    else:
        return None
    

def extract_pose_as_homog( obj_or_arr, noRot = False ):
    """ Return only a copy of the homogeneous coordinates of the 3D pose """
    bgnPose = extract_np_array_pose( obj_or_arr )
    if len( bgnPose ) == 4:
        rtnPose = bgnPose
    elif len( bgnPose ) == 16:
        rtnPose = np.array( bgnPose ).reshape( (4,4,) )
    else:
        raise ValueError( f"`extract_pose_as_homog`: Poorly formed pose:\n{bgnPose}" )
    if noRot:
        rtnPose[0:3,0:3] = np.eye(3)
    return rtnPose
    

def extract_position( obj_or_arr ):
    """ Return only a copy of the position vector of the 3D pose """
    pose = extract_pose_as_homog( obj_or_arr )
    return pose[0:3,3]


def p_symbol_inside_workspace_bounds( obj_or_arr ):
    """ Return True if inside the bounding box, Otherwise return False """
    posn = extract_position( obj_or_arr )      
    return (env_var("_MIN_X_OFFSET") <= posn[0] <= env_var("_MAX_X_OFFSET")) and (env_var("_MIN_Y_OFFSET") <= posn[1] <= env_var("_MAX_Y_OFFSET")) and (0.0 < posn[2] <= env_var("_MAX_Z_BOUND"))


def euclidean_distance_between_symbols( sym1, sym2 ):
    """ Extract pose component from symbols and Return the linear distance between those poses """
    pose1 = extract_pose_as_homog( sym1 )
    pose2 = extract_pose_as_homog( sym2 )
    return translation_diff( pose1, pose2 )



########## COMPONENTS ##############################################################################

class CPCD:
    """ Color Point Cloud Data """

    def __init__( self, data = None, points = None, colors = None ):
        """ Read CPCD """
        self.points : np.ndarray = None
        self.colors : np.ndarray = None
        if isinstance( data, dict ):
            self.points = data['points']
            self.colors = data['colors']
        else:
            self.points = np.array( points ) if (points is not None) else list()
            self.colors = np.array( colors ) if (colors is not None) else list()

    def __len__( self ):
        """ How many points? """
        return len( self.points )
    
    def copy( self ):
        """ Make a deep copy """
        return CPCD(
            points = self.points.copy(), 
            colors = self.colors.copy()
        )

    def transform( self, homog ):
        """ Transform all of the points with `homog` """
        pnts = np.hstack( (self.points, np.ones( (len( self.points ),1,) )) )
        xPts = np.dot( homog, pnts.transpose() )
        self.points = xPts[:-1,:].transpose()
    

class ObjPose:
    """ Combination of Position and Orientation (Quat) with a Unique Index """
    num = count()

    def __init__( self, pose = None ):
        self.pose  = extract_pose_as_homog( pose ) if (pose is not None) else np.eye(4)
        self.index = next( self.num )

    def row_vec( self ):
        """ Return the vector value of the pose """
        return np.array( self.pose )
    
    def copy( self ):
        """ Return a copy of this pose with a new index """
        return ObjPose( self.pose )
    
    def __repr__( self ):
        """ Text representation """
        return f"<ObjPose {self.index}, Vec: {extract_position( self.pose )} >"
    

class GraspObj:
    """ The concept of a named object at a pose """
    
    num = count()

    def __init__( self, label = None, pose = None, prob = None, score = None, labels = None, ts = None, 
                  count = 0, parent = None, cpcd = None ):
        """ Set components used by planners """
        ### Single Object ###
        self.index  = next( self.num )
        self.label  = label if (label is not None) else env_var("_NULL_NAME")
        self.prob   = prob if (prob is not None) else 0.0 # 2024-07-22: This is for sorting dupes in the planner and is NOT used by PDDLStream
        self.pose   = pose if (pose is not None) else ObjPose( np.eye(4) )
        ### Distribution ###
        self.labels : dict = labels if (labels is not None) else {} # Current belief in each class
        self.visited = False # -------------------------------- Flag: Was this belief associated with a relevant reading
        ### Object Memory ###
        self.ts     = ts if (ts is not None) else now() # --- When was this reading created? [epoch time]
        self.count  = count # ------------------------------- How many bounding boxes generated this reading?
        self.score  = score if (score is not None) else 0.0 # 2024-07-25: This is for sorting dupes in the planner and is NOT used by PDDLStream
        self.LKG    = False # ------------------------------- Flag: Part of the Last-Known-Good collection?
        self.SYM    = False
        self.parent : GraspObj = parent # ------------------------------ Parent object this was copied from
        self.cpcd   = CPCD( cpcd )
        self.meta   = dict()


    def __repr__( self ):
        """ Text representation of noisy reading """
        if self.label != env_var("_NULL_NAME"):
            return f"<GraspObj {self.index} @ {extract_position( self.pose )}, Class: {str(self.label)}, Score: {str(self.score)}>"
        elif len( self.labels ):
            return f"<GraspObj {self.index} @ {extract_position( self.pose )}, Class: {str(self.labels)}, Score: {str(self.score)}>"
        else:
            return f"<GraspObj {self.index} @ {extract_position( self.pose )}, Class: {str(self.label)}, Score: {str(self.score)}>"
    
    
    def get_dict( self ):
        """ Return a verison of the `GraspObj` suitable for a TXT file """
        return {
            'name'  : self.__class__.__name__,
            'time'  : now(),
            'ts'    : self.ts,
            'labels': self.labels,
            'pose'  : extract_pose_as_homog( self.pose ).tolist(),
            'index' : self.index,
            'score' : self.score,
        }
    

    def copy( self, copyParent = False ):
        """ Copy everything but the `parent` """
        ### Basic ###
        rtnObj = GraspObj()
        rtnObj.index  = self.index
        rtnObj.label  = self.label
        rtnObj.prob   = self.prob
        rtnObj.pose   = deepcopy( self.pose )
        ### Distribution ###
        rtnObj.labels  = deepcopy( self.labels ) # Current belief in each class
        rtnObj.visited = self.visited # -------------------------------- Flag: Was this belief associated with a relevant reading
        ### Object Memory ###
        rtnObj.ts     = self.ts   
        rtnObj.count  = self.count
        rtnObj.score  = self.score
        rtnObj.LKG    = self.LKG  
        rtnObj.SYM    = self.SYM  
        rtnObj.parent = None if (not copyParent) else self.parent 
        rtnObj.cpcd   = self.cpcd.copy() 
        rtnObj.meta   = deepcopy( self.meta )
        ### Return ###
        return rtnObj
    

    def copy_child( self ):
        """ Make a copy of this belief """
        rtnObj = self.copy()
        rtnObj.parent = self
        ### Return ###
        return rtnObj
    
    
    def copy_as_LKG( self ):
        """ Make a copy of this belief for the Last-Known-Good collection """
        rtnObj = self.copy()
        rtnObj.LKG = True
        return rtnObj
    

    def get_dict( self ):
        """ Return a verison of the `GraspObj` suitable for a TXT file """
        return {
            'name'  : self.__class__.__name__,
            'time'  : now(),
            'label' : self.label,
            'labels': self.labels,
            'pose'  : extract_pose_as_homog( self.pose ).tolist(),
            'index' : self.index,
            'prob'  : self.prob,
            'score' : self.score,
        }




        


    
