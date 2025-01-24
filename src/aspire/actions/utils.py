########## INIT ####################################################################################

##### Imports #####

### Standard ###
from datetime import datetime
from time import sleep

### Special ###
import numpy as np

### Local ###
from magpie_control.poses import vec_unit


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



