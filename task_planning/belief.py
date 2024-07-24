########## INIT ####################################################################################

###### Imports ######

### Standard ###
import atexit, sys
from time import sleep

### Special ###
import numpy as np
import open3d as o3d
import pyglet
from pyglet.window import Window

### Local ###
from env_config import ( _BLOCK_SCALE, _N_CLASSES, _CONFUSE_PROB, _NULL_NAME, _NULL_THRESH, 
                         _BLOCK_NAMES, _VERBOSE, _MIN_SEP, )
from task_planning.utils import ( extract_dct_values_in_order, sorted_obj_labels, )
from task_planning.symbols import GraspObj, extract_pose_as_homog
sys.path.append( "../graphics/" )
from graphics.draw_beliefs import generate_belief_geo
sys.path.append( "../magpie/" )
from magpie.poses import translation_diff

########## HELPER FUNCTIONS ########################################################################

def d_between_obj_poses( obj1, obj2 ):
    """ Calculate the translation between poses in the same frame """
    return np.linalg.norm( np.subtract( obj1.pose[:3], obj2.pose[:3] ) )


def extract_class_dist_in_order( obj, order = _BLOCK_NAMES ):
    """ Get the discrete class distribution, in order according to environment variable """
    if isinstance( obj, dict ):
        return np.array( extract_dct_values_in_order( obj, order ) )
    else:
        return np.array( extract_dct_values_in_order( obj.labels, order ) )


def extract_class_dist_sorted_by_key( obj ):
    """ Get the discrete class distribution, sorted by key name """
    return np.array( extract_dct_values_in_order( obj.labels, sorted_obj_labels( obj ) ) )


def extract_class_dist_in_order( obj, order = _BLOCK_NAMES ):
    """ Get the discrete class distribution, in order according to environment variable """
    return np.array( extract_dct_values_in_order( obj.labels, order ) )

def vis_window( geo ):
    """ Open3D SUUUUUUUUUCKS """
    # https://github.com/isl-org/Open3D/issues/3489#issuecomment-863704146
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry( geo )
    ctr = vis.get_view_control()  # Everything good
    print( f"Opened window: {ctr}" )
    vis.run()

########## BELIEFS #################################################################################

class ObjectMemory:
    """ Attempt to maintain recent and constistent object beliefs based on readings from the vision system """

    def reset_beliefs( self ):
        """ Remove all references to the beliefs, then erase the beliefs """
        self.beliefs = []
        # self.vis.clear_geometries()


    def __init__( self ):
        """ Set belief containers """
        # https://github.com/isl-org/Open3D/issues/2006#issuecomment-701340077
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window( window_name = "Planner: Current Belief", width = 100, height = 700, visible = True )
        self.reset_beliefs()
        # atexit.register( self.destroy )


    # def destroy( self ):
    #     """ Close window and cleanup """
    #     self.vis.destroy_window()


    def display_belief_geo( self ):
        # win = pyglet.window.Window()
        # pyglet.app.run()
        # self.vis.clear_geometries()
        # self.vis.add_geometry( geo )
        # self.vis.poll_events()
        # self.vis.update_renderer()
        geo = generate_belief_geo( self.beliefs )
        o3d.visualization.draw_geometries( geo )
        # vis_window( geo )
        

    
    def accum_evidence_for_belief( self, evidence, belief ):
        """ Use Bayesian multiclass update on `belief`, destructive """
        evdnc = extract_class_dist_in_order( evidence )
        prior = extract_class_dist_in_order( belief   )
        keys  = _BLOCK_NAMES
        pstrr = multiclass_Bayesian_belief_update( 
            get_confusion_matx( _N_CLASSES, confuseProb = _CONFUSE_PROB ), 
            prior, 
            evdnc 
        )
        for i, key in enumerate( keys ):
            belief.labels[ key ] = pstrr[i]


    def integrate_one_reading( self, objReading, maxRadius = 3.0*_BLOCK_SCALE ):
        """ Fuse this belief with the current beliefs """
        relevant = False

        # 1. Determine if this belief provides evidence for an existing belief
        dMin     = 1e6
        belBest  = None
        for belief in self.beliefs:
            d = d_between_obj_poses( objReading, belief )
            if (d < maxRadius) and (d < dMin):
                dMin     = d
                belBest  = belief
                relevant = True

        if relevant:
            belBest.visited = True
            self.accum_evidence_for_belief( objReading, belBest )
            # belBest.pose = np.array( objReading.pose ) # WARNING: ASSUME THE NEW NEAREST POSE IS CORRECT!
            belBest.pose = objReading.pose # WARNING: ASSUME THE NEW NEAREST POSE IS CORRECT!

        # 2. If this evidence does not support an existing belief, it is a new belief
        else:
            self.beliefs.append( objReading.copy() ) 

        # N. Return whether the reading was relevant to an existing belief
        return relevant
    

    def integrate_null( self, belief ):
        """ Accrue a non-observation """
        labels = get_confused_class_reading( _NULL_NAME, _CONFUSE_PROB, _BLOCK_NAMES )
        cnfMtx = get_confusion_matx( _N_CLASSES, _CONFUSE_PROB )
        priorB = [ belief.labels[ label ] for label in _BLOCK_NAMES ] 
        evidnc = [ labels[ label ] for label in _BLOCK_NAMES ]
        updatB = multiclass_Bayesian_belief_update( cnfMtx, priorB, evidnc )
        belief.labels = {}
        for i, name in enumerate( _BLOCK_NAMES ):
            belief.labels[ name ] = updatB[i]
    

    def unvisit_beliefs( self ):
        """ Set visited flag to False for all beliefs """
        for belief in self.beliefs:
            belief.visited = False


    def erase_dead( self ):
        """ Erase all beliefs and cached symbols that no longer have relevancy """
        retain = []
        for belief in self.beliefs:
            if belief.labels[ _NULL_NAME ] < _NULL_THRESH:
                retain.append( belief )
            elif _VERBOSE:
                print( f"{str(belief)} DESTROYED!" )
        self.beliefs = retain


    def decay_beliefs( self ):
        """ Destroy beliefs that have accumulated too many negative indications """
        for belief in self.beliefs:
            if not belief.visited:
                self.integrate_null( belief )
        self.erase_dead()
        self.unvisit_beliefs()


    def belief_update( self, evdncLst ):
        """ Gather and aggregate evidence """

        ## Integrate Beliefs ##
        cNu = 0
        cIn = 0
        self.unvisit_beliefs()
        
        if not len( self.beliefs ):
            # WARNING: ASSUMING EACH OBJECT IS REPRESENTED BY EXACTLY 1 READING
            for objEv in evdncLst:
                self.beliefs.append( objEv.copy() )
        else:
            for objEv in evdncLst:
                if self.integrate_one_reading( objEv ):
                    cIn += 1
                else:
                    cNu += 1
            ## Decay Irrelevant Beliefs ##
            self.decay_beliefs()

        if _VERBOSE:
            if (cNu or cIn):
                print( f"\t{cNu} new object beliefs this iteration!" )
                print( f"\t{cIn} object beliefs updated!" )
            else:
                print( f"\tNO belief update!" )
        
        if _VERBOSE:
            print( f"Total Beliefs: {len(self.beliefs)}" )
            for bel in self.beliefs:
                print( f"\t{bel}" )
            print()


    def most_likely_objects( self, N = 1, cleanDupes = 0, cleanCollision = 0 ):
        """ Get the `N` most likely combinations of object classes """

        def p_unique_labels( objLst ):
            """ Return true if there are as many classes as there are objects """
            lbls = set([sym.label for sym in objLst])
            if _NULL_NAME in lbls: # WARNING: HACK
                return False
            return len( lbls ) == len( objLst )
        
        # def clean_dupes( objLst ):
        #     """ Return a version of `objLst` with duplicate objects removed """
        #     dctMax = {}
        #     for sym in objLst:
        #         if not sym.label in dctMax:
        #             dctMax[ sym.label ] = sym
        #         elif sym.prob > dctMax[ sym.label ].prob:
        #             dctMax[ sym.label ] = sym
        #     return list( dctMax.values() )
        
        # def clean_colliding( objLst ):
        #     rtnLst = []
        #     M = len(objLst)
        #     for i, sym_i in enumerate( objLst ):
        #         collides = False
        #         for j in range( i+1, M ):
        #             sym_j = objLst[j]
        #             pos_i = extract_pose_as_homog( sym_i )
        #             pos_j = extract_pose_as_homog( sym_j )
        #             if translation_diff( pos_i, pos_j ) < _MIN_SEP:
        #                 collides = True
        #                 if sym_i.prob > sym_j.prob:
        #                     rtnLst.append( sym_i )
        #         if not collides:
        #             rtnLst.append( sym_i )
        #     return rtnLst


        ## Init ##
        comboList = [ [1.0,[],], ]
        ## Generate all class combinations with joint probabilities ##
        for bel in self.beliefs:
            nuCombos = []
            for combo_i in comboList:
                for label_j, prob_j in bel.labels.items():
                    prob_ij = combo_i[0] * prob_j

                    # objc_ij = GraspObj( label = label_j, pose = np.array( bel.pose ) )
                    objc_ij = GraspObj( label = label_j, pose = bel.pose, prob = prob_j )
                    
                    nuCombos.append( [prob_ij, combo_i[1]+[objc_ij,],] )
            comboList = nuCombos
        ## Sort all class combinations with decreasing probabilities ##
        comboList.sort( key = (lambda x: x[0]), reverse = True )
        ## Return top combos ##
        if N == 1:
            for combo in comboList:
                # if cleanDupes:
                #     if cleanCollision:
                #         return clean_colliding( clean_dupes( combo[1] ) )
                #     else:
                #         return clean_dupes( combo[1] )
                # elif cleanCollision:
                #     return clean_colliding( combo[1] )
                # el
                if p_unique_labels( combo[1] ):
                    return combo[1]
            return list()
        elif N > 1:
            rtnCombos = []
            for i in range(N):
                rtnCombos.append( comboList[i][1] )
            return rtnCombos
        else:
            return list()
