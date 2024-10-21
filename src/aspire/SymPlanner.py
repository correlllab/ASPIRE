"""
TaskPlanner.py
Correll Lab, CU Boulder
Contains the Baseline and Responsive Planners described in FIXME: INSERT PAPER REF AND DOI
Version 2024-07
Contacts: {james.watson-2@colorado.edu,}
"""
########## INIT ####################################################################################

##### Imports #####

### Standard ###
import sys, time, os
now = time.time
from pprint import pprint
from random import random
from traceback import print_exc, format_exc
from datetime import datetime

### Special ###
import numpy as np
from py_trees.common import Status

### ASPIRE ###
from aspire.symbols import ( ObjPose, GraspObj, extract_pose_as_homog, euclidean_distance_between_symbols )
from aspire.actions import ( display_PDLS_plan, get_BT_plan, get_BT_plan_until_block_change, )

### ASPIRE::PDDLStream ### 
from aspire.pddlstream.pddlstream.utils import read, INF, get_file_path
from aspire.pddlstream.pddlstream.language.generator import from_gen_fn, from_test
from aspire.pddlstream.pddlstream.language.constants import print_solution, PDDLProblem
from aspire.pddlstream.pddlstream.algorithms.meta import solve




##### Planner #############################################################

class SymPlanner:
    """ Basic task planning loop """

    ##### Init ############################################################

    def reset_symbols( self ):
        """ Erase belief memory """
        self.symbols : list[GraspObj] = list() # ------- Determinized beliefs
        self.facts   = list() # ------- Grounded predicates


    def reset_state( self ):
        """ Erase problem state """
        self.status = Status.INVALID # Running status
        self.task   = None # --------- Current task definition
        self.goal   = tuple() # ------ Current goal specification
        self.grasp  = list() # ------- ? NOT USED ?
        self.nxtAct = None
        self.action = None


    def __init__( self, domainPath, streamPath ):
        """ Create a pre-determined collection of poses and plan skeletons """
        self.reset_symbols()
        self.reset_state()
        # DEATH MONITOR
        self.noSoln =  0
        self.nonLim = 10
        self.domainPath = domainPath 
        self.streamPath = streamPath 


    ##### Stream Helpers ##################################################

    def get_grounded_pose_or_new( self, homog ):
        """ If there is a `Waypoint` approx. to `homog`, then return it, Else create new `ObjPose` """
        for fact in self.facts:
            if fact[0] == 'Waypoint' and ( euclidean_distance_between_symbols( homog, fact[1] ) <= os.environ["_ACCEPT_POSN_ERR"]):
                return fact[1]
        return ObjPose( homog )


    def p_grounded_fact_pose( self, poseOrObj ):
        """ Does this exist as a `Waypoint`? """
        homog = extract_pose_as_homog( poseOrObj )
        for fact in self.facts:
            if fact[0] == 'Waypoint' and ( euclidean_distance_between_symbols( homog, fact[1] ) <= os.environ["_ACCEPT_POSN_ERR"]):
                return True
        return False



    ##### Task Planning Helpers ###########################################

    def pddlstream_from_problem( self, pdls_stream_map = None ):
        """ Set up a PDDLStream problem with the UR5 """

        domain_pddl  = read( self.domainPath )
        stream_pddl  = read( self.streamPath )
        constant_map = {}
        stream_map = pdls_stream_map if ( pdls_stream_map is not None ) else dict()

        if os.environ["_VERBOSE"]:
            print( "About to create problem ... " )

        return PDDLProblem( domain_pddl, constant_map, stream_pddl, stream_map, self.facts, self.goal )
    

    def set_goal( self, nuGoal ):
        """ Set the goal """

        self.goal = nuGoal

        if os.environ["_VERBOSE"]:
            print( f"\n### Goal ###" )
            pprint( self.goal )
            print()


    def p_failed( self ):
        """ Has the system encountered a failure? """
        return (self.status == Status.FAILURE)
    

    
                

    ##### Noisy Task Monitoring ###########################################

    def get_labeled_symbol( self, label ):
        """ If a block with `label` was sampled, then return a reference to it, Otherwise return `None` """
        for sym in self.symbols:
            if sym.label == label:
                return sym
        return None
    

    def get_grounded_fact_pose_or_new( self, homog ):
        """ If there is a `Waypoint` approx. to `homog`, then return it, Else create new `ObjPose` """ 
        for fact in self.facts:
            if fact[0] == 'Waypoint' and (euclidean_distance_between_symbols( homog, fact[1] ) <= os.environ["_ACCEPT_POSN_ERR"]):
                return fact[1]
            if fact[0] == 'GraspObj' and (euclidean_distance_between_symbols( homog, fact[2] ) <= os.environ["_ACCEPT_POSN_ERR"]):
                return fact[2]
        return ObjPose( homog )
    
    
    


    def check_goal_objects( self, goal, symbols : list[GraspObj] ):
        """ Return True if the labels mentioned in the goals are a subset of the determinized symbols """
        goalSet = set([])
        symbSet = set( [sym.label for sym in symbols] )
        for g in goal:
            if isinstance( g, (tuple, list) ):
                prdName = g[0]
                if prdName == 'GraspObj':
                    goalSet.add( g[1] )
                elif prdName == 'Supported':
                    goalSet.add( g[1] )
                    goalSet.add( g[2] )
                else:
                    continue
        return (goalSet <= symbSet)
    

    def object_exists( self, label ):
        """ See if a fact already covers this block """
        for f in self.facts:
            if (f[0] == 'GraspObj') and (f[1] == label):
                return True
        return False


    def plan_task( self ):
        """ Attempt to solve the symbolic problem """

        self.task = self.pddlstream_from_problem()

        # self.logger.log_event( "Begin Solver" )

        try:
            
            solution = solve( 
                self.task, 
                algorithm      = "adaptive", 
                unit_costs     = True, 
                unit_efforts   = True, 
                reorder        = True,
                initial_complexity = 2,
            )

            print( "Solver has completed!\n\n\n" )
            print_solution( solution )
            
        except Exception as ex:
            # self.logger.log_event( "SOLVER FAULT", format_exc() )
            self.status = Status.FAILURE
            print_exc()
            solution = (None, None, None)
            self.noSoln += 1 # DEATH MONITOR

        plan, cost, evaluations = solution

        if (plan is not None) and len( plan ):
            display_PDLS_plan( plan )
            self.currPlan = plan
            self.nxtAct   = get_BT_plan_until_block_change( plan, self, os.environ["_UPDATE_PERIOD_S"] )
            self.action   = get_BT_plan( plan, self, os.environ["_UPDATE_PERIOD_S"] )
            self.noSoln   = 0 # DEATH MONITOR
        else:
            self.noSoln += 1 # DEATH MONITOR
            # self.logger.log_event( "NO SOLUTION" )
            self.status = Status.FAILURE


    

    def p_fact_match_noisy( self, pred ):
        """ Search grounded facts for a predicate that matches `pred` """
        for fact in self.facts:
            if pred[0] == fact[0]:
                same = True 
                for i in range( 1, len( pred ) ):
                    if type( pred[i] ) != type( fact[i] ):
                        same = False 
                        break
                    elif isinstance( pred[i], str ) and (pred[i] != fact[i]):
                        same = False
                        break
                    elif (pred[i].index != fact[i].index):
                        same = False
                        break
                if same:
                    return True
        return False

    
    def validate_goal_noisy( self, goal ):
        """ Check if the system believes the goal is met """
        if goal[0] == 'and':
            for g in goal[1:]:
                if not self.p_fact_match_noisy( g ):
                    return False
            return True
        else:
            raise ValueError( f"Unexpected goal format!: {goal}" )


    