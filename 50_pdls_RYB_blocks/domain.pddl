(define (domain pick-and-place)
;   (:requirements :strips :equality)
  (:requirements :strips :equality :negative-preconditions :derived-predicates)
  (:predicates

    (Pose ?o ?p)
    (Grasp ?o ?g)
    (Conf ?q)
    (Traj ?t)

    (Kin ?o ?p ?g ?q ?t)

    (Stackable ?o ?r)
    
    (Graspable ?o)
    
    (FreeMotion ?q1 ?t ?q2)
    (HoldingMotion ?q1 ?t ?q2 ?o ?g)
    (Supported ?o ?p ?r)
    

    (TrajCollision ?t ?o2 ?p2)
    (CFreePosePose ?o ?p ?o2 ?p2)
    (CFreeApproachPose ?o ?p ?g ?o2 ?p2)
    (CFreeTrajPose ?t ?o2 ?p2)

    (AtPose ?o ?p)
    (AtGrasp ?o ?g)
    (HandEmpty)
    (AtConf ?q)
    (CanMove)
    (On ?o ?r)
    (Holding ?o)
  )

  (:action move_free
    :parameters (?q1 ?q2 ?t)
    :precondition (and (FreeMotion ?q1 ?t ?q2)
                       (AtConf ?q1) (HandEmpty) (CanMove)
                  )
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1))
    )
  )

  (:action pick
    :parameters (?o ?p ?g ?q ?t)
    :precondition (and (Kin ?o ?p ?g ?q ?t)
                       (AtPose ?o ?p) (HandEmpty) (AtConf ?q)
                  )
    :effect (and (AtGrasp ?o ?g) (CanMove)
                 (not (AtPose ?o ?p)) (not (HandEmpty)))
  )

  (:action move_holding
    :parameters (?q1 ?q2 ?o ?g ?t)
    :precondition (and (HoldingMotion ?q1 ?t ?q2 ?o ?g)
                       (AtConf ?q1) (AtGrasp ?o ?g) (CanMove)
                  )
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1))) ;(not (CanMove)))
  )

  
  (:action place
    :parameters (?o ?p ?g ?q ?t)
    :precondition (and (Kin ?o ?p ?g ?q ?t)
                       (AtGrasp ?o ?g) (AtConf ?q)
                  )
    :effect (and (AtPose ?o ?p) (HandEmpty) (CanMove)
                 (not (AtGrasp ?o ?g)))
  )

  (:derived (On ?o ?r)
    (exists (?p) (and (Supported ?o ?p ?r)
                      (AtPose ?o ?p)))
  )

  (:derived (Holding ?o)
    (exists (?g) (and (Grasp ?o ?g)
                      (AtGrasp ?o ?g)))
  )
)