; https://planning.wiki/ref/pddl/domain

; GOAL: Stack 2x towers of 3x Blocks, No Motion

(define (domain pick-place-and-stack)
  (:requirements :strips :negative-preconditions)

  ;;;;;;;;;; PREDICATES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:predicates

    ;;; Domains ;;;
    (Graspable ?label); Name of a real object we can grasp
    (Waypoint ?pose) ; Model of any object we can go to in the world, real or not
    

    ;;; Objects ;;;
    (GraspObj ?label ?pose) ; The concept of a named object at a pose
  
    ;;; Object State ;;;
    (Free ?pose) ; This pose is free of objects, therefore we can place something here without collision
    
    ;;; Robot State ;;;
    (HandEmpty)
    (Holding ?label)
    (AtPose ?pose)

  )

  ;;;;;;;;;; ACTIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:action move_free
      :parameters (?poseBgn ?poseEnd)
      :precondition (and 
        ;; Robot State ;;
        (HandEmpty)
        (AtPose ?poseBgn)
      )
      :effect (and 
        ;; Robot State ;;
        (AtPose ?poseEnd)
        (not (AtPose ?poseBgn))
      )
  )
  
  (:action pick
    :parameters (?label ?pose ?prevSupport)
    :precondition (and
      ;; Domain ;;
      (Graspable ?label)
      (Waypoint ?pose)
      ;; Object State ;;
      (GraspObj ?label ?pose)
      ;; Robot State ;;
      (HandEmpty)
    )
    :effect (and
      ;; Robot State ;;
      (Holding ?label)
      (not (HandEmpty))
    )
  )
  
  (:action move_holding
      :parameters (?poseBgn ?poseEnd ?label)
      :precondition (and 
        ;; Robot State ;;
        (Holding ?label)
        (AtPose ?poseBgn)
        ;; Object State ;;
        (Free ?poseEnd)
        (GraspObj ?label ?poseBgn)
      )
      :effect (and 
        ;; Robot State ;;
        (AtPose ?poseEnd)
        (not (AtPose ?poseBgn))
        ;; Object State ;;
        (GraspObj ?label ?poseEnd)
        (not (GraspObj ?label ?poseBgn))
        (Free ?poseBgn)
        (not (Free ?poseEnd))
      )
  )
  
  (:action place
      :parameters (?label ?pose)
      :precondition (and 
                      ;; Domain ;;
                      (Graspable ?label)
                      (Waypoint ?pose)
                      ;; Object State ;;
                      (GraspObj ?label ?pose) 
                      ;; Robot State ;;
                      (Holding ?label)
                      )
      :effect (and 
                ;; Robot State ;;
                (HandEmpty)
                (not (Holding ?label))
              )
  )
  
)