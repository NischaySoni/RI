(define (domain hybrid_robot)
  (:requirements :strips :typing)
  (:types
    robot location object - item
  )

  (:predicates
    (robot_at ?r - robot ?l - location)
    (obj_at ?o - object ?l - location)
    (holding ?r - robot ?o - object)
    (empty ?r - robot)
    (path_clear ?o - object)
    (blocker ?o1 - object ?o2 - object)
    (obj_at_away ?o - object)
  )

  (:action pick
    :parameters (?r - robot ?o - object ?l - location)
    :precondition (and
      (robot_at ?r ?l)
      (obj_at ?o ?l)
      (empty ?r)
      (path_clear ?o) ; <-- This is the precondition we will check
    )
    :effect (and
      (not (obj_at ?o ?l))
      (not (empty ?r))
      (holding ?r ?o)
    )
  )

  (:action place
    :parameters (?r - robot ?o - object ?l - location)
    :precondition (and
      (robot_at ?r ?l)
      (holding ?r ?o)
    )
    :effect (and
      (not (holding ?r ?o))
      (obj_at ?o ?l)
      (empty ?r)
    )
  )
  
  (:action move_blocker
    :parameters (?r - robot ?o_blocker - object ?o_target - object ?l_start - location ?l_away - location)
    :precondition (and
        (robot_at ?r ?l_start)
        (obj_at ?o_blocker ?l_start)
        (empty ?r)
        (blocker ?o_blocker ?o_target) ; <-- Can only be called if this is true
    )
    :effect (and
        (not (obj_at ?o_blocker ?l_start))
        (obj_at ?o_blocker ?l_away)
        (obj_at_away ?o_blocker)
        (path_clear ?o_target) ; <-- The path is now clear!
        (not (blocker ?o_blocker ?o_target))
    )
  )
)