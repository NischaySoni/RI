(define (problem hybrid_problem_01)
  (:domain hybrid_robot)
  (:objects
    panda - robot
    obj_a obj_b - object
    loc_a loc_b loc_target loc_away - location
  )

  (:init
    (robot_at panda loc_a)
    (empty panda)

    (obj_at obj_a loc_a)
    (obj_at obj_b loc_b)
    
    ; This is the "lie" that the geometric check will expose!
    (path_clear obj_a)
    (path_clear obj_b) 
  )

  (:goal (and
    (obj_at obj_b loc_target)
  ))
)