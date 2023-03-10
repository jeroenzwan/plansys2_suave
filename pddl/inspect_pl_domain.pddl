(define (domain inspect-pipeline)

    (:requirements
        :typing
        :durative-actions
        :fluents
        :strips
        :adl
        )

    (:types
        functiondesign
        function
        pipeline
        uuv
        action
    )
    
    (:predicates
        (pipeline_found ?pl - pipeline)
        (pipeline_not_found ?pl - pipeline)
        (pipeline_inspected ?pl - pipeline)
        (pipeline_not_inspected ?pl - pipeline)
        (search_a ?a - action)
        (follow_a ?a - action)
        (recharge_a ?a - action)
        (a_req_f ?a - action ?f1 ?f2 - function)
        (fd_available ?fd - functiondesign)
        (fd_solve_f ?fd - functiondesign ?f - function)
        (charged ?auv - uuv)
        (recharge_required ?auv - uuv)
    )
    (:functions
        (time ?fd - functiondesign)
        ; (battery_usage ?fd - functiondesign)
        ; (battery_level ?auv - uuv)
    )
    
    (:durative-action search_pipeline
        :parameters (?auv - uuv ?f1 ?f2 - function ?fd1 ?fd2 - functiondesign ?pl - pipeline ?a - action)
        :duration ( = ?duration (+ (time ?fd1)(time ?fd2)))
        :condition (and
            (at start (search_a ?a))
            (at start (a_req_f ?a ?f1 ?f2))
            (at start (fd_available ?fd1))
            (at start (fd_available ?fd2))
            (at start (fd_solve_f ?fd1 ?f1))
            (at start (fd_solve_f ?fd2 ?f2))
            (at start (pipeline_not_found ?pl))
            (at start (pipeline_not_inspected ?pl))
            (at start (charged ?auv))
            ; (at start (> (battery_level ?auv) (+
            ;     (+ (battery_usage ?fd1)(battery_usage ?fd2))
            ;     10)
            ; ))
        )
        :effect (and
            (at end (pipeline_found ?pl))
            (at end (not
                (pipeline_not_found ?pl)
            ))
            ; (at end (decrease (battery_level ?auv) (+
            ;     (battery_usage ?fd1)(battery_usage ?fd2))
            ; ))
        )
    )
 
    (:durative-action follow_pipeline
        :parameters (?auv - uuv ?f1 ?f2 - function ?fd1 ?fd2 - functiondesign ?pl - pipeline ?a - action)
        :duration ( = ?duration (+ (time ?fd1)(time ?fd2)))
        :condition (and
            (at start (follow_a ?a))
            (at start (a_req_f ?a ?f1 ?f2))
            (at start (fd_available ?fd1))
            (at start (fd_available ?fd2))
            (at start (fd_solve_f ?fd1 ?f1))
            (at start (fd_solve_f ?fd2 ?f2))
            (at start (pipeline_found ?pl))
            (at start (pipeline_not_inspected ?pl))
            (at start (charged ?auv))
            ; (at start (> (battery_level ?auv) (+
            ;     (+ (battery_usage ?fd1)(battery_usage ?fd2))
            ;     10)
            ; ))
        )
        :effect (and
            (at end (pipeline_inspected ?pl))
            (at end (not
                (pipeline_not_inspected ?pl)
            ))
            ; (at end (decrease (battery_level ?auv) (+
            ;     (battery_usage ?fd1)(battery_usage ?fd2))
            ; ))
        )
    )

    (:durative-action recharge
        :parameters (?auv - uuv ?f1 ?f2 - function ?fd1 ?fd2 - functiondesign ?a - action)
        :duration (= ?duration (+ (time ?fd1)(time ?fd2)))
        :condition (and
            (at start (recharge_a ?a))
            (at start (a_req_f ?a ?f1 ?f2))
            (at start (fd_available ?fd1))
            (at start (fd_available ?fd2))
            (at start (fd_solve_f ?fd1 ?f1))
            (at start (fd_solve_f ?fd2 ?f2))
            (at start (recharge_required ?auv))
            ; (at start (> (battery_level ?auv) (+ (battery_usage
            ; ?fd1)(battery_usage ?fd2))))
        )
        :effect (and
            ; (at end (assign (battery_level ?auv) 100.))
            (at end (charged ?auv))
            (at end (not
                (recharge_required ?auv)
            ))
        )
    )
)

