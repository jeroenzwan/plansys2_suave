(define (domain inspect-pipeline)

    (:requirements
        :typing
        :durative-actions
        :fluents
        )

    (:types
        functiondesign
        function
        pipeline
        uuv
    )
    
    (:predicates
        (pipeline_found ?pl - pipeline)
        (pipeline_not_found ?pl - pipeline)
        (pipeline_inspected ?pl - pipeline)
        (pipeline_not_inspected ?pl - pipeline)
        (recharge_available ?fd - functiondesign)
        (search_available ?fd - functiondesign)
        (follow_available ?fd - functiondesign)
        (requires_f ?fd - functiondesign ?f_req1 ?f_req2 - function)
        (fd_selected ?fd - functiondesign ?f - function)
        (fd_available ?fd - functiondesign ?f - function)
        (solving ?fd - functiondesign)
        (not_occupied ?auv - uuv)
    )
    (:functions
        (speed ?fd - functiondesign)
        (efficiency ?fd - functiondesign)
        (battery_level ?auv - uuv)
    )
    
    (:durative-action search_pipeline
        :parameters (?auv - uuv ?f_req1 ?f_req2 - function ?fd_av ?fd1 ?fd2 - functiondesign ?pl - pipeline)
        :duration ( = ?duration (+ (speed ?fd1)(speed ?fd2)))
        :condition (and
            (at start (search_available ?fd_av))
            (at start (requires_f ?fd_av ?f_req1 ?f_req2))
            (at start (solving ?fd_av))
            (at start (fd_selected ?fd1 ?f_req1))
            (at start (fd_selected ?fd2 ?f_req2))
            (at start (pipeline_not_found ?pl))
            (at start (pipeline_not_inspected ?pl))
            (at start (> (battery_level ?auv) (+
                (+ (efficiency ?fd1)(efficiency ?fd2))
                10)
            ))
        )
        :effect (and
            (at end (pipeline_found ?pl))
            (at end (not_occupied ?auv))
            (at end (decrease (battery_level ?auv) (+
                (efficiency ?fd1)(efficiency ?fd2))
            ))
            (at end (not
                (pipeline_not_found ?pl)
            ))
            (at end (not
                (fd_selected ?fd1 ?f_req1)
            ))
            (at end (not
                (fd_selected ?fd2 ?f_req2)
            ))
            (at end (not
                (solving ?fd_av)
            ))
        )
    )
 
    (:durative-action follow_pipeline
        :parameters (?auv - uuv ?f_req1 ?f_req2 - function ?fd_av ?fd1 ?fd2 - functiondesign ?pl - pipeline)
        :duration ( = ?duration (+ (speed ?fd1)(speed ?fd2)))
        :condition (and
            (at start (follow_available ?fd_av))
            (at start (requires_f ?fd_av ?f_req1 ?f_req2))
            (at start (solving ?fd_av))
            (at start (fd_selected ?fd1 ?f_req1))
            (at start (fd_selected ?fd2 ?f_req2))
            (at start (pipeline_found ?pl))
            (at start (pipeline_not_inspected ?pl))
            (at start (> (battery_level ?auv) (+
                (+ (efficiency ?fd1)(efficiency ?fd2))
                10)
            ))
        )
        :effect (and
            (at end (pipeline_inspected ?pl))
            (at end (not_occupied ?auv))
            (at end (decrease (battery_level ?auv) (+
                (efficiency ?fd1)(efficiency ?fd2))
            ))
            (at end (not
                (pipeline_not_inspected ?pl)
            ))
            (at end (not
                (fd_selected ?fd1 ?f_req1)
            ))
            (at end (not
                (fd_selected ?fd2 ?f_req2)
            ))
            (at end (not
                (solving ?fd_av)
            ))
        )
    )

    (:durative-action recharge
        :parameters (?auv - uuv ?f_req1 ?f_req2 - function ?fd_av ?fd1 ?fd2 - functiondesign)
        :duration (= ?duration 40)
        :condition (and
            (at start (recharge_available ?fd_av))
            (at start (requires_f ?fd_av ?f_req1 ?f_req2))
            (at start (solving ?fd_av))
            (at start (fd_selected ?fd1 ?f_req1))
            (at start (fd_selected ?fd2 ?f_req2))
            (at start (> (battery_level ?auv) (+ (efficiency ?fd1)(efficiency ?fd2))))
        )
        :effect (and
            (at end (not_occupied ?auv))
            (at end (assign (battery_level ?auv) 100.))
            (at end (not
                (fd_selected ?fd1 ?f_req1)
            ))
            (at end (not
                (fd_selected ?fd2 ?f_req2)
            ))
            (at end (not
                (solving ?fd_av)
            ))
        )
    )
   
    (:durative-action reconfigure
        :parameters (?f - function ?fd_old ?fd_new - functiondesign)
        :duration (= ?duration 2)
        :condition (and
            (at start (fd_available ?fd_new ?f))
            (at start (fd_selected ?fd_old ?f))
        )
        :effect (and
            (at end (fd_selected ?fd_new ?f))
            (at end (not
                (fd_selected ?fd_old ?f)
            ))
        )
    )

    (:durative-action set_configuration
        :parameters (?auv - uuv ?f1 ?f2 - function ?fd_rand ?fd_act ?fd1 ?fd2 - functiondesign)
        :duration (= ?duration 2)
        :condition (and
            (at start (fd_available ?fd1 ?f1))
            (at start (fd_available ?fd2 ?f2))
            (at start (requires_f ?fd_act ?f1 ?f2))
            (at start (not_occupied ?auv))
        )
        :effect (and
            (at end (fd_selected ?fd1 ?f1))
            (at end (fd_selected ?fd2 ?f2))
            (at end (solving ?fd_act))
            (at end (not
                (not_occupied ?auv)
            ))
        )
    )
)

