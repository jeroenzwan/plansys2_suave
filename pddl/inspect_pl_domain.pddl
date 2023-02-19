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
    )
    
    (:predicates
        (pipeline_found ?pl - pipeline)
        (pipeline_not_found ?pl - pipeline)
        (pipeline_inspected ?pl - pipeline)
        (pipeline_not_inspected ?pl - pipeline)
;        (recharge_available ?fd - functiondesign)
;        (search_available ?fd - functiondesign)
;        (follow_available ?fd - functiondesign)
        (search_requires_f ?fd - functiondesign ?f_req1 ?f_req2 - function)
        (follow_requires_f ?fd - functiondesign ?f_req1 ?f_req2 - function)
        (recharge_requires_f ?fd - functiondesign ?f_req1 ?f_req2 - function)
        (fd_available ?fd - functiondesign ?f - function)
        (recharged ?auv - uuv)
    )
    (:functions
        (speed ?fd - functiondesign)
        (battery_usage ?fd - functiondesign)
        (battery_level ?auv - uuv)
    )
    
    (:durative-action search_pipeline
        :parameters (?f_req1 ?f_req2 - function ?fd_av ?fd1 ?fd2 - functiondesign ?pl - pipeline)
        :duration ( = ?duration (+ (speed ?fd1)(speed ?fd2)))
        :condition (and
;            (at start (search_available ?fd_av))
            (at start (search_requires_f ?fd_av ?f_req1 ?f_req2))
            (at start (fd_available ?fd1 ?f_req1))
            (at start (fd_available ?fd2 ?f_req2))
            (at start (pipeline_not_found ?pl))
            (at start (pipeline_not_inspected ?pl))
;            (at start (> (battery_level ?auv) (+
;                (+ (battery_usage ?fd1)(battery_usage ?fd2))
;                10)
;            ))
        )
        :effect (and
            (at end (pipeline_found ?pl))
            (at end (not
                (pipeline_not_found ?pl)
            ))
;            (at end (decrease (battery_level ?auv) (+
;                (battery_usage ?fd1)(battery_usage ?fd2))
;            ))
        )
    )
 
    (:durative-action follow_pipeline
        :parameters (?f_req1 ?f_req2 - function ?fd_av ?fd1 ?fd2 - functiondesign ?pl - pipeline)
        :duration ( = ?duration (+ (speed ?fd1)(speed ?fd2)))
        :condition (and
;            (at start (follow_available ?fd_av))
            (at start (follow_requires_f ?fd_av ?f_req1 ?f_req2))
            (at start (fd_available ?fd1 ?f_req1))
            (at start (fd_available ?fd2 ?f_req2))
            (at start (pipeline_found ?pl))
            (at start (pipeline_not_inspected ?pl))
;            (at start (> (battery_level ?auv) (+
;                (+ (battery_usage ?fd1)(battery_usage ?fd2))
;                10)
;            ))
        )
        :effect (and
            (at end (pipeline_inspected ?pl))
            (at end (not
                (pipeline_not_inspected ?pl)
            ))
;            (at end (decrease (battery_level ?auv) (+
;                (battery_usage ?fd1)(battery_usage ?fd2))
;            ))
        )
    )

    (:durative-action recharge
        :parameters (?auv - uuv ?f_req1 ?f_req2 - function ?fd_av ?fd1 ?fd2 - functiondesign)
        :duration (= ?duration 40)
        :condition (and
;            (at start (recharge_available ?fd_av))
            (at start (recharge_requires_f ?fd_av ?f_req1 ?f_req2))
            (at start (fd_available ?fd1 ?f_req1))
            (at start (fd_available ?fd2 ?f_req2))
;            (at start (> (battery_level ?auv) (+ (battery_usage
;            ?fd1)(battery_usage ?fd2))))
        )
        :effect (and
;            (at end (assign (battery_level ?auv) 100.))
            (at end (recharged ?auv))
        )
    )
)

