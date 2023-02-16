(define (problem inspect-pipeline)
    (:domain inspect-pipeline)
    (:requirements :strips :typing :fluents)

    (:objects   bluerov - uuv
                pl1 - pipeline
                fd1 fd2 fd3 - functiondesign
                p1 - path
    )
    (:init
        (deployed bluerov)
        (localized bluerov)
        (fd_selected fd2)
        (pipeline_found pl1)
        (pipeline_not_inspected pl1)
        (available_fd fd2)
        (available_fd fd3)
        (= (speed fd2) 5)
        (= (speed fd3) 20)
        (= (accuracy fd2) 20)
        (= (accuracy fd3) 5)
    )
    
    (:goal (and
        (pipeline_inspected pl1))
    )

)


