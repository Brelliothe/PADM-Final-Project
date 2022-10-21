(define (problem move) (:domain kitchen)
(:objects sugar_box spam_box - object 
          burner countertop drawer_red - furniture
)

(:init
    (on sugar_box burner)
    ;todo: put the initial state's facts and numeric values here
)

(:goal (and
    (not (on sugar_box burner))
    (on sugar_box countertop)
    (inside spam_box drawer_red)
    ;todo: put the goal condition here
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
