(define (problem move) (:domain kitchen)
(:objects 
    sugar spam - objective
    burner countertop drawer center - furniture
)

(:init
    (on sugar burner)
    (on spam burner)
    (near center)
)

(:goal (and
    (not (on sugar burner))
    (on sugar countertop)
    (inside spam drawer)
))
)
