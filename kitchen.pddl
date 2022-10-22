;Header and description

(define (domain kitchen)

;remove requirements that are not needed
(:requirements :strips :typing :negative-preconditions)

(:types 
    objective      ; can be moved
    furniture   ; fixed
)

; un-comment following line if constants are needed
;(:constants )

(:predicates 
    (near ?r)  ; whether the arm is close to an objective or furniture
    (grasp ?r - objective) ; whether the arm grasp the objective
    (on ?obj - objective ?furn - furniture) ; whether the objective is on the furniture
    (inside ?obj - objective ?furn - furniture) 
)

; move between furnitures
(:action navigate
    :parameters (?from - furniture ?to - furniture)
    :precondition (and (near ?from))
    :effect (and (near ?to) (not(near ?from)))
)

(:action pick
    :parameters (?obj - objective ?fur - furniture)
    :precondition (and (near ?fur) (on ?obj ?fur))
    :effect (and (grasp ?obj) (not (on ?obj ?fur)))
)

(:action place
    :parameters (?obj - objective ?fur - furniture)
    :precondition (and (grasp ?obj) (near ?fur))
    :effect (and (not (grasp ?obj)) (on ?obj ?fur))
)

(:action stow
    :parameters (?obj - objective ?fur - furniture)
    :precondition (and (grasp ?obj) (near ?fur))
    :effect (and (not (grasp ?obj)) (inside ?obj ?fur))
)


)