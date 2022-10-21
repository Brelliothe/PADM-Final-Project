;Header and description

(define (domain kitchen)

;remove requirements that are not needed
(:requirements :strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)

(:types object furniture ; things are divide into 2 classes, object is able to be pick and move, furniture is fixed
)

; un-comment following line if constants are needed
;(:constants )

(:predicates (near ?r)  ; whether the arm is close to an object or furniture
             (grasp ?r - object) ; whether the arm grasp the object
             (on ?obj - object ?furn - furniture) ; whether the object is on the furniture
             (inside ?obj - object ?furn - furniture) 
)


; (:functions ;todo: define numeric functions here
; )

(:action navigate
    :parameters (?from ?to)
    :precondition (and (near ?from))
    :effect (and (near ?to) (not(near ?from)))
)

(:action pick
    :parameters (?obj - object)
    :precondition (and (near ?obj))
    :effect (and (grasp ?obj))
)

(:action remove
    :parameters (?obj - object ?fur - furniture)
    :precondition (and (near ?obj) (on ?obj ?fur))
    :effect (and (grasp ?obj) (not (on ?obj ?fur)))
)


(:action place
    :parameters (?obj - object ?fur - furniture)
    :precondition (and (grasp ?obj) (near ?fur))
    :effect (and (not (grasp ?obj)) (on ?obj ?fur))
)

(:action stow
    :parameters (?obj - object ?fur - furniture)
    :precondition (and (grasp ?obj) (near ?fur))
    :effect (and (not (grasp ?obj)) (inside ?obj ?fur))
)


)