# PADM-Final-Project

## section 1

In this section, we make 2 assumptions: 
* the arm is not able to move furniture
* every object's relative location to furniture is known

Based on the assumptions, we design the domain file *kitchen.pddl* and problem file *task.pddl*. The high level idea of designing is use furniture as location basis. 

We define predicate `near(?r - furniture)` for arm to check current location and action `navigate(?from - furniture ?to - furniture)` to move between locations. We also define `pick(?obj - objective ?fur - furniture)` to represent picking an objective from a furniture and `grasp(?r - objective)` for arm to check what is in hand. For given tasks, we specifically design `on(?obj - objective ?fur - furniture)` and `inside(?obj - objective ?fur - furniture)` to check the relationship between objects and furnitures, and `place(?obj - objective ?fur - furniture)` and `stow(?obj - objective ?fur - furniture)` to modify the two predicates.

After define the domain file, the problem is easy to represent. Based on task description as well as assumption, we design that the arm is initially in the center and both the sugar box and the spam box are on burner. We ask them to be on countertop and drawer finally.

To find feasible plan, we write a BFS planner search in configuration space as learned in class. The main challenge comes from the unfamiliar pddl language and parser api.  
