# PADM-Final-Project

## section 1

In this section, we make 2 assumptions: 
* the arm is not able to move furniture
* every object's relative location to furniture is known

Based on the assumptions, we design the domain file *kitchen.pddl* and problem file *task.pddl*. The high level idea of designing is use furniture as location basis. 

We define predicates:
* `near(?r - furniture)` check current location for arm 
* `grasp(?r - objective)` for arm to check what is in hand
* `on(?obj - objective ?fur - furniture)` check item's relative location 
* `inside(?obj - objective ?fur - furniture)`check item's relative location
We define actions 
* `navigate(?from - furniture ?to - furniture)` move between locations 
* `pick(?obj - objective ?fur - furniture ?h - hand)` represent picking an objective from a furniture, here the hand class is defined to aviod picking 2 items in hand simultaneously.  
* `place(?obj - objective ?fur - furniture ?h - hand)` modify `on` predicate
* `stow(?obj - objective ?fur - furniture ?h - hand)` modify `inside` predicate

After define the domain file, the problem is easy to represent. Based on task description as well as assumption, we design that the arm is initially in the center and both the sugar box and the spam box are on burner. We ask them to be on countertop and drawer finally.

To find feasible plan, we write a BFS planner search in configuration space as learned in class. The main challenge comes from the unfamiliar pddl language and parser api.  

## section 2

In this section, we write a motion planner in *motion.py* and an engine in *engine.py*. 

In *engine.py*, it will first call the activity planner written in section 1 in *activity.py* to produce the plan. Then it will call the motion planner written in *motion.py* to generate the motion and do the simulation. The convertion from plan to commands for motion planner is also done in *motion.py*.

In *motion.py*, we implemented a RRT based motion planner. To make the problem easier, we made several assumptions: the items (including the sugar box, the spam box, and the drawer handle) do not have collision shape; the items will attach to the end-effector when it is within a radius; the drawer will open after attached to the end-effector and move together as the end-effector moves back. 

There are 2 main key functions in *motion.py*. The first one is the `motion_plan(self)` in the `MotionPlanner` class. This function defines the pipeline of the motion planning and convert the input from section 1. It deal with each action of the plan case by case. Within a certain case, it first get the start configuration and define the end_region. Then it feed the input into the `rrt` function to get the motion plan. Finally it will execute it to do the simulation. Another important function is `rrt(self, bounds, world, start_conf, radius, end_region)`. This is the main function solving the motion planning problem for given start position and end position. It written as the standard rrt framework, with problem specific sub-functions including `sample()`, `nearest(leaf, tree)`, `steer(parent, leaf, radius, bound)`, `collision_free(parent, des, world, radius)` and `arrive(end_region, des)`. 
* `sample()` is a wrapper of the sample function in *minimal_example.py*, which samples the configuration for the arm. 
* `nearest(leaf, tree)` find the closest configuration in the tree to be the parent. 
* `steer(parent, leaf, radius, bound)` limit the distance between the parent and child configuration by computing the intermedium of the child and the parent. 
* `collision_free(parent, des, world, radius)` checks the collision. It go through all the bodies in the environment except the spam box and the sugar box, to check collision with the arm's link.
* `arrive(end_region, des)` checks whether current pose is close enough to the target pose. We measure the distance by the L_1 norm.

To integrate the activity plan to motion plan, we do it separately. We have 4 actions in total, so in `motion_plan(self)` we use if - elif - else logic to deal with them. Our actions have 2 components: **action name** and **action parameters**.
* `navigate` is the action that robot move from one place to the other. For this action, because the arm does not move and there is usually no collision, we do it in the deterministic way: get current pose of robot base and the target pose, generate a safe path to it and execute it. 
* `pick` is the action that robot move to grasp the item (spam box or sugar box). We first obtain the item's pose and generate a small box as end_region (this is done by defining a class `Region`). Then we use `rrt` to get the path and execute it. Finally we attach the item to the grasp by adding it to an array `self.item_on_hand`, which means it will later update the pose together with the arm until it is removed. 
* `place` is the inverse action of `pick`, so we just do it in the same way with inversed start and end_region.
* `stow` is the most complex action. Because in section 1 we do not define the drawer object and open/close action for it separately, we need to do all of them in this part. We divide the stow into a set of previous 3 actions. We first `place` the item back to the top of the drawer, then `navigate` to the handle of the drawer. Next, we pull the drawer out by moving a straight line along x-axis. Later, we `pick` the item again and `place` it into the drawer. Finally, we `navigate` to the handle and push it back by moving the reversed straight line along x-axis. 

The video is uploaded in the repo called *simulation.mp4*.