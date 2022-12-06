# PADM-Final-Project

## section 1

In this section, we make 2 assumptions: 
* the arm is not able to move furniture
* every object's relative location to furniture is known

Based on the assumptions, we design the domain file ***kitchen.pddl*** and problem file ***task.pddl***. The high level idea of designing is use furniture as location basis. 

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

In this section, we write a motion planner in ***motion.py*** and an engine in ***engine.py***. 

In ***engine.py***, it will first call the activity planner written in section 1 in ***activity.py*** to produce the plan. Then it will call the motion planner written in ***motion.py*** to generate the motion and do the simulation. The convertion from plan to commands for motion planner is also done in ***motion.py***.

In ***motion.py***, we implemented a RRT based motion planner. To make the problem easier, we made several assumptions: the items (including the sugar box, the spam box, and the drawer handle) do not have collision shape; the items will attach to the end-effector when it is within a radius; the drawer will open after attached to the end-effector and move together as the end-effector moves back. 

There are 2 main key functions in ***motion.py***. The first one is the `motion_plan(self)` in the `MotionPlanner` class. This function defines the pipeline of the motion planning and convert the input from section 1. It deal with each action of the plan case by case. Within a certain case, it first get the start configuration and define the end_region. Then it feed the input into the `rrt` function to get the motion plan. Finally it will execute it to do the simulation. Another important function is `rrt(self, bounds, world, start_conf, radius, end_region)`. This is the main function solving the motion planning problem for given start position and end position. It written as the standard rrt framework, with problem specific sub-functions including `sample()`, `nearest(leaf, tree)`, `steer(parent, leaf, radius, bound)`, `collision_free(parent, des, world, radius)` and `arrive(end_region, des)`. 
* `sample()` is a wrapper of the sample function in ***minimal_example.py***, which samples the configuration for the arm. 
* `nearest(leaf, tree)` find the closest configuration in the tree to be the parent. 
* `steer(parent, leaf, radius, bound)` limit the distance between the parent and child configuration by computing the intermedium of the child and the parent. 
* `collision_free(parent, des, world, radius)` checks the collision. It go through all the bodies in the environment except the spam box and the sugar box, to check collision with the arm's link.
* `arrive(end_region, des)` checks whether current pose is close enough to the target pose. We measure the distance by the L_1 norm.

To integrate the activity plan to motion plan, we do it separately. We have 4 actions in total, so in `motion_plan(self)` we use if - elif - else logic to deal with them. Our actions have 2 components: **action name** and **action parameters**.
* `navigate` is the action that robot move from one place to the other. For this action, because the arm does not move and there is usually no collision, we do it in the deterministic way: get current pose of robot base and the target pose, generate a safe path to it and execute it. 
* `pick` is the action that robot move to grasp the item (spam box or sugar box). We first obtain the item's pose and generate a small box as end_region (this is done by defining a class `Region`). Then we use `rrt` to get the path and execute it. Finally we attach the item to the grasp by adding it to an array `self.item_on_hand`, which means it will later update the pose together with the arm until it is removed. 
* `place` is the inverse action of `pick`, so we just do it in the same way with inversed start and end_region.
* `stow` is the most complex action. Because in section 1 we do not define the drawer object and open/close action for it separately, we need to do all of them in this part. We divide the stow into a set of previous 3 actions. We first `place` the item back to the top of the drawer, then `navigate` to the handle of the drawer. Next, we pull the drawer out by moving a straight line along x-axis. Later, we `pick` the item again and `place` it into the drawer. Finally, we `navigate` to the handle and push it back by moving the reversed straight line along x-axis. 

The video is uploaded in the repo called ***simulation.mp4***. Because by our assumption, the collision shapes of items are removed, you can see that some times the item will be half-inside the table. This could be solved if we add it back. But we cannot deal with the collision between end-effector and the item when we need to pick them up can carry them. So in the final version we choose to keep the assumption.

## section 3

In section 3, we add 2 functions `generate_trajectory(self)` and `run_trajectory(self, plan)` in ***motion.py***. We implement the optimization encoding in ***optimizer.py*** by pyDrake and use ***compare.py*** for visualization.

* `generate_trajectory(self)` defines the start pose and end pose and uses `rrt` function implemented in section 2 to solve out a feasible trajectory. It will save the trajectory into file ***trajectory_rrt.npy***.
* `run_trajectory(self, plan)` will execute a given squence of configuration, which is used for visualization.
* `optimizer.py` read the trajectory generated by planner, and use it as input to generate the better trajectory.
* `compare.py` read the saved trajectory and call the `run_trajectory` for visualization.

The problem is relatively simple. We ask the robot to move its arm from initial pose to another pose along x-axis 0.6 distance away. Initially, the arm is at the negative direction along x-axis to the base, so this task will require the robot to move its arm to turn around. You can watch the video to get a more clear sense of the task. We choose this task because it is non-trivial. The best trajectory will be an arc which is hard for rrt to follow. 

Formally speaking, our optimization problem is $$
\begin{align*}
&min\sum_{i=1}^{N-1}\|x_i-x_{i-1}\|_2^2 \\
s.t.\ \ \ \ \ &x_0=f_0\\
& x_{N-1}=f_{N-1}\\
& (x_i-x_{i-1})_j\in[-\epsilon,\epsilon],\ i=1,2,\cdots,N-1, \ j=0,1,\cdots,6
\end{align*}$$
Here each $x$ is the joints configuration for the arm which is a 7-d vector. In implementation, we choose $\epsilon$ to be $0.3$.

This optimization method works really well. In the video, it is easy to see that the rrt algorithm provides a trajectory of polygonal line while the optimization method generate a much more smooth and efficient trajectory of arc. 

The sample-based trajectory is saved in ***trajectory_rrt.npy*** and is visualized in ***rrt.mp4***. The optimized trajectory is saved in ***trajectory_opt.npy*** and is visualized in ***opt.mp4***.
