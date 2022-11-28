# implement the RRT algorithm for motion planner
import gitmodules 
__import__('padm-project-2022f') 
import os
import sys
import argparse
import numpy as np
sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])
from src.world import World
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_pose, Pose, Point, Euler
from src.utils import name_from_type, compute_surface_aabb, sample_reachable_base
import operator

UNIT_POSE2D = (0., 0., 0.)

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn


class Configuration:
    def __init__(self, base_joints, arm_joints, parent=None):
        self.base_joints = base_joints
        self.arm_joints = arm_joints
        self.parent = parent
    
    def distance(self, conf):
        base_diff = tuple(map(operator.sub, self.base_joints, conf.base_joints))
        arm_diff = tuple(map(operator.sub, self.arm_joints, conf.arm_joints))
        return sum([abs(x) for x in base_diff + arm_diff])
    
    def inbounds(self, bounds):
        """ 
        The bounds should be a tuple of 4 elements (base_lower, base_upper, arm_lower, arm_upper)
        Each element is also a tuple
        """
        comparison = [i > j for i, j in zip(self.base_joints, bounds[0])] + [i < j for i, j in zip(self.base_joints, bounds[1])]
                    + [i > j for i, j in zip(self.arm_joints, bounds[2])] + [i < j for i, j in zip(self.arm_joints, bounds[3])]
        for consistent in comparison:
            if not consistent:
                return False
        return True

class MotionPlanner:
    def __init__(self, plan):
        """
        input plan from activity planner
        convert it to world configuration
        """
        self.world = World(use_gui=True)
        self.sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
        self.spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
        self.world._update_initial()
        self.base_sample = get_sample_fn(world.robot, world.base_joints)
        self.arm_sample = get_sample_fn(world.robot, world.arm_joints)
        self.plan = plan
        self.tasks = []
        self.poses = {} # TODO: add poses to this dict
        self.item_on_hand = []
        self.radius = 1
        base_lower_limits, base_upper_limits = get_custom_limits(world.robot, world.base_joints, {}, circular_limits=CIRCULAR_LIMITS)
        arm_lower_limits, arm_upper_limits = get_custom_limits(world.robot, world.arm_joints, {}, circular_limits=CIRCULAR_LIMITS)
        self.bounds = (base_lower_limits, base_upper_limits, arm_lower_limits, arm_upper_limits)
        
    
    def sample(self):
        base_joints = self.base_sample()
        arm_joints = self.arm_sample()
        return Configuration(base_joints, arm_joints)
    
    def nearest(self, leaf, tree):
        near = tree[0]
        dis = leaf.distance(near)
        for node in tree:
            if leaf.distance(node) < dis:
                near = node
                dis = leaf.distance(near)
        return near
    
    def steer(self, parent, leaf, radius, bounds):
        # TODO: convert to feasible code
        base_joints_diff = tuple(map(operator.sub, leaf.base_joints, parent.base_joints))
        arm_joints_diff = tuple(map(operator.sub, leaf.arm_joints, parent.arm_joints))
        base_movement = tuple([j * radius for j in base_joints_diff])
        arm_movement = tuple([j * radius for j in arm_joints_diff])
        conf = Configuration(tuple(map(operator.add, parent.base_joints, base_joints_diff)), 
                             tuple(map(operator.add, parent.arm_joints,arm_joints_diff)), parent=parent)
        if conf.inbounds(bounds):
            return conf
        else:
            leaf.parent = parent
            return leaf
    
    def collision_free(self, parent, des, world, radius):
        return True
    
    def arrive(end_region, conf):
        pass
    
    def rrt(self, bounds, world, start_pose, radius, end_region):
        tree = [Node(start_pose[0], start_pose[1])] # TODO: change it to configuration
        while (True):
            leaf = self.sample()
            parent = self.nearest(leaf, tree)
            des = self.steer(parent, leaf, radius, bounds)
            if self.collision_free(parent, des, world, radius):
                des.parent = tree.index(parent)
                tree.append(des)
                if self.arrive(end_region, des):
                    path = []
                    node = des
                    while (not (node.parent is None)):
                        path.append(node)
                        node = tree[node.parent]
                    path.append(start_pose)
                    path.reverse()
                    self.render(tree, path, world, bounds, radius, start_pose, end_region)
                    return path
    
    def plan(self):
        for act in self.plan:
            if act.name == 'navigate':
                path = self.rrt(self.bounds, self.world, self.poses[act.parameters[0]], self.radius, self.poses[act.parameters[1]])
                raise NotImplementedError
            elif act.name == 'pick':
                path = self.rrt(self.bounds, self.world, self.poses[act.parameters[1]], self.radius, self.poses[act.parameters[0]])
                self.item_on_hand.append(act.parameters[0])
                raise NotImplementedError
            elif act.name == 'place':
                path = self.rrt(self.bounds, self.world, self.poses[act.parameters[0]], self.radius, self.poses[act.parameters[1]])
                self.item_on_hand.remove(act.parameters[0])
                raise NotImplementedError
            elif act.name == 'stow': 
                path = self.rrt(self.bounds, self.world, self.poses[act.parameters[0]], self.radius, self.poses[act.parameters[1]])
                self.item_on_hand.remove(act.parameters[0])
                raise NotImplementedError
            else:
                raise NotImplementedError