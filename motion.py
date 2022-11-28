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
from src.utils import name_from_type, compute_surface_aabb

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
    def __init__(self, base_joints, arm_joints):
        self.base_joints = base_joints
        self.arm_joints = arm_joints


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
    
    def sample(self):
        return Configuration(self.base_sample(), self.arm_sample())
    
    def nearest(self, leaf, tree):
        pass
    
    def steer(self, parent, leaf, radius, bounds):
        pass
    
    def collision_free(self, parent, des, world, radius):
        return True
    
    def arrive(end_region, pose):
        pass
    
    def render(self, tree, path, world, bounds, radius, start_pose, end_region):
        pass
    
    def rrt(self, bounds, world, start_pose, radius, end_region):
        tree = [Node(start_pose[0], start_pose[1])]
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
                