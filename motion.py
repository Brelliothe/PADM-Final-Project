# implement the RRT algorithm for motion planner
import gitmodules 
__import__('padm-project-2022f') 
import os
import sys
import argparse
import numpy as np

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, get_all_links, inverse_kinematics_helper
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_positions
from pybullet_tools.utils import single_collision, pairwise_collision, remove_body, get_bodies, get_body_name, get_body_info, has_body, body_from_name
from pybullet_tools.utils import get_joint, get_joint_position, set_joint_position, wait_for_duration

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly
import operator

UNIT_POSE2D = (0., 0., 0.)

def tuple_mean(a, b):
    result = [b[0]-0.1, (a[1] + b[1]) / 2, b[2]]
    return tuple(result)

def interpolate(start_joints, end_joints, distance):
    joints = []
    for x in np.linspace(start_joints[0], end_joints[0], int(abs(end_joints[0] - start_joints[0]) / distance)):
        joints.append([x, start_joints[1], start_joints[2]])
    if start_joints[1] < end_joints[1]:
        for theta in np.linspace(start_joints[2], np.pi / 2, 90):
            joints.append([end_joints[0], start_joints[1], theta])
        for y in np.linspace(start_joints[1], end_joints[1], int(abs(end_joints[1] - start_joints[1]) / distance)):
            joints.append([end_joints[0], y, np.pi / 2])
        for theta in np.linspace(np.pi / 2, end_joints[2], 90):
            joints.append([end_joints[0], end_joints[1], theta])
    elif start_joints[1] > end_joints[1]:
        for theta in np.linspace(start_joints[2], np.pi * 3 / 2, 90):
            joints.append([end_joints[0], start_joints[1], theta])
        for y in np.linspace(start_joints[1], end_joints[1], int(abs(end_joints[1] - start_joints[1]) / distance)):
            joints.append([end_joints[0], y, np.pi * 3 / 2])
        for theta in np.linspace(np.pi * 3 / 2, end_joints[2], 90):
            joints.append([end_joints[0], end_joints[1], theta])
    return joints
    

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    # print('add body: ', name)
    world.add_body(name, color=np.ones(4))
    pose = pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name, pose

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    # print(body, entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

def pose2d_on_surface_virtual(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    # set_pose(body, pose)
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
        return sum([abs(x) for x in arm_diff])
    
    def inbounds(self, bounds):
        """ 
        The bounds should be a tuple of 4 elements (base_lower, base_upper, arm_lower, arm_upper)
        Each element is also a tuple
        """
        comparison = [i > j for i, j in zip(self.base_joints, bounds[0])] + [i < j for i, j in zip(self.base_joints, bounds[1])]\
                    + [i > j for i, j in zip(self.arm_joints, bounds[2])] + [i < j for i, j in zip(self.arm_joints, bounds[3])]
        for consistent in comparison:
            if not consistent:
                return False
        return True
    
class Region:
    def __init__(self, pose):
        # self.pose = (pose, (0, 0, 0, 1)) 
        self.radius = 0.1
        self.point = pose
        # self.conf = conf
    
    def contain(self, point):
        for i in range(3):
            if point[i] < self.point[i] - self.radius or point[i] > self.point[i] + self.radius:
                return False
        return True

class MotionPlanner:
    def __init__(self, plan):
        """
        input plan from activity planner
        convert it to world configuration
        """
        self.world = World(use_gui=True)
        self.sugar_box, sugar_box_pose = add_sugar_box(self.world, idx=0, counter=1, pose2d=(0.05, 0.65, np.pi / 4))
        self.spam_box, spam_box_pose = add_spam_box(self.world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
        self.world._update_initial()
        # remove_body(self.world.gripper)
        self.base_sample = get_sample_fn(self.world.robot, self.world.base_joints)
        self.arm_sample = get_sample_fn(self.world.robot, self.world.arm_joints)
        self.plan = plan
        self.tasks = []
        self.tool_link = link_from_name(self.world.robot, 'panda_hand')
        self.all_link = get_all_links(self.world.robot)
        # print('tool link: {}'.format(self.tool_link))
        # print('all link: {}'.format(self.all_link))
        self.base_joints = get_joint_positions(self.world.robot, self.world.base_joints)
        self.arm_joints = get_joint_positions(self.world.robot, self.world.arm_joints)
        self.ik_joints = get_ik_joints(self.world.robot, PANDA_INFO, self.tool_link)
        self.initial_orientaion = self.base_joints[-1]
        self.burner_surface = compute_surface_aabb(self.world, 'front_right_stove')   # lower-left, upper right
        # print(self.burner_surface)
        self.drawer_surface = compute_surface_aabb(self.world, 'indigo_tmp')
        self.countertop_surface = compute_surface_aabb(self.world, 'hitman_tmp')
        self.drawer_front_surface = compute_surface_aabb(self.world, 'indigo_drawer_top')
        # print(self.drawer_surface, self.drawer_front_surface)
        self.poses = {'center': get_link_pose(self.world.robot, self.tool_link), 'sugar': sugar_box_pose, 'spam': spam_box_pose, 
                      'burner': (tuple_mean(self.burner_surface[0], self.burner_surface[1]), (0, 0, 0, 1)), 
                      'drawer': (tuple_mean(self.drawer_surface[0], self.drawer_surface[1]), (0, 0, 0, 1)), 
                      'countertop': (tuple_mean(self.countertop_surface[0], self.countertop_surface[1]), (0, 0, 0, 1))}
        self.item_on_hand = []
        self.radius = 1
        base_lower_limits, base_upper_limits = get_custom_limits(self.world.robot, self.world.base_joints, {}, circular_limits=CIRCULAR_LIMITS)
        arm_lower_limits, arm_upper_limits = get_custom_limits(self.world.robot, self.world.arm_joints, {}, circular_limits=CIRCULAR_LIMITS)
        self.bounds = (base_lower_limits, base_upper_limits, arm_lower_limits, arm_upper_limits)
        self.no_collision_body = [self.world.body_from_name[key] for key in self.world.body_from_name.keys()]
    
    def sample(self):
        base_joints = get_joint_positions(self.world.robot, self.world.base_joints)
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
        # TODO: add the correct version
        origin_conf = get_joint_positions(self.world.robot, self.world.arm_joints)
        set_joint_positions(self.world.robot, self.ik_joints, des.arm_joints)
        result = True
        for body in get_bodies():
            if body not in self.no_collision_body and pairwise_collision(self.tool_link, body):
                result = False
        set_joint_positions(self.world.robot, self.ik_joints, origin_conf)
        return result
    
    def arrive(self, end_region, conf):
        # pose = end_region.pose
        # radius = end_region.radius
        # end_conf = inverse_kinematics_helper(self.world.robot, self.all_link, pose)
        # end_conf = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, pose, max_time=0.05), None)
        # end_conf = end_region.conf
        # arm_conf = conf.arm_joints
        origin_conf = get_joint_positions(self.world.robot, self.world.arm_joints)
        set_joint_positions(self.world.robot, self.ik_joints, conf.arm_joints)
        pose = get_link_pose(self.world.robot, self.tool_link)[0]
        result = end_region.contain(pose)
        set_joint_positions(self.world.robot, self.ik_joints, origin_conf)
        return result
        # conf_diff = tuple(map(operator.sub, end_conf, arm_conf))
        # return sum([abs(x) for x in conf_diff]) < 0.01
    
    def rrt(self, bounds, world, start_conf, radius, end_region):
        tree = [start_conf] 
        while (True):
            leaf = self.sample()
            parent = self.nearest(leaf, tree)
            des = self.steer(parent, leaf, radius, bounds)
            if self.collision_free(parent, des, world, radius):
                tree.append(des)
                if self.arrive(end_region, des):
                    path = []
                    node = des
                    while (not (node.parent is None)):
                        path.append(node)
                        node = node.parent
                    path.append(start_conf)
                    path.reverse()
                    # self.render(tree, path, world, bounds, radius, start_pose, end_region)
                    return path
            # break
    
    def motion_plan(self):
        plan = []
        tool_link = link_from_name(self.world.robot, 'panda_hand')
        ik_joints = get_ik_joints(self.world.robot, PANDA_INFO, tool_link)
        for act in self.plan:
            print(act)
            if act.name == 'navigate':
                end_pose = self.poses[act.parameters[1]][0]
                goal_joints = [end_pose[0] + 0.65, end_pose[1], self.initial_orientaion] # 0.5 threshold to keep the distance
                joints = interpolate(get_joint_positions(self.world.robot, self.world.base_joints), goal_joints, 0.01)
                for joint in joints:
                    wait_for_duration(0.01)
                    set_joint_positions(self.world.robot, self.world.base_joints, joint)
                    pose = get_link_pose(self.world.robot, self.tool_link)
                    for item in self.item_on_hand:
                        body = 4 if item == 'sugar' else 5
                        set_pose(body, pose)
                        self.poses[item] = pose
                plan += joints
                # set_joint_positions(self.world.robot, self.world.base_joints, plan[-1])
                # end_region = Region(self.poses[act.parameters[1]])
                # path = self.rrt(self.bounds, self.world, Configuration(self.base_joints, self.arm_joints), self.radius, end_region)
                # for conf in path:
                #     set_joint_positions(self.world.robot, self.world.base_joints, conf.base_joints)
                #     set_joint_positions(self.world.robot, ik_joints, conf.arm_joints)
            elif act.name == 'pick':
                end_pose = self.poses[act.parameters[0]][0]
                current_joint = get_joint_positions(self.world.robot, self.world.base_joints)
                goal_joints = [current_joint[0], end_pose[1], self.initial_orientaion] # 0.5 threshold to keep the distance
                joints = interpolate(get_joint_positions(self.world.robot, self.world.base_joints), goal_joints, 0.01)
                for joint in joints:
                    wait_for_duration(0.01)
                    set_joint_positions(self.world.robot, self.world.base_joints, joint)
                # end_pose = (self.poses[act.parameters[0]][0], get_link_pose(self.world.robot, self.tool_link)[1])
                # print(end_pose)
                # print(get_link_pose(self.world.robot, self.tool_link))
                # end_conf = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, end_pose, max_time=5), None)
                # end_conf = inverse_kinematics_helper(self.world.robot, self.tool_link, end_pose)
                # print(end_conf)
                end_region = Region(end_pose)
                start_joints = Configuration(get_joint_positions(self.world.robot, self.world.base_joints), get_joint_positions(self.world.robot, self.world.arm_joints))
                path = self.rrt(self.bounds, self.world, start_joints, self.radius, end_region)
                plan += path
                self.item_on_hand.append(act.parameters[0])
                for conf in path:
                    wait_for_duration(1 / len(path))
                    set_joint_positions(self.world.robot, self.ik_joints, conf.arm_joints)
                pose = get_link_pose(self.world.robot, self.tool_link)
                for item in self.item_on_hand:
                    body = 4 if item == 'sugar' else 5
                    set_pose(body, pose)
                    self.poses[item] = pose
                # wait_for_user()
            elif act.name == 'place':
                pose2d = self.poses[act.parameters[1]][0]
                for item in self.item_on_hand:
                    if item == 'sugar':
                        name = name_from_type('sugar_box', 0)
                        end_pose = pose2d_on_surface_virtual(self.world, name, COUNTERS[1], pose2d=(pose2d[0], pose2d[1], np.pi / 4))
                    else:
                        name = name_from_type('potted_meat_can', 1)
                        end_pose = pose2d_on_surface_virtual(self.world, name, COUNTERS[0], pose2d=(pose2d[0], pose2d[1], np.pi / 4))
                end_region = Region(end_pose[0])
                start_joints = Configuration(get_joint_positions(self.world.robot, self.world.base_joints), get_joint_positions(self.world.robot, self.world.arm_joints))
                path = self.rrt(self.bounds, self.world, start_joints, self.radius, end_region)
                for conf in path:
                    wait_for_duration(1 / len(path))
                    set_joint_positions(self.world.robot, self.ik_joints, conf.arm_joints)
                    pose = get_link_pose(self.world.robot, self.tool_link)
                    for item in self.item_on_hand:
                        body = 4 if item == 'sugar' else 5
                        set_pose(body, pose)
                        self.poses[item] = pose
                for item in self.item_on_hand:
                    self.poses[item] = end_pose
                    body = 4 if item == 'sugar' else 5
                    set_pose(body, end_pose)
                self.item_on_hand.remove(act.parameters[0])
                # wait_for_user()
            elif act.name == 'stow': 
                # place back
                pose2d = self.poses[act.parameters[1]][0]
                for item in self.item_on_hand:
                    if item == 'sugar':
                        name = name_from_type('sugar_box', 0)
                        end_pose = pose2d_on_surface_virtual(self.world, name, COUNTERS[1], pose2d=(pose2d[0], pose2d[1], np.pi / 4))
                    else:
                        name = name_from_type('potted_meat_can', 1)
                        end_pose = pose2d_on_surface_virtual(self.world, name, COUNTERS[0], pose2d=(0.2, 1.1, np.pi / 4))
                end_region = Region(end_pose[0])
                start_joints = Configuration(get_joint_positions(self.world.robot, self.world.base_joints), get_joint_positions(self.world.robot, self.world.arm_joints))
                path = self.rrt(self.bounds, self.world, start_joints, self.radius, end_region)
                for conf in path:
                    wait_for_duration(1/len(path))
                    set_joint_positions(self.world.robot, self.ik_joints, conf.arm_joints)
                    pose = get_link_pose(self.world.robot, self.tool_link)
                    for item in self.item_on_hand:
                        body = 4 if item == 'sugar' else 5
                        set_pose(body, pose)
                        self.poses[item] = pose
                for item in self.item_on_hand:
                    self.poses[item] = end_pose
                    body = 4 if item == 'sugar' else 5
                    set_pose(body, end_pose)
                self.item_on_hand.remove(act.parameters[0])
                # wait_for_user()
                # navigate to drawer
                end_pose = tuple([self.drawer_front_surface[1][0] + 0.1, 
                            (self.drawer_front_surface[0][1] + self.drawer_front_surface[1][1]) / 2, 
                            (self.drawer_front_surface[0][2] + self.drawer_front_surface[1][2]) / 2])
                end_region = Region(end_pose)
                start_joints = Configuration(get_joint_positions(self.world.robot, self.world.base_joints), get_joint_positions(self.world.robot, self.world.arm_joints))
                path = self.rrt(self.bounds, self.world, start_joints, self.radius, end_region)
                for conf in path:
                    wait_for_duration(1/len(path))
                    set_joint_positions(self.world.robot, self.ik_joints, conf.arm_joints)
                # wait_for_user()
                # pull
                start_pose = get_link_pose(self.world.robot, self.tool_link)
                end_pose = ((start_pose[0][0]+0.3, start_pose[0][1], start_pose[0][2]), start_pose[1])
                joint = get_joint(self.world.kitchen, 'indigo_drawer_top_joint')
                # print(get_joint_position(self.world.kitchen, joint))
                for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
                    conf = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, pose, max_time=0.05), None)
                    set_joint_positions(self.world.robot, self.ik_joints, conf)
                    set_joint_position(self.world.kitchen, joint, get_joint_position(self.world.kitchen, joint) + 0.01)
                self.drawer_front_surface = compute_surface_aabb(self.world, 'indigo_drawer_top')
                # wait_for_user()
                # pick
                end_pose = self.poses[act.parameters[0]][0]
                end_region = Region(end_pose)
                start_joints = Configuration(get_joint_positions(self.world.robot, self.world.base_joints), get_joint_positions(self.world.robot, self.world.arm_joints))
                path = self.rrt(self.bounds, self.world, start_joints, self.radius, end_region)
                plan += path
                self.item_on_hand.append(act.parameters[0])
                for conf in path:
                    wait_for_duration(1/len(path))
                    set_joint_positions(self.world.robot, self.ik_joints, conf.arm_joints)
                pose = get_link_pose(self.world.robot, self.tool_link)
                for item in self.item_on_hand:
                    body = 4 if item == 'sugar' else 5
                    set_pose(body, pose)
                # wait_for_user()
                # TODO: place inside
                # TODO: determine end_region
                name = name_from_type('potted_meat_can', 1)
                pose2d = (self.drawer_front_surface[0][0] / 2 + self.drawer_front_surface[1][0] / 2,
                          self.drawer_front_surface[0][1] / 2 + self.drawer_front_surface[1][1] / 2,
                          0)
                end_pose = pose2d_on_surface_virtual(self.world, name, 'indigo_drawer_top', pose2d=pose2d)
                end_region = Region(end_pose[0])
                start_joints = Configuration(get_joint_positions(self.world.robot, self.world.base_joints), get_joint_positions(self.world.robot, self.world.arm_joints))
                path = self.rrt(self.bounds, self.world, start_joints, self.radius, end_region)
                for conf in path:
                    wait_for_duration(1/len(path))
                    set_joint_positions(self.world.robot, self.ik_joints, conf.arm_joints)
                    pose = get_link_pose(self.world.robot, self.tool_link)
                    for item in self.item_on_hand:
                        body = 4 if item == 'sugar' else 5
                        set_pose(body, pose)
                        self.poses[item] = pose
                for item in self.item_on_hand:
                    self.poses[item] = end_pose
                    body = 4 if item == 'sugar' else 5
                    set_pose(body, end_pose)
                self.item_on_hand.remove(act.parameters[0])
                # wait_for_user()
                # navigate to drawer
                end_pose = tuple([self.drawer_front_surface[1][0] + 0.1, 
                            (self.drawer_front_surface[0][1] + self.drawer_front_surface[1][1]) / 2, 
                            (self.drawer_front_surface[0][2] + self.drawer_front_surface[1][2]) / 2])
                end_region = Region(end_pose)
                start_joints = Configuration(get_joint_positions(self.world.robot, self.world.base_joints), get_joint_positions(self.world.robot, self.world.arm_joints))
                path = self.rrt(self.bounds, self.world, start_joints, self.radius, end_region)
                for conf in path:
                    wait_for_duration(1/len(path))
                    set_joint_positions(self.world.robot, self.ik_joints, conf.arm_joints)
                # wait_for_user()
                # push
                start_pose = get_link_pose(self.world.robot, self.tool_link)
                end_pose = ((start_pose[0][0]-0.3, start_pose[0][1], start_pose[0][2]), start_pose[1])
                joint = get_joint(self.world.kitchen, 'indigo_drawer_top_joint')
                name = name_from_type('potted_meat_can', 1)
                for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
                    conf = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, pose, max_time=0.05), None)
                    set_joint_positions(self.world.robot, self.ik_joints, conf)
                    set_joint_position(self.world.kitchen, joint, get_joint_position(self.world.kitchen, joint)-0.01)
                    self.drawer_front_surface = compute_surface_aabb(self.world, 'indigo_drawer_top')
                    pose2d = (self.drawer_front_surface[0][0] / 2 + self.drawer_front_surface[1][0] / 2, 
                              self.drawer_front_surface[0][1] / 2 + self.drawer_front_surface[1][1] / 2, 
                              0)
                    pose3d = pose2d_on_surface(self.world, name, 'indigo_drawer_top', pose2d=pose2d)
                    self.poses['spam'] = pose3d
            else:
                raise NotImplementedError
            