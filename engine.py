from activity import ActivityPlanner
from motion import MotionPlanner
import pddl_parser
import pddl_parser.PDDL as PDDL
import sys

parser = PDDL.PDDL_Parser()
parser.parse_domain('kitchen.pddl')
parser.parse_problem('task.pddl')
planner = ActivityPlanner()
plan = planner.BFSPlan(parser)
motion_planner = MotionPlanner(plan)
motion_planner.motion_plan()