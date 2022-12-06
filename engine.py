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
motion_planner = MotionPlanner(plan, use_gui=False)
motion_planner.motion_plan()
real_planner = MotionPlanner(plan)
real_planner.run_plan()