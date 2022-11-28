import pddl_parser
import pddl_parser.PDDL as PDDL
import sys

class ActivityPlanner:
    def appliable(self, conf, action):
        return action.positive_preconditions.issubset(conf) and action.negative_preconditions.isdisjoint(conf)
    
    def act(self, conf, action):
        return conf.difference(action.del_effects).union(action.add_effects)
    
    def BFSPlan(self, parser):
        actions = []
        for action in parser.actions:
            for act in action.groundify(parser.objects, parser.types):
                actions.append(act)
        queue = [(parser.state, [])]
        visited = []
        while len(queue) > 0:
            conf, plan = queue.pop(0)
            if parser.positive_goals.issubset(conf) and parser.negative_goals.isdisjoint(conf):
                return plan
            for action in actions:
                if self.appliable(conf, action):
                    new_conf = self.act(conf, action)
                    if not (new_conf in visited):
                        queue.append((new_conf, plan+[action]))
                        visited.append(new_conf)
        return None
    
    def output(self, plan):
        for act in plan:
            print(act.name + ' ' + ' '.join(act.parameters))
            print(act.parameters)

parser = PDDL.PDDL_Parser()
parser.parse_domain(sys.argv[1])
parser.parse_problem(sys.argv[2])
planner = ActivityPlanner()
plan = planner.BFSPlan(parser)
planner.output(plan)