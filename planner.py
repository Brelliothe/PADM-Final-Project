import pddl_parser
import pddl_parser.PDDL as PDDL
import sys

class Planner():
    def appliable(self, conf, action):
        return action.positive_preconditions.issubset(conf) and action.negative_preconditions.isdisjoint(conf)
    
    def act(self, conf, action):
        return conf.difference(action.del_effects).union(action.add_effects)
    
    def BFSPlan(self, parser):
        actions = []
        for action in parser.actions:
            for act in action.groundify(parser.objects, parser.types):
                actions.append(act)
        queue = [(parser.state, None)]
        visited = []
        while len(queue) > 0:
            print(len(queue))
            conf, plan = queue.pop(0)
            if parser.positive_goals.issubset(conf) and parser.negative_goals.isdisjoint(conf):
                return plan
            for action in actions:
                if self.appliable(conf, action):
                    plan.append(action)
                    conf = self.act(conf, action)
                    if not conf in visited:
                        queue.append((conf, plan))
                        visited.append(conf)
        return None


parser = PDDL.PDDL_Parser()
parser.parse_domain(sys.argv[1])
parser.parse_problem(sys.argv[2])
planner = Planner()
plan = planner.BFSPlan(parser)
print(plan)