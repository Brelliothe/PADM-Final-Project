from motion import MotionPlanner
import numpy as np
import os

from pydrake.all import (Binding, Box, DiagramBuilder, DirectCollocation,
                         DirectTranscription,
                         FiniteHorizonLinearQuadraticRegulatorOptions,
                         GraphOfConvexSets, GraphOfConvexSetsOptions,
                         GurobiSolver, HPolyhedron, L2NormCost, LinearSystem,
                         LogVectorOutput,
                         MakeFiniteHorizonLinearQuadraticRegulator,
                         MakeFirstAvailableSolver, MathematicalProgram,
                         MosekSolver, MultibodyPlant,
                         MultibodyPositionToGeometryPose, Parser,
                         PiecewisePolynomial, PlanarSceneGraphVisualizer,
                         Point, PointCloud, Rgba, RigidTransform,
                         RotationMatrix, SceneGraph, Simulator, Solve, Sphere,
                         StartMeshcat, TrajectorySource, Variable, eq, le, ge)

if not os.path.exists('trajectory_rrt.npy'):
    planner = MotionPlanner(None, False)
    planner.generate_trajectory()

trajectory = np.load('trajectory_rrt.npy', allow_pickle=True)
num = len(trajectory)
# create decision variables
x = np.empty((7, num), dtype=Variable)
prog = MathematicalProgram()
for n in range(num):
    x[:, n] = prog.NewContinuousVariables(7, 'x' + str(n))

# add cost
prog.AddCost(np.sum(np.subtract(x[:, 1:], x[:, :-1]) ** 2))

# add contraints
prog.AddConstraint(eq(x[:, 0], trajectory[0]))
prog.AddConstraint(eq(x[:, -1], trajectory[-1]))
for n in range(num-1):
    prog.AddConstraint(le(np.subtract(x[:, n+1], x[:, n]), 0.3 * np.ones(7)))
    prog.AddConstraint(ge(np.subtract(x[:, n+1], x[:, n]), -0.3 * np.ones(7)))

result = Solve(prog)
x_sol = result.GetSolution(x)
np.save('trajectory_opt.npy', x_sol.T)