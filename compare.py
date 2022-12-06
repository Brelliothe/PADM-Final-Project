from motion import MotionPlanner
import numpy as np
import sys

traj = np.load('trajectory_{}.npy'.format(sys.argv[1]), allow_pickle=True)

planner = MotionPlanner(None)
planner.run_trajectory(traj)