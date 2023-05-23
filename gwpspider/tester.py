from planning import trajectoryplanner as tp
from planning import pathplanner as pp
from gwpconfig import commconstants

import numpy as np
import config


if __name__ == '__main__':
    start_pose = np.array([0, 0, 0, 0])
    goal_pose = np.array([0.5, 0.5, 0, 0])

    path = pp.calculate_spider_body_path(start_pose, goal_pose)
    print(path)
    # _ = tp.get_trajectory(start, goal, 5, config.MINJERK_TRAJECTORY)






