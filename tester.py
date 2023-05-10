from planning import trajectoryplanner as tp

import numpy as np
import config


if __name__ == '__main__':
    start = np.array([0, 0, 0])
    goal = np.array([0.5, 0, 0])

    _ = tp.getTrajectory(start, goal, 5, config.BEZIER_TRAJECTORY)
    # _ = tp.getTrajectory(start, goal, 5, config.MINJERK_TRAJECTORY)






