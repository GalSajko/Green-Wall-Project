from planning import trajectoryplanner as tp
from planning import pathplanner as pp
from gwpconfig import commconstants
from utils import csvfilemanager as cm
from utils import jsonfilemanager as jm


import numpy as np
import config


if __name__ == '__main__':
    start_pose = np.array([0, 0, 0, 0])
    goal_pose = np.array([0.5, 0.5, 0, 0])

    path = pp.calculate_spider_body_path(start_pose, goal_pose)
    print(path)

    csv_manager = cm.CsvFileManager()
    json_manager = jm.JsonFileManager()

    data = json_manager.read_spider_state()
    print(data)

    # _ = tp.get_trajectory(start, goal, 5, config.MINJERK_TRAJECTORY)






