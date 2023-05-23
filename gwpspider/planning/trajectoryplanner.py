import numpy as np

import config

#region public methods
def get_trajectory(leg_current_position: np.ndarray, leg_goal_position: np.ndarray, duration: float, trajectory_type: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calculate trajectories of desired type.

    Args:
        leg_current_position (np.ndarray): Start pose or position.
        leg_goal_position (np.ndarray): Goal pose or position.
        duration (float): Desired duration of movement.
        trajectory_type (str): Desired trajectory type (bezier or min jerk).

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray]: Position, velocity and acceleration trajectory, if calculation was succesfull.
    """
    if trajectory_type == config.BEZIER_TRAJECTORY:
        return _bezierTrajectory(leg_current_position, leg_goal_position, duration)
    if trajectory_type == config.MINJERK_TRAJECTORY:
        return _min_jerk_trajectory(leg_current_position, leg_goal_position, duration)
#endregion

#region private methods
def _min_jerk_trajectory(start_pose: np.ndarray, goal_pose: np.ndarray, duration: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calculate minimum jerk trajectory of positions and velocities between two points.

    Args:
        start_pose (np.ndarray): 1xn array of starting pose, where n can be: 
            - 3 for representing x, y, and z starting position,
            - 4 for representing x, y, z and rotZ, where rotZ is rotation around global z axis,
            - 6 for representing x, y, z, r, p and y pose given in global origin.
        goal_pose (np.ndarray): 1xn array of goal pose, where n can be: 
            - 3 for representing x, y, and z goal position,
            - 4 for representing x, y, z and rotZ, where rotZ is rotation around global z axis,
            - 6 for representing x, y, z, r, p and y pose given in global origin.
        duration (float): Duration of trajectory.

    Raises:
        ValueError: If lengths of start and goal pose are not the same.
        ValueError: If value of duration parameter is smaller or equal to 0.

    Returns:
        Tuple: Three nx7 array, representing pose trajectory with x, y, z, r, p, y and t values, where t are time stamps, 
        velocity trajectory and acceleration trajectory, where n is the number of steps in trajectory.
    """
    if len(start_pose) == 3:
        start_pose = [start_pose[0], start_pose[1], start_pose[2], 0.0 , 0.0, 0.0]
    elif len(start_pose) == 4:
        start_pose = [start_pose[0], start_pose[1], start_pose[2], 0.0, 0.0, start_pose[3]]
    
    if len(goal_pose) == 3:
        goal_pose = [goal_pose[0], goal_pose[1], goal_pose[2], 0.0, 0.0, 0.0]
    elif len(goal_pose) == 4:
        goal_pose = [goal_pose[0], goal_pose[1], goal_pose[2], 0.0, 0.0, goal_pose[3]]

    if (len(start_pose) != len(goal_pose)):
        raise ValueError("Lengths of start_pose and goal_pose do not match.")
    if duration <= 0:
        raise ValueError("Movement duration cannot be shorter than 0 seconds.")
    
    start_pose = np.array(start_pose, dtype = np.float32)
    goal_pose = np.array(goal_pose, dtype = np.float32)

    time_step = 1 / config.CONTROLLER_FREQUENCY
    number_of_steps = int(duration / time_step)
    time_vector = np.linspace(0, duration, number_of_steps)

    trajectory = np.zeros((len(time_vector), len(start_pose) + 1), dtype = np.float32)
    velocities = np.zeros((len(time_vector), len(start_pose)), dtype = np.float32)
    accelerations = np.zeros((len(time_vector), len(start_pose)), dtype = np.float32)

    for idx, t in enumerate(time_vector):
        trajectory[idx, :-1] = start_pose + (goal_pose - start_pose) * (6 * (t / duration)**5 - 15 * (t / duration)**4 + 10 * (t / duration)**3)
        trajectory[idx, -1] = t
        velocities[idx] = (30 * t**2 * (duration - t)**2 * (goal_pose - start_pose)) / duration**5
        accelerations[idx] = -(60 * (start_pose - goal_pose) * t * (2 * t**2 - 3 * t * duration + duration**2)) / duration**5

    return trajectory, velocities, accelerations

def _bezierTrajectory(start_position: np.ndarray, goal_position: np.ndarray, duration: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calculate cubic bezier trajectory between start and goal point with fixed intermediat control points.

    Args:
        start_position (np.ndarray): 1x3 array of x, y, z values, representing starting position of trajectory, given in global origin.
        goal_position (np.ndarray): 1x3 array of x, y, z values, representing goal position of trajectory, given in global origin.
        duration (float): Duration of trajectory.

    Raises:
        ValueError: If lengths of given start and/or goal pose are not equal to 3.
        ValueError: If value of duration parameter is smaller or equal to 0.

    Returns:
        numpy.ndarray: nx4 array, representing position trajectory with x, y, z and t values, where t are time stamps,
        velocity trajectory and acceleration trajectory, where n is the number of steps in trajectory.
    """

    if len(start_position) != 3 or len(goal_position) != 3:
        raise ValueError("Start and/or goal position were not given correctly.")
    if duration <= 0:
        raise ValueError("Movement duration cannot be shorter than 0 seconds.")

    start_position, goal_position = np.array(start_position), np.array(goal_position)

    time_step = 1 / config.CONTROLLER_FREQUENCY
    number_of_steps = int(duration / time_step)
    time_vector = np.linspace(0, duration, number_of_steps)

    # Ratio beween height of trajectory and distance between start and goal points.
    height_percent = 0.8 if np.linalg.norm(goal_position - start_position) > 0.25 else 1.0

    start_to_goal_direction = np.array(goal_position - start_position)

    p_0 = np.copy(start_position)
    p_1 = np.copy(start_position)
    p_2 = np.copy(start_position)

    p_3 = np.copy(start_position)
    p_3[2] += height_percent * np.linalg.norm(start_to_goal_direction)
    p_4 = np.copy(goal_position)
    p_4[2] += height_percent * np.linalg.norm(start_to_goal_direction)

    p_5 = np.copy(goal_position)
    p_6 = np.copy(goal_position)
    p_7 = np.copy(goal_position)

    trajectory = np.empty([len(time_vector), len(start_position) + 1], dtype = np.float32)
    velocity = np.empty([len(time_vector), len(start_position)], dtype = np.float32)
    accelerations = np.empty([len(time_vector), len(start_position)], dtype = np.float32)

    for idx, t in enumerate(time_vector):
        param = t / duration

        trajectory_point = p_0 * (1 - param)**7 + p_1 * 7 * (1 - param)**6 * param + p_2 * 21 * (1 - param)**5 * param**2 + p_3 * 35 * (1 - param)**4 * param**3 + \
            p_4 * 35 * (1 - param)**3 * param**4 + p_5 * 21 * (1 - param)**2 * param**5 + p_6 * 7 * (1 - param) * param**6 + p_7 * param**7
        trajectory[idx] = [trajectory_point[0], trajectory_point[1], trajectory_point[2], t]

        velocity[idx] = -(7 * (p_0 * (t - duration)**6 - p_1 * (t - duration)**5 * (7 * t - duration) + t * (3 * p_2 * (7 * t - 2 * duration) * (t - duration)**4 - \
            t * (5 * p_3 * ( 7 * t - 3 * duration) * (t - duration)**3 + t * (-5 * p_4 * (7 * t - 4 * duration) * (t - duration)**2 + \
                t * (t * (-7 * p_6 * t + p_7 * t + 6 * p_6 * duration) + 3 * p_5 * (7 * t**2 - 12 * t * duration + 5 * duration**2))))))) / duration**7

        accelerations[idx] = 42 * (-21 * p_2 * t**5 + 35 * p_3 * t**5 - 35 * p_4 * t**5 + 21 * p_5 * t**5 - 7 * p_6 * t**5 + p_7 * t**5 + p_1 * (7 * t - 2 * duration) * (t - duration)**4 - \
            p_0 * (t - duration)**5 + 75 * p_2 * t**4 * duration - 100 * p_3 * t**4 * duration + 75 * p_4 * t**4 * duration - 30 * p_5 * t**4 * duration + \
                  5 * p_6 * t**4 * duration - 100 * p_2 * t**3 * duration**2 + 100 * p_3 * t**3 * duration**2 - 50 * p_4 * t**3 * duration**2 + \
                    10 * p_5 * t**3 * duration**2 + 60 * p_2 * t**2 * duration**3 - 40 * p_3 * t**2 * duration**3 + \
                        10 * p_4 * t**2 * duration**3 - 15 * p_2 * t * duration**4 + 5 * p_3 * t * duration**4 + p_2 * duration**5) / duration**7

    return trajectory, velocity, accelerations
#endregion