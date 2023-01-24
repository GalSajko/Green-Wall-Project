import numpy as np
import math
import matplotlib.pyplot as plt

import config

#region public methods
def calculateTrajectory(start, goal, duration, trajectoryType):
    """Wrapper for calcuating trajectories of desired type.

    Args:
        start: Start pose or position.
        goal: Goal pose or position.
        duration: Desired duration of movement.
        trajectoryType: Desired trajectory type (bezier or minJerk).

    Raises:
        ValueError: If trajectory type is unknown.      

    Returns:
        Position trajectory if trajectory calculation was succesfull.
    """
    if trajectoryType == config.BEZIER_TRAJECTORY:
        return _bezierTrajectory(start, goal, duration)
    if trajectoryType == config.MINJERK_TRAJECTORY:
        return _minJerkTrajectory(start, goal, duration)

    raise ValueError("Unknown trajectory type!")
#endregion

#region private methods
def _minJerkTrajectory(startPose, goalPose, duration):
    """Calculate minimum jerk trajectory of positions and velocities between two points.

    Args:
        startPose (list): 1xn array of starting pose, where n can be: 
            - 3 for representing x, y, and z starting position,
            - 4 for representing x, y, z and rotZ, where rotZ is rotation around global z axis,
            - 6 for representing x, y, z, r, p and y pose given in global origin.
        goalPose (list): 1xn array of goal pose, where n can be: 
            - 3 for representing x, y, and z goal position,
            - 4 for representing x, y, z and rotZ, where rotZ is rotation around global z axis,
            - 6 for representing x, y, z, r, p and y pose given in global origin.
        duration (float): Duration of trajectory.

    Raises:
        ValueError: If lengths of start and goal pose are not the same.
        ValueError: If value of duration parameter is smaller or equal to 0.

    Returns:
        numpy.ndarray: nx7 array, representing pose trajectory with x, y, z, r, p, y and t values, where t are time stamps and n is the number of steps in trajectory.
    """
    if len(startPose) == 3:
        startPose = [startPose[0], startPose[1], startPose[2], 0.0 , 0.0, 0.0]
    elif len(startPose) == 4:
        startPose = [startPose[0], startPose[1], startPose[2], 0.0, 0.0, startPose[3]]
    
    if len(goalPose) == 3:
        goalPose = [goalPose[0], goalPose[1], goalPose[2], 0.0, 0.0, 0.0]
    elif len(goalPose) == 4:
        goalPose = [goalPose[0], goalPose[1], goalPose[2], 0.0, 0.0, goalPose[3]]

    if (len(startPose) != len(goalPose)):
        raise ValueError("Lengths of startPose and goalPose do not match.")
    if duration <= 0:
        raise ValueError("Movement duration cannot be shorter than 0 seconds.")
    
    startPose = np.array(startPose, dtype = np.float32)
    goalPose = np.array(goalPose, dtype = np.float32)

    timeStep = 1 / config.CONTROLLER_FREQUENCY
    numberOfSteps = int(duration / timeStep)
    timeVector = np.linspace(0, duration, numberOfSteps)

    trajectory = np.zeros((len(timeVector), len(startPose) + 1), dtype = np.float32)
    velocities = np.zeros((len(timeVector), len(startPose)), dtype = np.float32)
    accelerations = np.zeros((len(timeVector), len(startPose)), dtype = np.float32)

    for idx, t in enumerate(timeVector):
        # trajectoryRow = np.empty((len(startPose) + 1), dtype = np.float32)
        # velocityRow = np.empty((len(startPose)), dtype = np.float32)
        # accelerationRow = np.empty((len(startPose)), dtype = np.float32)
        # for i in range(len(startPose)):
        #     trajectoryRow[i] = startPose[i] + (goalPose[i] - startPose[i]) * (6 * (t / duration)**5 - 15 * (t / duration)**4 + 10 * (t / duration)**3)
        #     velocityRow[i] = (30 * t**2 * (duration - t)**2 * (goalPose[i] - startPose[i])) / duration**5
        #     accelerationRow[i] = -(60 * (startPose[i] - goalPose[i]) * t * (2 * t**2 - 3 * t * duration + duration**2)) / duration**5
        # trajectoryRow[-1] = t
        # trajectory[idx] = trajectoryRow
        # velocities[idx] = velocityRow
        # accelerations[idx] = accelerationRow
        trajectory[idx, :-1] = startPose + (goalPose - startPose) * (6 * (t / duration)**5 - 15 * (t / duration)**4 + 10 * (t / duration)**3)
        trajectory[idx, -1] = t
        velocities[idx] = (30 * t**2 * (duration - t)**2 * (goalPose - startPose)) / duration**5
        accelerations[idx] = -(60 * (startPose - goalPose) * t * (2 * t**2 - 3 * t * duration + duration**2)) / duration**5

    return trajectory, velocities, accelerations

def _bezierTrajectory(startPosition, goalPosition, duration):
    """Calculate cubic bezier trajectory between start and goal point with fixed intermediat control points.

    Args:
        startPosition (list): 1x3 array of x, y, z values, representing starting position of trajectory, given in global origin.
        goalPosition (list): 1x3 array of x, y, z values, representing goal position of trajectory, given in global origin.
        duration (float): Duration of trajectory.

    Raises:
        ValueError: If lengths of given start and/or goal pose are not equal to 3.
        ValueError: If value of duration parameter is smaller or equal to 0.

    Returns:
        numpy.ndarray: nx4 array, representing position trajectory with x, y, z and t values, where t are time stamps and n is the number of steps in trajectory.
    """

    if len(startPosition) != 3 or len(goalPosition) != 3:
        raise ValueError("Start and/or goal position were not given correctly.")
    if duration <= 0:
        raise ValueError("Movement duration cannot be shorter than 0 seconds.")

    startPosition, goalPosition = np.array(startPosition), np.array(goalPosition)

    timeStep = 1 / config.CONTROLLER_FREQUENCY
    numberOfSteps = int(duration / timeStep)
    timeVector = np.linspace(0, duration, numberOfSteps)

    # Ratio beween height of trajectory and distance between start and goal points.
    heightPercent = 0.8

    startToGoalDirection = np.array(goalPosition - startPosition)

    P0 = np.copy(startPosition)
    P1 = np.copy(startPosition)
    P2 = np.copy(startPosition)

    P3 = np.copy(startPosition)
    P3[2] += heightPercent * np.linalg.norm(startToGoalDirection)
    P4 = np.copy(goalPosition)
    P4[2] += heightPercent * np.linalg.norm(startToGoalDirection)

    P5 = np.copy(goalPosition)
    P6 = np.copy(goalPosition)
    P7 = np.copy(goalPosition)

    trajectory = np.empty([len(timeVector), len(startPosition) + 1], dtype = np.float32)
    velocity = np.empty([len(timeVector), len(startPosition)], dtype = np.float32)
    accelerations = np.empty([len(timeVector), len(startPosition)], dtype = np.float32)

    for idx, t in enumerate(timeVector):
        param = t / duration

        trajectoryPoint = P0 * (1 - param)**7 + P1 * 7 * (1 - param)**6 * param + P2 * 21 * (1 - param)**5 * param**2 + P3 * 35 * (1 - param)**4 * param**3 + \
            P4 * 35 * (1 - param)**3 * param**4 + P5 * 21 * (1 - param)**2 * param**5 + P6 * 7 * (1 - param) * param**6 + P7 * param**7
        trajectory[idx] = [trajectoryPoint[0], trajectoryPoint[1], trajectoryPoint[2], t]

        velocity[idx] = -(7 * (P0 * (t - duration)**6 - P1 * (t - duration)**5 * (7 * t - duration) + t * (3 * P2 * (7 * t - 2 * duration) * (t - duration)**4 - \
            t * (5 * P3 * ( 7 * t - 3 * duration) * (t - duration)**3 + t * (-5 * P4 * (7 * t - 4 * duration) * (t - duration)**2 + \
                t * (t * (-7 * P6 * t + P7 * t + 6 * P6 * duration) + 3 * P5 * (7 * t**2 - 12 * t * duration + 5 * duration**2))))))) / duration**7

        accelerations[idx] = 42 * (-21 * P2 * t**5 + 35 * P3 * t**5 - 35 * P4 * t**5 + 21 * P5 * t**5 - 7 * P6 * t**5 + P7 * t**5 + P1 * (7 * t - 2 * duration) * (t - duration)**4 - \
            P0 * (t - duration)**5 + 75 * P2 * t**4 * duration - 100 * P3 * t**4 * duration + 75 * P4 * t**4 * duration - 30 * P5 * t**4 * duration + \
                  5 * P6 * t**4 * duration - 100 * P2 * t**3 * duration**2 + 100 * P3 * t**3 * duration**2 - 50 * P4 * t**3 * duration**2 + \
                    10 * P5 * t**3 * duration**2 + 60 * P2 * t**2 * duration**3 - 40 * P3 * t**2 * duration**3 + \
                        10 * P4 * t**2 * duration**3 - 15 * P2 * t * duration**4 + 5 * P3 * t * duration**4 + P2 * duration**5) / duration**7
    

    # plt.subplot(1, 3, 1)
    # plt.plot(timeVector, trajectory[:, 0], 'g*')
    # plt.plot(timeVector, trajectory[:, 1], 'r*')
    # plt.plot(timeVector, trajectory[:, 2], 'b*')   
    # plt.legend(['x', 'y', 'z'])
    # plt.title("Positions")
    # plt.subplot(1, 3, 2)
    # plt.plot(timeVector, velocity[:, 0], 'g*')
    # plt.plot(timeVector, velocity[:, 1], 'r*')
    # plt.plot(timeVector, velocity[:, 2], 'b*')
    # plt.legend(['x', 'y', 'z'])
    # plt.title("Velocities")
    # plt.subplot(1, 3, 3)
    # plt.plot(timeVector, acceleration[:, 0], 'g*')
    # plt.plot(timeVector, acceleration[:, 1], 'r*')
    # plt.plot(timeVector, acceleration[:, 2], 'b*')
    # plt.legend(['x', 'y', 'z'])
    # plt.title("Acceleration")

    # plt.show()

    return trajectory, velocity, accelerations
#endregion