import numpy as np
import math
import matplotlib.pyplot as plt

import config

BEZIER_TRAJECTORY = 'bezier'
MINJERK_TRAJECTORY = 'minJerk'

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
        Position and velocity trajectory if trajectory calculation was succesfull.
    """
    if trajectoryType == BEZIER_TRAJECTORY:
        # return self.__bezierTrajectory(start, goal, duration)
        return _bezierTrajectory(start, goal, duration)
    if trajectoryType == MINJERK_TRAJECTORY:
        # return self.__minJerkTrajectory(start, goal, duration)
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
            - 3 for representing x, y, and z starting position,
            - 4 for representing x, y, z and rotZ, where rotZ is rotation around global z axis,
            - 6 for representing x, y, z, r, p and y pose given in global origin.
        duration (float): Duration of trajectory.

    Raises:
        ValueError: If lengths of start and goal pose are not the same.
        ValueError: If value of duration parameter is smaller or equal to 0.

    Returns:
        tuple: nx7 array, representing pose trajectory with x, y, z, r, p, y and t values, where t are time stamps and 
        nx6 array representing velocity trajectory with x, y, z, r, p, y velocities, where n is the number of steps in trajectory.
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

    timeStep = 1 / config.CONTROLLER_FREQUENCY
    numberOfSteps = int(duration / timeStep)
    timeVector = np.linspace(0, duration, numberOfSteps)

    trajectory = np.empty([len(timeVector), len(startPose) + 1], dtype = np.float32)
    velocities = np.empty([len(timeVector), len(startPose)], dtype = np.float32)
    for idx, t in enumerate(timeVector):
        trajectoryRow = np.empty([len(startPose) + 1], dtype = np.float32)
        velocityRow = np.empty([len(startPose)], dtype = np.float32)
        for i in range(len(startPose)):
            trajectoryRow[i] = startPose[i] + (goalPose[i] - startPose[i]) * (6 * math.pow(t / duration, 5) - 15 * math.pow(t / duration, 4) + 10 * math.pow(t / duration, 3))
            velocityRow[i] = (30 * math.pow(t, 2) * math.pow(duration - t, 2) * (goalPose[i] - startPose[i])) / math.pow(duration, 5)
        trajectoryRow[-1] = t
        trajectory[idx] = trajectoryRow
        velocities[idx] = velocityRow

    plt.plot(trajectory[:, 0])
    plt.plot(trajectory[:, 1])
    plt.plot(trajectory[:, 2])
    plt.legend(['x', 'y', 'z'])
    plt.show()
    plt.plot(velocities[:, 0])
    plt.plot(velocities[:, 1])
    plt.plot(velocities[:, 2])
    plt.legend(['x', 'y', 'z'])
    plt.show()

    return trajectory, velocities

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
        tuple: nx4 array, representing position trajectory with x, y, z and t values, where t are time stamps and 
        nx3 array representing velocity trajectory with x, y and z velocities, where n is the number of steps in trajectory.
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
    heightPercent = 0.4

    startToGoalDirection = np.array(goalPosition - startPosition)
    midPoint = np.array(startToGoalDirection / 2.0)
    startToGoalDirectionUnit = startToGoalDirection / np.linalg.norm(startToGoalDirection)

    if startToGoalDirectionUnit[2] > 0.0:
        orthogonalDirection = np.array([-startToGoalDirectionUnit[0], -startToGoalDirectionUnit[1], (math.pow(startToGoalDirectionUnit[0], 2) + math.pow(startToGoalDirectionUnit[1], 2)) / startToGoalDirectionUnit[2]])
    elif startToGoalDirectionUnit[2] < 0.0:
        orthogonalDirection = np.array([startToGoalDirectionUnit[0], startToGoalDirectionUnit[1], (-math.pow(startToGoalDirectionUnit[0], 2) - math.pow(startToGoalDirectionUnit[1], 2)) / startToGoalDirectionUnit[2]])
    else:
        orthogonalDirection = np.array([0, 0, 1])
    orthogonalDirectionUnit = orthogonalDirection / np.linalg.norm(orthogonalDirection)
    
    firstInterPoint = np.copy(startPosition)
    firstInterPoint[2] += heightPercent * np.linalg.norm(startToGoalDirection)
    secondInterPoint = np.copy(goalPosition)
    secondInterPoint[2] += heightPercent * np.linalg.norm(startToGoalDirection)
    controlPoints =  np.array([startPosition, firstInterPoint, secondInterPoint, goalPosition])

    d = np.array([
        startToGoalDirection[0],
        startToGoalDirection[1],
        (midPoint + orthogonalDirectionUnit * heightPercent)[2]
    ], dtype = np.float32)

    vMax = 2 * (d / duration)
    a = 3 * vMax / duration
    t1 = duration / 3.0
    t2 = 2 * t1

    trajectory = np.empty([len(timeVector), len(startPosition) + 1], dtype = np.float32)
    velocity = np.empty([len(timeVector), len(startPosition)], dtype = np.float32)
    for idx, time in enumerate(timeVector):
        param = time / duration
        trajectoryPoint = controlPoints[0] * math.pow(1 - param, 3) + controlPoints[1] * 3 * param * math.pow(1 - param, 2) + controlPoints[2] * 3 * math.pow(param, 2) * (1 - param) + controlPoints[3] * math.pow(param, 3)
        trajectory[idx] = [trajectoryPoint[0], trajectoryPoint[1], trajectoryPoint[2], time]
        # velocity[idx] = (3 * (-controlPoints[0] * (time - duration)**2 + controlPoints[1] * (3 * time**2 - 4 * time * duration + duration**2) + time * (-3 * controlPoints[2] * time + 2 * controlPoints[2] * duration + controlPoints[3] * time))) / duration**3
        if 0 <= time <= t1:
            velocity[idx] = a * time
        elif t1 < time < t2:
            velocity[idx] = vMax
        elif t2 <= time <= duration:
            velocity[idx] = vMax - a * (time - t2)

    plt.plot(trajectory[:, 0])
    plt.plot(trajectory[:, 1])
    plt.plot(trajectory[:, 2])
    plt.legend(['x', 'y', 'z'])
    plt.show()
    plt.plot(velocity[:, 0])
    plt.plot(velocity[:, 1])
    plt.plot(velocity[:, 2])
    plt.legend(['x', 'y', 'z'])
    plt.show()

    return trajectory, velocity
#endregion