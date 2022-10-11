import numba
import numpy as np

import config

class NumbaWrapper:
    @numba.njit
    def calculateGravityVectors(gravityRotationMatrices, spiderGravityVector):
        """Calculate gravity vectors in segments' origins.

        Args:
            gravityRotationMatrices (list): 5x3x3 array of five 3x3 rotation matrices.
            spiderGravityVector (list): 1x3 gravity vector in spider's origin.

        Returns:
            numpy.ndarray: 3x3 array of three local gravity vectors, given in segments' origins.
        """
        localGravityVectors = np.zeros((3, 3), dtype = np.float32)
        for i in range(3):
            localGravityVectors[i] = np.dot(np.transpose(gravityRotationMatrices[i]), spiderGravityVector)
        return localGravityVectors

    @numba.njit
    def calculateForcesAndTorques(vectorsToCog, legsDimensions, segmentMasses, forceRotationMatrices, localGravityVectors):
        """Calculate forces on leg-tip and torques in the motors, using Newton-Euler method.

        Args:
            vectorsToCog (list): 1x3 array of three local vectors to centers of gravity of segments.
            legsDimensions (list): 1x3 array of leg dimensions in segments' x direction.
            segmentMasses (list): 1x3 array of masses of three segments.
            forceRotationMatrices (list): 3x3x3 array of three rotation matrices.
            localGravityVectors (list): 3x3 array of three gravity vectors in segments' origins.

        Returns:
            numpy.ndarray: 1x3 array of torques in joints.
        """
        torquesVectorsInLeg = np.zeros((3, 3), dtype = np.float32)
        torquesValuesInLeg = np.zeros(3)
        forces = np.zeros((3, 3), dtype = np.float32)
        
        for i in range(3):
            lc = np.array([1, 0, 0], dtype = np.float32) * vectorsToCog[2 - i]
            l = np.array([1, 0, 0], dtype = np.float32) * legsDimensions[2 - i]

            if i != 0:
                fgSegment = segmentMasses[2 - i] * localGravityVectors[2 - i]
                forces[i] = np.dot(forceRotationMatrices[i - 1], forces[i - 1]) - fgSegment
                torquesVectorsInLeg[i] = np.dot(forceRotationMatrices[i - 1], torquesVectorsInLeg[i - 1]) + np.cross(fgSegment, lc) - np.cross(np.dot(forceRotationMatrices[i - 1], forces[i - 1]), l)
                torquesValuesInLeg[i] = torquesVectorsInLeg[i][2]
                continue

            forces[i] = -segmentMasses[2 - i] * localGravityVectors[2 - i]
            torquesVectorsInLeg[i] = (-1) * np.cross(forces[i], lc)
            torquesValuesInLeg[i] = torquesVectorsInLeg[i][2]
        
        return torquesValuesInLeg
        
    @numba.njit
    def minJerkTrajectory(startPose, goalPose, duration):
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
            startPose = np.array([startPose[0], startPose[1], startPose[2], 0.0 , 0.0, 0.0], dtype = np.float32)
        elif len(startPose) == 4:
            startPose = np.array([startPose[0], startPose[1], startPose[2], 0.0, 0.0, startPose[3]], dtype = np.float32)
        
        if len(goalPose) == 3:
            goalPose = np.array([goalPose[0], goalPose[1], goalPose[2], 0.0, 0.0, 0.0], dtype = np.float32)
        elif len(goalPose) == 4:
            goalPose = np.array([goalPose[0], goalPose[1], goalPose[2], 0.0, 0.0, goalPose[3]], dtype = np.float32)

        if (len(startPose) != len(goalPose)):
            raise ValueError("Lengths of startPose and goalPose do not match.")
        if duration <= 0:
            raise ValueError("Movement duration cannot be shorter than 0 seconds.")

        timeStep = 1 / config.CONTROLLER_FREQUENCY
        numberOfSteps = int(duration / timeStep)
        timeVector = np.linspace(0, duration, numberOfSteps)

        trajectory = np.zeros((len(timeVector), len(startPose) + 1), dtype = np.float32)
        velocities = np.zeros((len(timeVector), len(startPose)), dtype = np.float32)
        for idx, t in enumerate(timeVector):
            trajectoryRow = np.zeros((len(startPose) + 1), dtype = np.float32)
            velocityRow = np.zeros((len(startPose)), dtype = np.float32)
            for i in range(len(startPose)):
                trajectoryRow[i] = startPose[i] + (goalPose[i] - startPose[i]) * (6 * (t / duration)**5 - 15 * (t / duration)**4 + 10 * (t / duration)**3)
                velocityRow[i] = (30 * t**2 * (duration - t)**2 * (goalPose[i] - startPose[i])) / duration**5
            trajectoryRow[-1] = t
            trajectory[idx] = trajectoryRow
            velocities[idx] = velocityRow
        return trajectory, velocities    