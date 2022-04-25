import numpy as np

import calculations 
import environment as env
import dynamixel as dmx

class VelocityController:
    """ Class for velocity-control of spider's movement.
    """
    def __init__ (self):
        self.matrixCalculator = calculations.MatrixCalculator()
        self.kinematics = calculations.Kinematics()
        self.spider = env.Spider()
        self.motorsDriver = dmx.MotorDriver(False)

    def getSpiderToLegReferenceVelocities(self, spiderVelocity):
        """Calculate current reference leg velocities from current spider velocity.

        :param spiderVelocity: Current spider's velocity.
        :return: Reference legs velocities.
        """
        linearSpiderVelocity = spiderVelocity[:3]
        anguarSpiderVelocity = spiderVelocity[3:]

        # Rotate spiders reference velocity into anchors velocities which represent reference velocities for single leg.
        anchorsVelocities = [np.dot(np.linalg.inv(tAnchor[:3,:3]), linearSpiderVelocity) for tAnchor in self.spider.T_ANCHORS]
        # Add angular velocities.
        for i, anchorVelocity in enumerate(anchorsVelocities):
            anchorPosition = self.spider.LEG_ANCHORS[i]
            wx, wy, wz = anguarSpiderVelocity
            anchorsVelocities[i] = np.array([
                anchorVelocity[0],
                anchorVelocity[1] + self.spider.BODY_RADIUS * wz,
                anchorVelocity[2] - anchorPosition[0] * wy + anchorPosition[1] * wx
            ])
        refereneceLegVelocities = np.copy(anchorsVelocities) * (-1)

        return np.array(refereneceLegVelocities)

    # def movePlatform(self, startGlobalLegsPositions, trajectory, velocity):
    #     """Move platform by sending calculated reference values to motors.

    #     :param startGlobalLegsPosition: Starting legs positions in global origin.
    #     :param trajectory: Trajectory for spider's movement.
    #     :param velocity: Velocities in each step of trajectory.
    #     :return: Reference motors positions and velocities for each leg in each step of trajectory.
    #     """
    #     qD = []
    #     qDd = []
    #     timeStep = trajectory[1][-1] - trajectory[0][-1]
    #     for idx, traj in enumerate(trajectory):

    def moveLeg(self, legIdx, trajectory, velocity):
        """Move leg by sending calculated reference values to motors.

        :param legIdx: Leg id.
        :param trajectory: Trajectory.
        :param velocity: Velocities in each trajectory step.
        """
            
            

    def getCurrentQdQddPlatform(self, currentPose, currentVelocity, globalLegsPositions):
        """Calculate current reference positions and velocities for motors from current spider's pose and velocity.

        :param currentPose: Current spider's pose.
        :param currentVelocity: Current spider's velocity
        :param globalLegsPositions: Legs positions during platform movement in global origin.
        :return: Current Qd and Qdd.
        """
        currentQd = self.kinematics.platformInverseKinematics(currentPose, globalLegsPositions)
        referenceLegsVelocities = self.getSpiderToLegReferenceVelocities(currentVelocity)
        currentQdd = []
        for leg in range(self.spider.NUMBER_OF_LEGS):
            J = self.kinematics.legJacobi(leg, currentQd[leg])
            currentQdd.append(np.dot(np.linalg.inv(J), referenceLegsVelocities[leg]))
        
        return np.array(currentQd), np.array(currentQdd)
    
    def legController(self, currentQdLeg, currentQddLeg, legIdx):
        """Calculate reference motor values for single leg, for moving along a given trajectory.

        :param currentQd: Current motors reference positions.
        :param currentQdd: Current motors reference velocities.
        :param currentQa: Current motors positions.
        :return: Reference motors values.
        """  
        # Read current leg-motors positions.
        qA = self.motorsDriver.readMotorsPositionsInLeg(legIdx)
        # Position error.
        error = currentQdLeg - qA
        # Position P-controller (maybe upgrade to PD-controller later).
        K = 2
        qC = K * error
        # Add reference velocity.
        qCdot = qC + currentQddLeg

        return np.array(qCdot)
        




