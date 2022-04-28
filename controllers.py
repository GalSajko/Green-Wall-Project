from os import times
import numpy as np

import calculations 
import environment as env
import dynamixel as dmx
import time

class VelocityController:
    """ Class for velocity-control of spider's movement.
    """
    def __init__ (self):
        self.matrixCalculator = calculations.MatrixCalculator()
        self.kinematics = calculations.Kinematics()
        self.geometryTools = calculations.GeometryTools()
        self.spider = env.Spider()
        motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
        self.motorDriver = dmx.MotorDriver(motors, False)

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
    
    def getQdQddLegFF(self, legIdx, xD, xDd):
        """Feed forward calculations of reference joints positions and velocities for single leg movement.

        :param legIdx: Leg id.
        :param xD: Leg tip reference trajectory.
        :param xDd: Leg tip reference velocities.
        :return: Matrices of reference joints positions and velocities.
        """
        qD = []
        qDd = []
        for idx, pose in enumerate(xD):
            currentQd = self.kinematics.legInverseKinematics(legIdx, pose[:3])
            J = self.kinematics.legJacobi(legIdx, currentQd)
            currentQdd = np.dot(np.linalg.inv(J), xDd[idx][:3])
            qD.append(currentQd)
            qDd.append(currentQdd)
        
        return np.array(qD), np.array(qDd)
    
    def getQdQddPlatformFF(self, xD, xDd, globalLegsPositions):
        """Feed forward calculations of reference joints positions and velocities for parallel platform movement. 

        :param xD: Platform referenece positions.
        :param xDd: Platform reference velocities.
        :param globalLegsPositions: Legs positions during parallel movement in global origin.
        :return: Matrices of reference joints positions and velocities.
        """
        qD = []
        qDd = []
        for idx, pose in enumerate(xD):
            currentQd = self.kinematics.platformInverseKinematics(pose[:,-1], globalLegsPositions)
            referenceLegsVelocities = self.getSpiderToLegReferenceVelocities(xDd[idx])
            currentQdd = []
            for leg in range(self.spider.NUMBER_OF_LEGS):
                J = self.kinematics.legJacobi(leg, currentQd[leg])
                currentQdd.append(np.dot(np.linalg.inv(J), referenceLegsVelocities))
            qD.append(currentQd)
            qDd.append(currentQdd)
        
        return np.array(qD), np.array(qDd)

    def moveLegs(self, legsIds, trajectories, velocities):
        """Move any number of legs (within number of spiders legs) along given trajectories.

        :param ledIds: Array of legs ids.
        :param trajectory: 2D array of legs tips trajectories.
        :param velocity: 2D array of legs tips velocities.
        :return: True if movements were successfull, false otherwise.
        """
        # if len(legsIds) > 2:
        #     print("Cannot move more than 2 legs at the same time!")
        #     return False
        if len(legsIds) != len(trajectories) and len(legsIds) != len(velocities):
            print("Invalid parameters.")
            return False
        
        qDs = []
        qDds = []
        # Time steps of all trajectories should be the same (it is defined in trajectory planner).
        timeStep = trajectories[0][:,-1][1] - trajectories[0][:,-1][0]
        self.motorDriver.clearGroupSyncReadParams()
        self.motorDriver.clearGroupSyncWriteParams()
        if not self.motorDriver.addGroupSyncReadParams(legsIds):
            return False
        for idx, leg in enumerate(legsIds):
            qD, qDd = self.getQdQddLegFF(leg, trajectories[idx], velocities[idx])
            qDs.append(qD)
            qDds.append(qDd)
        # Index of longer trajectory:
        longerIdx = trajectories.index(max(trajectories, key = len))
        # lastErrors = np.zeros(len(legsIds))
        lastErrors = np.zeros([len(legsIds), 3])
        Kp = 10
        Kd = 1
        # Use indexes of longest trajectory.
        for i, _ in enumerate(trajectories[longerIdx]):
            # Read motors positions in all legs.
            qA = self.motorDriver.syncReadMotorsPositionsInLegs(legsIds)
            qCds = []
            startTime = time.time()
            for l, leg in enumerate(legsIds):
                # If index is still whithin boundaries of current trajectory.
                if i < len(trajectories[l]) - 1:
                    currentQd = qDs[l][i]
                    currentQdd = qDds[l][i]
                    currentQa = qA[l]
                    error = currentQd - currentQa
                    dE = (error - lastErrors[l]) / timeStep
                    qCd = Kp * error + Kd * dE + currentQdd
                    lastErrors[l] = error
                # If not, stop the leg.
                elif i >= len(trajectories[l]) - 1:
                    qCd = [0, 0, 0]
                qCds.append(qCd)
            if not self.motorDriver.syncWriteMotorsVelocitiesInLegs(legsIds, qCds, i == 0):
                return False
            try:
                time.sleep(timeStep - (time.time() - startTime))
            except:
                time.sleep(0)
        self.motorDriver.clearGroupSyncReadParams()
        self.motorDriver.clearGroupSyncWriteParams()
        return True

    # def movePlatform(self, trajectory, velocity, globalStartPose):
    #     """Move spider body as a parallel platform along a given trajectory.

    #     :param trajectory: Spider's body trajectory.
    #     :param velocity: Spider's body velocity.
    #     :return: True if movement was successfull, false otherwise.
    #     """
    #     localLegsPositions = [self.motorDriver.readLegPosition(leg) for leg in range(self.spider.NUMBER_OF_LEGS)]
    #     globalLegsPositions = self.matrixCalculator.getLegsInGlobal(localLegsPositions, globalStartPose)
    #     qD, qDd = self.getQdQddPlatformFF(trajectory, velocity, globalLegsPositions)


        

            


        




