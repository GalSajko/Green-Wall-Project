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

    def getCurrentQdQddLeg(self, legIdx, xD, xDd):
        """Calculate current reference motors positions and velocities.

        :param legIdx: Leg id.
        :param xD: Current reference leg-tip position.
        :param xDd: Current reference leg-tip velocity.
        :return: Reference motors positions and velocities calculated from leg-tip reference.
        """
        currentQd = self.kinematics.legInverseKinematics(legIdx, xD)
        J = self.kinematics.legJacobi(legIdx, currentQd)
        currentQdd = np.dot(np.linalg.inv(J), xDd)

        return np.array(currentQd), np.array(currentQdd)

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

    def moveLeg(self, legIdx, trajectory, velocity):
        """Move leg along given trajectory.

        :param legIdx: Leg id
        :param trajectory: Trajectory.
        :param velocity: Velocity.
        """
        timeStep = trajectory[1][-1] - trajectory[0][-1]
        Kp = 10
        Kd = 1
        lastError = 0
        self.motorDriver.clearGroupSyncReadParams()
        self.motorDriver.clearGroupSyncWriteParams()
        if self.motorDriver.addGroupSyncReadParams(legIdx):
            for idx, pose in enumerate(trajectory):
                startTime = time.time()
                currentQd, currentQdd = self.getCurrentQdQddLeg(legIdx, pose[:3], velocity[idx][:3])
                qA = self.motorDriver.syncReadMotorsPositionsInLeg(legIdx)
                error = currentQd - qA
                dE = (error - lastError) / timeStep
                qCd = (Kp * error + Kd * dE) + currentQdd
                lastError = error

                self.motorDriver.syncWriteMotorsVelocitiesInLeg(legIdx, qCd, idx == 0)
    
                if idx == len(trajectory) - 1:
                    print("ON POSITIONS")
                    qCd = [0, 0, 0]
                    self.motorDriver.syncWriteMotorsVelocitiesInLeg(legIdx, qCd, idx == 0)
                    break
                else:
                    idx += 1
                try:
                    time.sleep(timeStep - (time.time() - startTime))
                except:
                    time.sleep(0)
                

            self.motorDriver.clearGroupSyncReadParams()
            self.motorDriver.clearGroupSyncWriteParams()
            return True
        
        return False
            


        




