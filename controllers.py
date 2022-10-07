import numpy as np
import time
import serial
import threading
import queue
import os
import inspect
import multiprocessing
import matplotlib.pyplot as plt
import math

import calculations
import environment as env
import dynamixel as dmx
import planning
import config
import mappers
import periphery

class VelocityController:
    """ Class for velocity-control of spider's movement. All legs are controlled with same controller, but can be moved separately and independently
    from other legs. Reference positions for each legs are writen in legs-queues. On each control-loop controller takes first values from all of the legs-queues.
    """
    def __init__ (self, isVertical = False):
        self.transformations = calculations.TransformationCalculator()
        self.kinematics = calculations.Kinematics()
        self.dynamics = calculations.Dynamics()
        self.mathTools = calculations.MathTools()
        self.spider = env.Spider()
        self.trajectoryPlanner = planning.TrajectoryPlanner()
        self.motorDriver = dmx.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]])
        self.gripperController = periphery.GripperController()
        # This line will cause 2s long pause, to initialize the sensor.
        self.bno055 = periphery.BNO055(isVertical)

        self.legsQueues = [queue.Queue() for i in range(self.spider.NUMBER_OF_LEGS)]
        self.sentinel = object()
        self.lastMotorsPositions = np.zeros([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        self.locker = threading.Lock()
        self.killControllerThread = False

        self.Kp = np.array([[1.0, 1.0, 1.0]] * self.spider.NUMBER_OF_LEGS) * 18.0
        self.Kd = np.array([[1.0, 1.0, 1.0]] * self.spider.NUMBER_OF_LEGS) * 0.25
        self.period = 1.0 / config.CONTROLLER_FREQUENCY

        self.qA = []
        self.fA = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        self.fD = np.array([0.0, 0.0, 0.0])

        self.isForceMode = False
        self.forceModeLegId = None

        self.initControllerThread()

    def controller(self):
        """Velocity controller to controll all five legs continuously. Runs in separate thread.
        """
        lastQErrors = np.zeros([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        lastFErrors = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        qDd = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        qD = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        qDdF = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        qDf = np.zeros([self.spider.NUMBER_OF_LEGS , 3])
        init = True

        fBuffer = np.zeros([10, 5, 3])
        counter = 0

        while True:
            if self.killControllerThread:
                break

            startTime = time.perf_counter()

            # Get current data.
            with self.locker: 
                currentAngles, currents = self.motorDriver.syncReadAnglesAndCurrentsWrapper()
                self.qA = currentAngles
                forceMode = self.isForceMode
                forceModeLeg = self.forceModeLegId
                # If controller was just initialized, save current positions and keep legs on these positions until new command is given.
                if init:
                    self.lastMotorsPositions = currentAngles
                    init = False

            qD, qDd = self.getQdQddFromQueues()

            if forceMode:
                spiderGravityVector = self.bno055.readGravity()

                # Read leg's current pose in spider's origin.
                xSpider = self.kinematics.spiderBaseToLegTipForwardKinematics(forceModeLeg, currentAngles[forceModeLeg])
                xD = np.dot(np.linalg.inv(self.spider.T_ANCHORS[forceModeLeg]), xSpider)[:3][:,3]

                self.fA = self.dynamics.getForcesOnLegsTips(currentAngles, currents, spiderGravityVector)
                # Running average of measured forces.
                fAMean, fBuffer, counter = self.mathTools.runningAverage(fBuffer, counter, self.fA)
                dXSpider, offsets = self.forcePositionP(self.fD, fAMean, xD)
            
                # Add calculated offset.
                xSpider[:3][:,3] += offsets[forceModeLeg]
                # Transform into leg's origin.
                xD = np.dot(np.linalg.inv(self.spider.T_ANCHORS[forceModeLeg]), xSpider)[:3][:,3]

                # Calculate values in joints space and avoid reaching out of the working space. 
                if np.linalg.norm(xD) >= self.spider.LEG_LENGTH_LIMIT:
                    xSpider[:3][:,3] -= offsets[forceModeLeg]
                    xD = np.dot(np.linalg.inv(self.spider.T_ANCHORS[forceModeLeg]), xSpider)[:3][:,3]
                    dXSpider[forceModeLeg] = np.zeros(3)
                    
                qD[forceModeLeg] = self.kinematics.legInverseKinematics(forceModeLeg, xD)
                # qDd[forceModeLeg] = np.dot(np.linalg.inv(self.kinematics.spiderBaseToLegTipJacobi(forceModeLeg, currentAngles[forceModeLeg])), dXSpider[forceModeLeg])

                J = self.kinematics.spiderBaseToLegTipJacobi(forceModeLeg, currentAngles[forceModeLeg])
                alpha = np.ones([3, 3]) * 0.0
                dampedPseudoInverse = self.mathTools.dampedPseudoInverse(J, alpha)
                torques = self.dynamics.getTorques(currentAngles, currents, spiderGravityVector)
                forceFactor = (self.fD - np.dot(np.transpose(dampedPseudoInverse), torques[forceModeLeg])) * 0.05
                qDd[forceModeLeg] = np.dot(dampedPseudoInverse, forceFactor)
                
                with self.locker:
                    self.lastMotorsPositions[forceModeLeg] = currentAngles[forceModeLeg]

            qCds, lastQErrors = self.positionVelocityPD(qD, currentAngles, qDd, lastQErrors)

            # Limit joints velocities, before sending them to motors.
            if forceMode:
                qCds[qCds > 1.0] = 1.0
                qCds[qCds < -1.0] = -1.0

            # Send new commands to motors.
            with self.locker:
                self.motorDriver.syncWriteMotorsVelocitiesInLegs(self.spider.LEGS_IDS, qCds)

            elapsedTime = time.perf_counter() - startTime
            while elapsedTime < self.period:
                elapsedTime = time.perf_counter() - startTime
                time.sleep(0)
    
    def positionVelocityPD(self, desiredAngles, currentAngles, ffVelocity, lastErrors):
        """Position-velocity PD controller. Calculate commanded velocities from position input and add feed-forward calculated velocities.

        Args:
            desiredAngles (list): 5x3 array of desired angles in joints.
            currentAngles (list): 5x3 array of measured current angles in joints.
            ffVelocity (list): 5x3 array of feed forward calculated joints velocities.
            lastErrors (list): 5x3 array of last position errors.

        Returns:
            tuple: 5x3 numpy.ndarray of commanded joints velocities and updated errors.
        """
        qErrors = np.array(desiredAngles - currentAngles)
        dQe = (qErrors - lastErrors) / self.period
        qCds = self.Kp * qErrors + self.Kd * dQe + ffVelocity
        lastErrors = qErrors

        return qCds, lastErrors

    def forcePositionP(self, desiredForces, currentForces, xD):
        """Force-position P controller. Calculate position offsets from desired forces.

        Args:
            desiredForces (list): 5x3 array of desired forces in spider's origin.
            currentForces (list): 5x3 array of measured current forces in spider's origin.
            xD (list): 1x3 array of leg's position in leg's origin.

        Returns:
            tuple: 5x3 array of leg-tips velocities and 5x3 array of position offsets of leg-tips, both given in spider's origin.
        """
        fErrors = desiredForces - currentForces
        Kp = self.__calculateKpForce(xD)
        dXSpider = fErrors * Kp
        offsets = dXSpider * self.period

        return dXSpider, offsets

    def initControllerThread(self):
        """Start a thread with controller loop.
        """
        thread = threading.Thread(target = self.controller, name = 'velocity_controller_thread', daemon = False)
        try:
            thread.start()
            print("Controller thread is running.")
        except RuntimeError as re:
            print(re)

    def endControllerThread(self):
        """Set flag to kill a controller thread.
        """
        self.killControllerThread = True

    def getQdQddFromQueues(self):
        """Read current qD and qDd from queues for each leg. If leg-queue is empty, keep leg on latest position.

        Returns:
            tuple: Two 5x3 numpy.ndarrays of current qDs and qDds.
        """
        qD = np.empty([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG], dtype = np.float32)
        qDd = np.empty([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG], dtype = np.float32)

        for leg in self.spider.LEGS_IDS:
            try:
                queueData = self.legsQueues[leg].get(False)
                with self.locker:
                    self.lastMotorsPositions[leg] = self.qA[leg]
                if queueData is self.sentinel:
                    qD[leg] = self.lastMotorsPositions[leg]
                    qDd[leg] = ([0, 0, 0])
                else:
                    qD[leg] = queueData[0]
                    qDd[leg] = queueData[1]
            except queue.Empty:
                with self.locker:
                    qD[leg] = self.lastMotorsPositions[leg]
                qDd[leg] = ([0, 0, 0])

        return qD, qDd

    def moveLegAsync(self, legId, goalPositionOrOffset, origin, duration, trajectoryType, spiderPose = None, isOffset = False):
        """Write reference positions and velocities into leg-queue.

        Args:
            legId (int): Leg id.
            goalPositionOrOffset (list): 1x3 array of  desired x, y, and z goal positons or offsets.
            origin (str): Origin that goal position is given in. Wheter 'l' for leg-local or 'g' for global.
            duration (float): Desired movement duration.
            trajectoryType (str): Type of movement trajectory (bezier or minJerk).
            spiderPose (list, optional): Spider pose in global origin, used if goalPositionOrOffset is given in global origin. Defaults to None.
            offset(bool, optional): If true, move leg relatively on current position, goalPositionOrOffset should be given as desired offset. Defaults to False.

        Raises:
            ValueError: If origin is unknown.
            TypeError: If origin is global and spiderPose is None.

        Returns:
            bool: False if ValueError is catched during trajectory calculation, True otherwise.
        """
        if origin not in ('l', 'g'):
            raise ValueError("Unknown origin.")
        if origin == 'g' and spiderPose is None:
            raise TypeError("Parameter spiderPose should not be None.")

        self.legsQueues[legId] = queue.Queue()
        self.legsQueues[legId].put(self.sentinel)

        with self.locker:
            currentAngles = self.qA[legId]

        legCurrentPosition = self.kinematics.legForwardKinematics(legId, currentAngles)[:,3][:3]
        localGoalPosition = self.__convertIntoLocalGoalPosition(legId, legCurrentPosition, goalPositionOrOffset, origin, isOffset, spiderPose)

        positionTrajectory, velocityTrajectory = self.__getTrajectory(legCurrentPosition, localGoalPosition, duration, trajectoryType)

        qDs, qDds = self.getQdQddLegFF(legId, positionTrajectory, velocityTrajectory)

        for idx, qD in enumerate(qDs):
            self.legsQueues[legId].put([qD, qDds[idx]])
        self.legsQueues[legId].put(self.sentinel)

        return True
            
    def moveLegsSync(self, legsIds, goalPositionsOrOffsets, origin, duration, trajectoryType, spiderPose = None, offset = False):
        """Write reference positions and velocities in any number (within 5) of leg-queues. Legs start to move at the same time. 
        Meant for moving a platform.

        Args:
            legsIds (list): Legs ids.
            goalPositionsOrOffsets (list): nx3x1 array of goal positions, where n is number of legs.
            origin (str): Origin that goal positions are given in, 'g' for global or 'l' for local.
            duration (float): Desired duration of movements.
            trajectoryType (str): Type of movement trajectory (bezier or minJerk).
            spiderPose (list, optional): Spider pose in global origin, used if goalPositionsOrOffsets are given in global. Defaults to None.
            offset (bool, optional): If true, move legs relatively to current positions, goalPositionsOrOffsets should be given as desired offsets in global origin. Defaults to False.

        Raises:
            ValueError: If origin is unknown.
            TypeError: If origin is global and spider pose is not given.
            ValueError: If number of used legs and given goal positions are not the same.

        Returns:
            bool: False if ValueError is catched during trajectory calculations, True otherwise.
        """
        if origin not in ('l', 'g'):
            raise ValueError(f"Unknown origin {origin}.")
        if origin == 'g' and spiderPose is None:
            raise TypeError("If origin is global, spider pose should be given.")  
        if len(legsIds) != len(goalPositionsOrOffsets):
            raise ValueError("Number of legs and given goal positions should be the same.")
        
        # Stop all of the given legs.
        for leg in legsIds:
            self.legsQueues[leg] = queue.Queue()
            self.legsQueues[leg].put(self.sentinel)
        
        qDs = []
        qDds = []

        for idx, leg in enumerate(legsIds):
            with self.locker:
                currentAngles = self.qA[leg]
                
            # Get current leg position in local.
            legCurrentPosition = self.kinematics.legForwardKinematics(leg, currentAngles)[:,3][:3]            
            localGoalPosition = self.__convertIntoLocalGoalPosition(leg, legCurrentPosition, goalPositionsOrOffsets[idx], origin, spiderPose)
            
            positionTrajectory, velocityTrajectory = selg.__getTrajectory(legCurrentPosition, localGoalPosition, duration, trajectoryType)
            
            qD, qDd = self.getQdQddLegFF(leg, positionTrajectory, velocityTrajectory)
            qDs.append(qD)
            qDds.append(qDd)
        
        for i in range(len(qDs[0])):
            for idx, leg in enumerate(legsIds):
                self.legsQueues[leg].put([qDs[idx][i], qDds[idx][i]])
        for leg in legsIds:
            self.legsQueues[leg].put(self.sentinel)
        
        return True
   
    def moveLegAndGripper(self, legId, goalPosition, duration, spiderPose, legsGlobalPositions):
        """Open gripper, move leg to the new pin and close the gripper.

        Args:
            legId (int): Leg id.
            goalPosition (list): 1x3 array of global x, y and z position of leg.
            duration (float): Duration of movement.
            spiderPose (list): 1x4 array of global x, y, z and rotZ pose of spider's body.
        """
        attachTime = 1
        detachTime = 1

        with self.locker:
            legPosition = self.motorDriver.syncReadMotorsPositionsInLegs([legId], True, 'l')
        globalLegPosition = self.transformations.getLegsInGlobal([legId], legPosition, spiderPose)[0]
        detachPosition = self.transformations.getLegsApproachPositionsInGlobal([legId], spiderPose, [globalLegPosition], offset = 0.1)

        # If leg is attached, open a gripper.
        if legId in self.gripperController.getIdsOfAttachedLegs():
            self.gripperController.openGrippersAndWait([legId])

        # Move leg on detach position.
        self.moveLegAsync(legId, detachPosition, 'g', detachTime, 'minJerk', spiderPose)
        time.sleep(detachTime + 0.5)

        # Measure new spider pose after detaching the leg.
        usedLegs = self.spider.LEGS_IDS.copy()
        usedLegs.remove(legId)
        legsGlobalPositions = np.delete(legsGlobalPositions, legId, 0)
        with self.locker:
            currentAngles = self.qA
        newPose = self.transformations.getSpiderPose(usedLegs, legsGlobalPositions, currentAngles)
        
        # Move leg on attach position.
        attachPosition = self.transformations.getLegsApproachPositionsInGlobal([legId], newPose, [goalPosition])
        self.moveLegAsync(legId, attachPosition, 'g', duration, 'bezier', newPose)
        time.sleep(duration + 0.5)

        self.moveLegAsync(legId, goalPosition, 'g', attachTime, 'minJerk', newPose)
        time.sleep(attachTime + 0.5)

        # Close gripper.
        self.gripperController.moveGripper(legId, self.gripperController.CLOSE_COMMAND)

    def getQdQddLegFF(self, legId, xD, xDd):
        """(Feed-forward) calculate and write reference joints positions and velocities for single leg movement into leg-queue.

        Args:
            legId (int): Leg id.
            xD (list): nx7 array of position (6 values for xyzrpy) trajectory and time stamps (7th value in a row), where n is number of steps in trajectory.
            xDd (list): nx6 array for xyzrpy values of velocity trajectory, where n is number of steps in trajectory.

        Returns:
            tuple: Two nx3 numpy.ndarrays of reference joints positions and velocities, where n is number of steps in trajectory.
        """

        qDs = np.zeros([len(xD), self.spider.NUMBER_OF_MOTORS_IN_LEG])
        qDds = np.zeros([len(xD), self.spider.NUMBER_OF_MOTORS_IN_LEG])
        for idx, pose in enumerate(xD):
            qDs[idx] = self.kinematics.legInverseKinematics(legId, pose[:3])
            J = self.kinematics.legJacobi(legId, qDs[idx])
            qDds[idx] = np.dot(np.linalg.inv(J), xDd[idx][:3])

        return qDs, qDds

    def startForceMode(self, legId):
        """Start force controller inside main velocity controller loop.

        Args:
            legId (int): Id of leg, which is to be force-controlled.
        """
        with self.locker:
            self.isForceMode = True
            self.forceModeLegId = legId
    
    def stopForceMode(self):
        """Stop force controller inside main velocity controller loop.
        """
        with self.locker:
            self.isForceMode = False

    def __calculateKpForce(self, xD):
        r = np.linalg.norm(xD[0:2])
        limit = 0.1
        a = 42.8571
        b = 0.5714
        c = -0.0357
        d = 0.005
        # if r >= limit:
        #     return 0.05
        # return a * math.pow(r, 3) + b * math.pow(r, 2) + c * r + d
        return 0.03
    
    def __convertIntoLocalGoalPosition(self, legId, legCurrentPosition, goalPositionOrOffset, origin, isOffset, spiderPose):
        if origin == 'l':
            localGoalPosition = goalPositionOrOffset
            if isOffset:
                localGoalPosition += legCurrentPosition
            return localGoalPosition
        if not isOffset:
            return self.transformations.getLegInLocal(legId, goalPositionOrOffset, spiderPose)
        return legCurrentPosition + self.transformations.getGlobalDirectionInLocal(legId, spiderPose, goalPositionsOrOffsets)
    
    def __getTrajectory(self, legCurrentPosition, legGoalPosition, duration, trajectoryType):
        # If movement goes over x = 0, add additional point at (0, 0, dz/2).
        xSigns = [np.sign(legCurrentPosition[0]), np.sign(legGoalPosition[0])]

        if xSigns[0] != xSigns[1] and 0 not in xSigns:
            interPoint = np.array([0.0, 0.0, (legCurrentPosition[2] + legGoalPosition[2]) / 2.0])
            firstPositionTrajectory, firstVelocityTrajectory = self.trajectoryPlanner.calculateTrajectory(legCurrentPosition, interPoint, duration / 2, trajectoryType)
            secondPositionTrajectory, secondVelocityTrajectory = self.trajectoryPlanner.calculateTrajectory(interPoint, legGoalPosition, duration / 2, trajectoryType)

            return np.append(firstPositionTrajectory, secondPositionTrajectory, axis = 0), np.append(firstVelocityTrajectory, secondVelocityTrajectory, axis = 0)

        return self.trajectoryPlanner.calculateTrajectory(legCurrentPosition, legGoalPosition, duration, trajectoryType)
