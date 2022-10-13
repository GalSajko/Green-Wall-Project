import numpy as np
import time
import threading
import queue
import matplotlib.pyplot as plt

import calculations
import environment as env
import dynamixel as dmx
import planning
import config
import numbafunctions as nf

class VelocityController:
    """ Class for velocity-control of spider's movement. All legs are controlled with same controller, but can be moved separately and independently
    from other legs. Reference positions for each legs are writen in legs-queues. On each control-loop controller takes first values from all of the legs-queues.
    """
    def __init__ (self, isVertical = False):
        self.K_P_FORCE = 0.05

        self.transformations = calculations.TransformationCalculator()
        self.kinematics = calculations.Kinematics()
        self.dynamics = calculations.Dynamics()
        self.mathTools = calculations.MathTools()
        self.spider = env.Spider()
        self.trajectoryPlanner = planning.TrajectoryPlanner()
        self.motorDriver = dmx.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]])
        # self.gripperController = periphery.GripperController()
        # This line will cause 2s long pause, to initialize the sensor.
        # self.bno055 = periphery.BNO055(isVertical)

        self.legsQueues = [queue.Queue() for i in range(self.spider.NUMBER_OF_LEGS)]
        self.sentinel = object()
        self.lastMotorsPositions = np.zeros([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        self.locker = threading.Lock()
        self.killControllerThread = False

        self.period = 1.0 / config.CONTROLLER_FREQUENCY
        self.Kp = np.array([[1.0, 1.0, 1.0]] * self.spider.NUMBER_OF_LEGS) * 12.0
        self.Kd = np.array([[1.0, 1.0, 1.0]] * self.spider.NUMBER_OF_LEGS) * 0.1

        self.qA = []
        self.fAMean = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        self.fD = np.zeros([self.spider.NUMBER_OF_LEGS, 3])

        self.isForceMode = False
        self.forceModeLegsIds = None

        self.__initControllerThread()
        time.sleep(1)

    #region public methods
    def controller(self):
        """Velocity controller to controll all five legs continuously. Runs in separate thread.
        """
        lastQErrors = np.zeros([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        fErrors = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        qDd = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        qD = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
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
                forceModeLegs = self.forceModeLegsIds
                # If controller was just initialized, save current positions and keep legs on these positions until new command is given.
                if init:
                    self.lastMotorsPositions = currentAngles
                    init = False

            qD, qDd = self.__getQdQddFromQueues()

            spiderGravityVector = np.array([0.0, -9.81, 0.0], dtype = np.float32)
            fA, Jhash = self.dynamics.getForcesOnLegsTips(currentAngles, currents, spiderGravityVector)
            self.fAMean, fBuffer, counter = self.mathTools.runningAverage(fBuffer, counter, fA)
            if forceMode:
                # spiderGravityVector = self.bno055.readGravity()
                offsets = self.__forcePositionP(self.fD, self.fAMean)
                fErrors = self.fD - self.fAMean

                for leg in forceModeLegs:
                    # Read leg's current pose in spider's origin.
                    xSpider = self.kinematics.spiderBaseToLegTipForwardKinematics(leg, currentAngles[leg])
                    # Add calculated offset.
                    xSpider[:3][:,3] += offsets[leg]
                    # Transform into leg's origin.
                    xD = np.dot(np.linalg.inv(self.spider.T_ANCHORS[leg]), xSpider)[:3][:,3]

                    qD[leg] = self.kinematics.legInverseKinematics(leg, xD)
                    qDd[leg] = np.dot(Jhash[leg], fErrors[leg] * self.K_P_FORCE)

                    with self.locker:
                        self.lastMotorsPositions[leg] = currentAngles[leg]

            qCds, lastQErrors = self.__positionVelocityPD(qD, currentAngles, qDd, lastQErrors)

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

        qDs, qDds = self.__getQdQddLegFF(legId, positionTrajectory, velocityTrajectory)

        for idx, qD in enumerate(qDs):
            self.legsQueues[legId].put([qD, qDds[idx]])
        self.legsQueues[legId].put(self.sentinel)

        return True
            
    def moveLegsSync(self, legsIds, goalPositionsOrOffsets, origin, duration, trajectoryType, spiderPose = None, isOffset = False):
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
            localGoalPosition = self.__convertIntoLocalGoalPosition(leg, legCurrentPosition, goalPositionsOrOffsets[idx], origin, isOffset, spiderPose)
            
            positionTrajectory, velocityTrajectory = self.__getTrajectory(legCurrentPosition, localGoalPosition, duration, trajectoryType)
            
            qD, qDd = self.__getQdQddLegFF(leg, positionTrajectory, velocityTrajectory)
            qDs.append(qD)
            qDds.append(qDd)
        
        for i in range(len(qDs[0])):
            for idx, leg in enumerate(legsIds):
                self.legsQueues[leg].put([qDs[idx][i], qDds[idx][i]])
        for leg in legsIds:
            self.legsQueues[leg].put(self.sentinel)
        
        return True
   
   # TODO: Change this method to use angles read at the beginning at the control loop.
    # def moveLegAndGripper(self, legId, goalPosition, duration, spiderPose, legsGlobalPositions):
    #     """Open gripper, move leg to the new pin and close the gripper.

    #     Args:
    #         legId (int): Leg id.
    #         goalPosition (list): 1x3 array of global x, y and z position of leg.
    #         duration (float): Duration of movement.
    #         spiderPose (list): 1x4 array of global x, y, z and rotZ pose of spider's body.
    #     """
    #     attachTime = 1
    #     detachTime = 1

    #     with self.locker:
    #         legPosition = self.motorDriver.syncReadMotorsPositionsInLegs([legId], True, 'l')
    #     globalLegPosition = self.transformations.getLegsInGlobal([legId], legPosition, spiderPose)[0]
    #     detachPosition = self.transformations.getLegsApproachPositionsInGlobal([legId], spiderPose, [globalLegPosition], offset = 0.1)

    #     # If leg is attached, open a gripper.
    #     if legId in self.gripperController.getIdsOfAttachedLegs():
    #         self.gripperController.openGrippersAndWait([legId])

    #     # Move leg on detach position.
    #     self.moveLegAsync(legId, detachPosition, 'g', detachTime, 'minJerk', spiderPose)
    #     time.sleep(detachTime + 0.5)

    #     # Measure new spider pose after detaching the leg.
    #     usedLegs = self.spider.LEGS_IDS.copy()
    #     usedLegs.remove(legId)
    #     legsGlobalPositions = np.delete(legsGlobalPositions, legId, 0)
    #     with self.locker:
    #         currentAngles = self.qA
    #     newPose = self.transformations.getSpiderPose(usedLegs, legsGlobalPositions, currentAngles)
        
    #     # Move leg on attach position.
    #     attachPosition = self.transformations.getLegsApproachPositionsInGlobal([legId], newPose, [goalPosition])
    #     self.moveLegAsync(legId, attachPosition, 'g', duration, 'bezier', newPose)
    #     time.sleep(duration + 0.5)

    #     self.moveLegAsync(legId, goalPosition, 'g', attachTime, 'minJerk', newPose)
    #     time.sleep(attachTime + 0.5)

    #     # Close gripper.
    #     self.gripperController.moveGripper(legId, self.gripperController.CLOSE_COMMAND)

    def startForceMode(self, legsIds, desiredForces):
        """Start force controller inside main velocity controller loop.

        Args:
            legsIds (list): Ids of leg, which is to be force-controlled.
            desiredForce (list): nx3 vector of x, y, z values of force, given in spider's origin, where n is number of used legs.
        """
        if len(legsIds) != len(desiredForces):
            raise ValueError("Number of legs used in force controll does not match with number of given desired forces vectors.")
        with self.locker:
            self.isForceMode = True
            self.forceModeLegsIds = legsIds
            self.fD[legsIds] = desiredForces
    
    def stopForceMode(self):
        """Stop force controller inside main velocity controller loop.
        """
        with self.locker:
            self.isForceMode = False

    def endControllerThread(self):
        """Set flag to kill a controller thread.
        """
        self.killControllerThread = True
    #endregion

    #region private methods
    def __initControllerThread(self):
        """Start a thread with controller loop.
        """
        thread = threading.Thread(target = self.controller, name = 'velocity_controller_thread', daemon = False)
        try:
            thread.start()
            print("Controller thread is running.")
        except RuntimeError as re:
            print(re)
    
    def __getQdQddFromQueues(self):
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

    def __getQdQddLegFF(self, legId, xD, xDd):
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
    
    def __positionVelocityPD(self, desiredAngles, currentAngles, ffVelocity, lastErrors):
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

    def __forcePositionP(self, desiredForces, currentForces):
        """Force-position P controller. Calculate position offsets from desired forces.

        Args:
            desiredForces (list): 5x3 array of desired forces in spider's origin.
            currentForces (list): 5x3 array of measured current forces in spider's origin.

        Returns:
            numpy.ndarray: 5x3 array of of position offsets of leg-tips, given in spider's origin.
        """
        fErrors = desiredForces - currentForces
        dXSpider = fErrors * self.K_P_FORCE
        offsets = dXSpider * self.period

        return offsets
    
    def __convertIntoLocalGoalPosition(self, legId, legCurrentPosition, goalPositionOrOffset, origin, isOffset, spiderPose):
        if origin == 'l':
            localGoalPosition = goalPositionOrOffset
            if isOffset:
                localGoalPosition += legCurrentPosition
            return localGoalPosition
        if not isOffset:
            return self.transformations.getLegInLocal(legId, goalPositionOrOffset, spiderPose)
        return np.array(legCurrentPosition + self.transformations.getGlobalDirectionInLocal(legId, spiderPose, goalPositionOrOffset), dtype = np.float32)
    
    def __getTrajectory(self, legCurrentPosition, legGoalPosition, duration, trajectoryType):
        # If movement goes over x = 0, add additional point at (0, 0, dz/2).
        xSigns = [np.sign(legCurrentPosition[0]), np.sign(legGoalPosition[0])]
        legCurrentPosition = np.array(legCurrentPosition, dtype = np.float32)
        legGoalPosition = np.array(legGoalPosition, dtype = np.float32)
        if xSigns[0] != xSigns[1] and 0 not in xSigns:
            interPoint = np.array([0.0, 0.0, (legCurrentPosition[2] + legGoalPosition[2]) / 2.0])
            firstPositionTrajectory, firstVelocityTrajectory = self.trajectoryPlanner.calculateTrajectory(legCurrentPosition, interPoint, duration / 2, trajectoryType)
            secondPositionTrajectory, secondVelocityTrajectory = self.trajectoryPlanner.calculateTrajectory(interPoint, legGoalPosition, duration / 2, trajectoryType)

            return np.append(firstPositionTrajectory, secondPositionTrajectory, axis = 0), np.append(firstVelocityTrajectory, secondVelocityTrajectory, axis = 0)

        return self.trajectoryPlanner.calculateTrajectory(legCurrentPosition, legGoalPosition, duration, trajectoryType)
    #endregion