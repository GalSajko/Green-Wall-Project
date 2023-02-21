import numpy as np
import time
import threading
import queue

import config
from environment import spider
from calculations import mathtools as mathTools
from calculations import transformations as tf
from calculations import kinematics as kin
from calculations import dynamics as dyn
from planning import trajectoryplanner as trajPlanner
from periphery import grippers


class VelocityController:
    """ Class for velocity-control of spider's movement. All legs are controlled with same controller, but can be moved separately and independently
    from other legs. Reference positions for each legs are writen in legs-queues. On each control-loop controller takes first values from all of the legs-queues.
    """
    def __init__ (self):
        self.grippersArduino = grippers.GrippersArduino()

        self.legsQueues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
        self.sentinel = object()

        self.lastLegsPositions = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.lastXErrors = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        self.locker = threading.Lock()

        self.period = 1.0 / config.CONTROLLER_FREQUENCY
        self.Kp = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * config.K_P
        self.Kd = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * config.K_D

        self.isForceMode = False
        self.forceModeLegsIds = None
        self.fD = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        self.isImpedanceMode = False
        self.impedanceLegId = None
        self.impedanceDirection = np.zeros(3, dtype = np.float32)

        time.sleep(1)

    #region public methods
    def jointsVelocityController(self, qA, xA, fA, init):
        # If controller was just initialized, save current positions and keep legs on these positions until new command is given.
        with self.locker:
            if init:
                self.lastLegsPositions = xA
            isForceMode = self.isForceMode
            fD = self.fD
            forceModeLegsIds = self.forceModeLegsIds
            isImpedanceMode = self.isImpedanceMode
            impedanceDirection = self.impedanceDirection
            impedanceLegId = self.impedanceLegId

        xD, xDd, xDdd = self.__getXdXddXdddFromQueues()
        
        if isForceMode:
            legOffsetsInSpider, legVelocitiesInSpider = self.__forcePositionP(fD, fA)
            localOffsets, localVelocities = kin.getXdXddFromOffsets(forceModeLegsIds, legOffsetsInSpider, legVelocitiesInSpider)
            xD[forceModeLegsIds] += localOffsets
            xDd[forceModeLegsIds] = localVelocities
            with self.locker:
                self.lastLegsPositions[forceModeLegsIds] = xA[forceModeLegsIds]
        
        if isImpedanceMode:
            xDd[impedanceLegId] = 0.1 * impedanceDirection * int(np.linalg.norm(fA[impedanceLegId]) < 3.0)
            with self.locker:
                self.lastLegsPositions[impedanceLegId] = xA[impedanceLegId]

        xCd, self.lastXErrors = self.__eePositionVelocityPd(xA, xD, xDd, xDdd)
        qCd = kin.getJointsVelocities(qA, xCd)

        return qCd
    
    def moveLegAsync(self, legId, legCurrentPosition, goalPositionOrOffset, origin, duration, trajectoryType, spiderPose = None, isOffset = False):
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
        if origin not in (config.LEG_ORIGIN, config.GLOBAL_ORIGIN):
            raise ValueError("Unknown origin.")
        if origin == config.GLOBAL_ORIGIN and spiderPose is None:
            raise TypeError("Parameter spiderPose should not be None.")

        self.legsQueues[legId] = queue.Queue()

        localGoalPosition = tf.convertIntoLocalGoalPosition(legId, legCurrentPosition, goalPositionOrOffset, origin, isOffset, spiderPose)
        positionTrajectory, velocityTrajectory, accelerationTrajectory = trajPlanner.getTrajectory(legCurrentPosition, localGoalPosition, duration, trajectoryType)

        for idx, position in enumerate(positionTrajectory):
            self.legsQueues[legId].put([position[:3], velocityTrajectory[idx][:3], accelerationTrajectory[idx][:3]])
        self.legsQueues[legId].put(self.sentinel)

        return True
            
    def moveLegsSync(self, legsIds, legsCurrentPositions, goalPositionsOrOffsets, origin, duration, trajectoryType, spiderPose = None, isOffset = False):
        """Write reference positions and velocities in any number (less than 5) of leg-queues. Legs start to move at the same time. 
        Meant for moving a platform.

        Args:
            legsIds (list): Legs ids.
            legsCurrentPositions (list): nx3x1 array of legs' current positions, where n is number of legs.
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
        if origin not in (config.LEG_ORIGIN, config.GLOBAL_ORIGIN):
            raise ValueError(f"Unknown origin {origin}.")
        if origin == config.GLOBAL_ORIGIN and spiderPose is None:
            raise TypeError("If origin is global, spider pose should be given.")
        if len(legsIds) != len(goalPositionsOrOffsets):
            raise ValueError("Number of legs and given goal positions should be the same.")
        
        # Stop all of the given legs.
        for leg in legsIds:
            self.legsQueues[leg] = queue.Queue()

        xDs = np.zeros((len(legsIds), int(duration / self.period), 3), dtype = np.float32)
        xDds = np.zeros((len(legsIds), int(duration / self.period), 3), dtype = np.float32)
        xDdds = np.zeros((len(legsIds), int(duration / self.period), 3) , dtype = np.float32)

        for idx, leg in enumerate(legsIds):          
            localGoalPosition = tf.convertIntoLocalGoalPosition(leg, legsCurrentPositions[leg], goalPositionsOrOffsets[idx], origin, isOffset, spiderPose) 
            positionTrajectory, velocityTrajectory, accelerationTrajectory = trajPlanner.getTrajectory(legsCurrentPositions[leg], localGoalPosition, duration, trajectoryType)

            xDs[idx] = positionTrajectory[:, :3]
            xDds[idx] = velocityTrajectory[:, :3]
            xDdds[idx] = accelerationTrajectory[:, :3]
        
        for i in range(len(xDs[0])):
            for idx, leg in enumerate(legsIds):
                self.legsQueues[leg].put([xDs[idx][i], xDds[idx][i], xDdds[idx][i]])

        for leg in legsIds:
            self.legsQueues[leg].put(self.sentinel)
        
        return True

    def clearInstructionQueues(self):
        """Clear instruction queues for all legs.
        """
        self.legsQueues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
    
    def updateLastLegsPositions(self, xA):
        """Update last legs positions.

        Args:
            xA (list): 5x3 array of legs' positions in leg-local origins.
        """
        with self.locker:
            self.lastLegsPositions = xA

    def startForceMode(self, legsIds, desiredForces):
        """Start force mode inside main velocity controller loop.

        Args:
            legsIds (list): Ids of leg, which is to be force-controlled.
            desiredForce (list): 5x3 vector of x, y, z values of force, given in spider's origin, where n is number of used legs.
        """
        if len(legsIds) != len(desiredForces):
            raise ValueError("Number of legs used in force controll does not match number of given desired forces vectors.")
        with self.locker:
            self.isForceMode = True
            self.forceModeLegsIds = legsIds
            self.fD[legsIds] = desiredForces
    
    def stopForceMode(self):
        """Stop force mode inside main velocity self loop.
        """
        with self.locker:
            self.isForceMode = False
    
    def startImpedanceMode(self, legId, velocityDirection):
        """Start impedance mode.

        Args:
            legId (int): Id of leg.
            velocityDirection (list): 1x3 array of desired velocity direction, given in leg's origin.
        """
        with self.locker:
            self.isImpedanceMode = True
            self.impedanceDirection = velocityDirection
            self.impedanceLegId = legId
    
    def stopImpedanceMode(self):
        with self.locker:
            self.isImpedanceMode = False

    #endregion

    #region private methods
    def __getXdXddXdddFromQueues(self):
        """Read current desired position, velocity and acceleration from queues for each leg. If leg-queue is empty, keep leg on latest position.

        Returns:
            tuple: Three 5x3 arrays of current desired positions, velocities and accelerations of legs.
        """
        xD = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        xDd = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        xDdd = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        for leg in spider.LEGS_IDS:
            try:
                queueData = self.legsQueues[leg].get(False)
                if queueData is not self.sentinel:
                    with self.locker:
                        self.lastLegsPositions[leg] = queueData[0]
                if queueData is self.sentinel:
                    with self.locker:
                        xD[leg] = np.copy(self.lastLegsPositions[leg])
                    xDd[leg] = np.zeros(3, dtype = np.float32)
                    xDdd[leg] = np.zeros(3, dtype = np.float32)
                else:
                    xD[leg] = queueData[0]
                    xDd[leg] = queueData[1]
                    xDdd[leg] = queueData[2]
            except queue.Empty:
                with self.locker:
                    xD[leg] = np.copy(self.lastLegsPositions[leg])
                xDd[leg] = np.zeros(3, dtype = np.float32)
                xDdd[leg] = np.zeros(3, dtype = np.float32)

        return xD, xDd, xDdd

    def __eePositionVelocityPd(self, xA, xD, xDd, xDdd):
        """PD controller. Feed-forward velocity is used only in force mode, otherwise its values are zeros.

        Args:
            xD (numpy.ndarray): 5x3 array of desired legs' positions.
            xA (numpy.ndarray): 5x3 array of actual legs' positions.
            xDd (numpy.ndarray): 5x3 array of feed-forward velocities.
            xDdd (numpy.ndarray): 5x3 array of feed-forward accelerations.
            lastErrors (numpy.ndarray): 5x3 array of position errors from previous loop.

        Returns:
            tuple: Two 5x3 arrays of commanded legs velocities and current position errors.
        """
        xErrors = np.array(xD - xA)
        dXe = (xErrors - self.lastXErrors) / self.period
        xCd = np.array(self.Kp * xErrors + self.Kd * dXe + xDd + config.K_ACC * xDdd, dtype = np.float32)

        return xCd, xErrors

    def __forcePositionP(self, desiredForces, currentForces):
        """Force-position P controller. Calculate position offsets and desired legs velocities in spider's origin from desired forces.

        Args:
            desiredForces (list): 5x3 array of desired forces in spider's origin.
            currentForces (list): 5x3 array of measured current forces in spider's origin.

        Returns:
            tuple: Two 5x3 array of of position offsets of leg-tips and leg-tips velocities, given in spider's origin.
        """
        fErrors = desiredForces - currentForces
        dXSpider = fErrors * config.K_P_FORCE
        offsets = dXSpider * self.period

        return offsets, dXSpider
    #endregion