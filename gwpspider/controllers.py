import numpy as np
import time
import threading
import queue

import config
import spider
from calculations import transformations as tf
from calculations import kinematics as kin
from planning import trajectoryplanner as tp
from periphery import arduinocomm


class VelocityController:
    """ Class for velocity-control of spider's movement. All legs are controlled with same controller, but can be moved separately and independently
    from other legs. Reference positions for each legs are writen in legs-queues. On each control-loop controller takes first values from all of the legs-queues.
    """
    def __init__ (self):
        self.grippers_arduino = arduinocomm.GrippersArduino()

        self.legs_queues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
        self.sentinel = object()

        self.last_legs_positions = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.last_legs_position_errors = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        self.locker = threading.Lock()

        self.k_p = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * config.K_P
        self.k_d = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * config.K_D

        self.is_force_mode = False
        self.force_mode_legs_ids = None
        self.f_d = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        self.is_velocity_mode = False
        self.velocity_mode_legs_ids = None
        self.velocity_mode_direction = np.zeros(3, dtype = np.float32)

        time.sleep(1)
    
    @property
    def MAX_ALLOWED_FORCE(self):
        return 4.0
    
    @property
    def VELOCITY_AMP_FACTOR(self):
        return 0.1
    
    @property
    def PERIOD(self):
        return 1.0 / config.CONTROLLER_FREQUENCY


    #region public methods
    def joints_velocity_controller(self, q_a, x_a, f_a, do_init):
        """Control of joints velocities with implemented position, force and velocity modes.

        Args:
            q_a (list): 5x3 array of current angles in joints.
            x_a (list): 5x3 array of current legs positions, given in local origins.
            f_a (list): 5x3 array of current forces on leg tips, given in spider origin.
            do_init (bool): If True set values of last legs positions to current legs positions. Should be True only at the start.

        Returns:
            numpy.ndarray: 5x3 array of commanded joints velocities.
        """
        # If controller was just initialized, save current positions and keep legs on these positions until new command is given.
        with self.locker:
            if do_init:
                self.last_legs_positions = x_a
            is_force_mode = self.is_force_mode
            f_d = self.f_d
            force_mode_legs_ids = np.array(self.force_mode_legs_ids)
            is_velocity_mode = self.is_velocity_mode
            velocity_mode_direction = self.velocity_mode_direction
            velocity_mode_legs_ids = self.velocity_mode_legs_ids

        x_d, dx_d, ddx_d = self.__get_pos_vel_acc_from_queues()
        
        if is_force_mode:
            legs_position_offsets_in_spider, legs_velocities_in_spider = self.__force_position_p_controller(f_d, f_a)
            legs_position_offsets_in_local, legs_velocities_in_local = kin.get_xd_xdd_from_offsets(force_mode_legs_ids, legs_position_offsets_in_spider, legs_velocities_in_spider)

            new_x_d = x_d[force_mode_legs_ids] + legs_position_offsets_in_local
            new_x_d_norms = np.linalg.norm(new_x_d, axis = 1)
            allowed_legs_ids = np.where(new_x_d_norms < spider.LEG_LENGTH_MAX_LIMIT)
            on_limit_legs_ids = np.where(new_x_d_norms >= spider.LEG_LENGTH_MAX_LIMIT)

            x_d[force_mode_legs_ids[allowed_legs_ids]] = new_x_d[allowed_legs_ids]
            dx_d[force_mode_legs_ids[allowed_legs_ids]] = legs_velocities_in_local[allowed_legs_ids]
            x_d[force_mode_legs_ids[on_limit_legs_ids]] = x_a[force_mode_legs_ids[on_limit_legs_ids]]

            with self.locker:
                self.last_legs_positions[force_mode_legs_ids] = x_a[force_mode_legs_ids]
        
        if is_velocity_mode:
            dx_d[velocity_mode_legs_ids] = self.VELOCITY_AMP_FACTOR * velocity_mode_direction * int(np.linalg.norm(f_a[velocity_mode_legs_ids]) < self.MAX_ALLOWED_FORCE)
            with self.locker:
                self.last_legs_positions[velocity_mode_legs_ids] = x_a[velocity_mode_legs_ids] + dx_d[velocity_mode_legs_ids] * self.PERIOD

        dx_c, self.last_legs_position_errors = self.__ee_position_velocity_pd_controlloer(x_a, x_d, dx_d, ddx_d)

        dq_c = kin.get_joints_velocities(q_a, dx_c)

        if is_force_mode:
            dq_c[dq_c > 1.0] = 1.0
            dq_c[dq_c < -1.0] = -1.0

        return dq_c
    
    def move_leg_async(self, leg_id, leg_current_position, leg_goal_position_or_offset, origin, duration, trajectory_type, spider_pose = None, is_offset = False):
        """Write reference positions and velocities into leg-queue.

        Args:
            leg_id (int): Leg id.
            leg_goal_position_or_offset (list): 1x3 array of  desired x, y, and z goal positons or offsets.
            origin (str): Origin that goal position is given in. Wheter 'l' for leg-local or 'g' for global.
            duration (float): Desired movement duration.
            trajectory_type (str): Type of movement trajectory (bezier or minJerk).
            spider_pose (list, optional): Spider pose in global origin, used if leg_goal_position_or_offset is given in global origin. Defaults to None.
            offset(bool, optional): If true, move leg relatively on current position, leg_goal_position_or_offset should be given as desired offset. Defaults to False.

        Raises:
            ValueError: If origin is unknown.
            TypeError: If origin is global and spider_pose is None.

        Returns:
            bool: False if ValueError is catched during trajectory calculation, True otherwise.
        """
        if origin not in (config.LEG_ORIGIN, config.GLOBAL_ORIGIN):
            raise ValueError("Unknown origin.")
        if origin == config.GLOBAL_ORIGIN and spider_pose is None:
            raise TypeError("Parameter spider_pose should not be None.")

        self.legs_queues[leg_id] = queue.Queue()

        leg_goal_position_in_local = tf.convert_in_local_goal_positions(leg_id, leg_current_position, leg_goal_position_or_offset, origin, is_offset, spider_pose)
        position_trajectory, velocity_trajectory, acceleration_trajectory = tp.get_trajectory(leg_current_position, leg_goal_position_in_local, duration, trajectory_type)

        for idx, position in enumerate(position_trajectory):
            self.legs_queues[leg_id].put([position[:3], velocity_trajectory[idx][:3], acceleration_trajectory[idx][:3]])
        self.legs_queues[leg_id].put(self.sentinel)

        return leg_goal_position_in_local
            
    def move_legs_sync(self, legs_ids, legs_current_positions, legs_goal_positions_or_offsets, origin, duration, trajectory_type, spider_pose = None, is_offset = False):
        """Write reference positions and velocities in any number (less than 5) of leg-queues. Legs start to move at the same time. 
        Meant for moving a platform.

        Args:
            legs_ids (list): Legs ids.
            legs_current_positions (list): nx3x1 array of legs' current positions, where n is number of legs.
            legs_goal_positions_or_offsets (list): nx3x1 array of goal positions, where n is number of legs.
            origin (str): Origin that goal positions are given in, 'g' for global or 'l' for local.
            duration (float): Desired duration of movements.
            trajectory_type (str): Type of movement trajectory (bezier or minJerk).
            spider_pose (list, optional): Spider pose in global origin, used if legs_goal_positions_or_offsets are given in global. Defaults to None.
            offset (bool, optional): If true, move legs relatively to current positions, legs_goal_positions_or_offsets should be given as desired offsets in global origin. Defaults to False.

        Raises:
            ValueError: If origin is unknown.
            TypeError: If origin is global and spider pose is not given.
            ValueError: If number of used legs and given goal positions are not the same.

        Returns:
            bool: False if ValueError is catched during trajectory calculations, True otherwise.
        """
        if origin not in (config.LEG_ORIGIN, config.GLOBAL_ORIGIN):
            raise ValueError(f"Unknown origin {origin}.")
        if origin == config.GLOBAL_ORIGIN and spider_pose is None:
            raise TypeError("If origin is global, spider pose should be given.")
        if len(legs_ids) != len(legs_goal_positions_or_offsets):
            raise ValueError("Number of legs and given goal positions should be the same.")
        
        # Stop all of the given legs.
        for leg in legs_ids:
            self.legs_queues[leg] = queue.Queue()

        x_d = np.zeros((len(legs_ids), int(duration / self.PERIOD), 3), dtype = np.float32)
        dx_d = np.zeros((len(legs_ids), int(duration / self.PERIOD), 3), dtype = np.float32)
        ddx_d = np.zeros((len(legs_ids), int(duration / self.PERIOD), 3) , dtype = np.float32)

        for idx, leg in enumerate(legs_ids):          
            leg_goal_position_in_local = tf.convert_in_local_goal_positions(leg, legs_current_positions[leg], legs_goal_positions_or_offsets[idx], origin, is_offset, spider_pose) 
            position_trajectory, velocity_trajectory, acceleration_trajectory = tp.get_trajectory(legs_current_positions[leg], leg_goal_position_in_local, duration, trajectory_type)

            x_d[idx] = position_trajectory[:, :3]
            dx_d[idx] = velocity_trajectory[:, :3]
            ddx_d[idx] = acceleration_trajectory[:, :3]
        
        for i in range(len(x_d[0])):
            for idx, leg in enumerate(legs_ids):
                self.legs_queues[leg].put([x_d[idx][i], dx_d[idx][i], ddx_d[idx][i]])

        for leg in legs_ids:
            self.legs_queues[leg].put(self.sentinel)
        
        return True

    def clear_instruction_queues(self):
        """Clear instruction queues for all legs.
        """
        self.legs_queues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
    
    def update_last_legs_positions(self, x_a):
        """Update last legs positions.

        Args:
            x_a (list): 5x3 array of legs' positions in leg-local origins.
        """
        with self.locker:
            self.last_legs_positions = x_a

    def start_force_mode(self, legs_ids, f_d):
        """Start force mode inside main velocity controller loop.

        Args:
            legs_ids (list): Ids of leg, which is to be force-controlled.
            desiredForce (list): 5x3 vector of x, y, z values of force, given in spider's origin, where n is number of used legs.
        """
        if len(legs_ids) != len(f_d):
            raise ValueError("Number of legs used in force controll does not match number of given desired forces vectors.")
        with self.locker:
            self.is_force_mode = True
            self.force_mode_legs_ids = legs_ids
            self.f_d[legs_ids] = f_d
    
    def stop_force_mode(self):
        """Stop force mode inside main velocity self loop.
        """
        with self.locker:
            self.is_force_mode = False
    
    def start_velocity_mode(self, leg_id, velocity_direction):
        """Start velocity mode.

        Args:
            leg_id (int): Id of leg.
            velocity_direction (list): 1x3 array of desired velocity direction, given in leg's origin.
        """
        with self.locker:
            self.is_velocity_mode = True
            self.velocity_mode_direction = velocity_direction
            self.velocity_mode_legs_ids = leg_id
    
    def stop_velocity_mode(self):
        """Stop velocity mode.
        """
        with self.locker:
            self.is_velocity_mode = False
    #endregion

    #region private methods
    def __get_pos_vel_acc_from_queues(self):
        """Read current desired position, velocity and acceleration from queues for each leg. If leg-queue is empty, keep leg on latest position.

        Returns:
            tuple: Three 5x3 arrays of current desired positions, velocities and accelerations of legs.
        """
        x_d = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        dx_d = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        ddx_d = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        for leg in spider.LEGS_IDS:
            try:
                queue_data = self.legs_queues[leg].get(False)
                if queue_data is not self.sentinel:
                    with self.locker:
                        self.last_legs_positions[leg] = queue_data[0]
                if queue_data is self.sentinel:
                    with self.locker:
                        x_d[leg] = np.copy(self.last_legs_positions[leg])
                    dx_d[leg] = np.zeros(3, dtype = np.float32)
                    ddx_d[leg] = np.zeros(3, dtype = np.float32)
                else:
                    x_d[leg] = queue_data[0]
                    dx_d[leg] = queue_data[1]
                    ddx_d[leg] = queue_data[2]
            except queue.Empty:
                with self.locker:
                    x_d[leg] = np.copy(self.last_legs_positions[leg])
                dx_d[leg] = np.zeros(3, dtype = np.float32)
                ddx_d[leg] = np.zeros(3, dtype = np.float32)

        return x_d, dx_d, ddx_d

    def __ee_position_velocity_pd_controlloer(self, x_a, x_d, dx_d, ddx_d):
        """PD controller. Feed-forward velocity is used only in force mode, otherwise its values are zeros.

        Args:
            x_d (numpy.ndarray): 5x3 array of desired legs' positions.
            x_a (numpy.ndarray): 5x3 array of actual legs' positions.
            dx_d (numpy.ndarray): 5x3 array of feed-forward velocities.
            ddx_d (numpy.ndarray): 5x3 array of feed-forward accelerations.

        Returns:
            tuple: Two 5x3 arrays of commanded legs velocities and current position errors.
        """
        legs_position_errors = np.array(x_d - x_a)
        dx_e = (legs_position_errors - self.last_legs_position_errors) / self.PERIOD
        dx_c = np.array(self.k_p * legs_position_errors + self.k_d * dx_e + dx_d + config.K_ACC * ddx_d, dtype = np.float32)

        return dx_c, legs_position_errors

    def __force_position_p_controller(self, f_d, f_a):
        """Force-position P controller. Calculate position offsets and desired legs velocities in spider's origin from desired forces.

        Args:
            f_d (list): 5x3 array of desired forces in spider's origin.
            f_a (list): 5x3 array of measured current forces in spider's origin.

        Returns:
            tuple: Two 5x3 array of of position offsets of leg-tips and leg-tips velocities, given in spider's origin.
        """
        force_errors = f_d - f_a
        legs_positions_in_spider = force_errors * config.K_P_FORCE
        offsets = legs_positions_in_spider * self.PERIOD

        return offsets, legs_positions_in_spider
    #endregion