import numpy as np
import threading
import time
import os
from gwpconfig import commconstants as cc
from gwpconfig import wall

import config
import controllers
from utils import threadmanager
from utils import jsonfilemanager
from utils import csvfilemanager
from periphery import dynamixel as dmx
from periphery import arduinocomm
from communication import servercomm
import spider
from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import mathtools
from calculations import transformations as tf
from planning import pathplanner

class App:
    """Application layer.
    """
    def __init__(self, do_record_data = False):
        self.q_a = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.x_a = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.tau_a = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.motors_errors = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.temperatures = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.dq_c = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)

        self.joints_velocity_controller = controllers.VelocityController()
        self.motor_driver = dmx.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]])
        self.pumps_bno_arduino = arduinocomm.WaterPumpsBnoArduino()
        self.thread_manager = threadmanager.CustomThread()
        self.json_file_manager = jsonfilemanager.JsonFileManager()
        self.server_comm = servercomm.ServerComm(self.json_file_manager.FILENAME)

        self.locker = threading.Lock()
        self.safety_kill_event = threading.Event()
        self.safety_thread = None
        self.motor_control_thread = None
        self.safety_thread_kill_event = None
        self.motor_control_thread_kill_event = None
        self.spider_state_thread = None
        self.spider_states_thread_kill_event = None

        self.current_state = None
        self.watering_counter = 0

        self.was_in_resting_state = False
        self.last_plant_or_refill_position = None

        self.init_bno = True
        self.is_working_init = True

        self.moving_leg_id = None

        self.is_gripper_error = False

        self.do_record_data = do_record_data
        if do_record_data:
            self.csv_file_manager = csvfilemanager.CsvFileManager()
            
        self.__init_layers()

    def safety_layer(self):
        """Continuously checking for hardware errors on the motors. If error occurs, inititate resting procedure and then continue with working. 
        """
        def safety_checking(kill_event: threading.Event):
            self.safety_kill_event.clear()
            while True:
                if kill_event.is_set():
                    break
                with self.locker:
                    is_motors_errors = self.motors_errors
                    is_gripper_error = self.is_gripper_error

                is_microswitch_error = self.__is_microswitch_error()
                if is_microswitch_error:
                    self.server_comm.send_message(cc.MICROSWITCH_ERROR)
                is_motors_error = is_motors_errors.any() and (self.current_state == config.WORKING_STATE)
                if is_motors_error:
                    self.server_comm.send_message(cc.ERROR_IN_MOTOR_WARNING)
               
                is_error = is_microswitch_error or is_motors_error or is_gripper_error

                if is_error:
                    self.__error_case_procedure()

                time.sleep(0.1)
        self.safety_thread, self.safety_thread_kill_event = self.thread_manager.run(safety_checking, config.SAFETY_THREAD_NAME, False, True, do_print = True)
  
    def motor_control_layer(self):
        """Motors control layer with loop for reading, recalculating and sending data to the motors.
        """
        def control_loop(kill_event: threading.Event):
            force_buffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
            tau_buffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
            tau_counter = 0
            force_counter = 0
            do_init = True
            self.motor_driver.set_bus_watchdog(15)
            while True:
                start_time = time.perf_counter()

                if kill_event.is_set():
                    break
                
                # Reading data from motors.
                try:
                    q_a, i_a, hw_errors, temperatures = self.motor_driver.sync_read_motors_data()
                except KeyError:
                    print("Reading error.")
                    continue

                # Calculating input data for controller.
                x_a = kin.all_legs_positions(q_a, config.LEG_ORIGIN)
                tau, f = dyn.get_torques_and_forces_on_legs_tips(q_a, i_a, self.pumps_bno_arduino.get_gravity_vector())
                tau_mean, tau_buffer, tau_counter = mathtools.running_average(tau_buffer, tau_counter, tau)
                force_mean, force_buffer, force_counter = mathtools.running_average(force_buffer, force_counter, f)

                with self.locker:
                    self.q_a = q_a
                    self.motors_errors = hw_errors
                    self.temperatures = temperatures
                    self.x_a = x_a
                    self.tau_a = tau_mean

                # Controller.
                dq_c = self.joints_velocity_controller.joints_velocity_controller(q_a, x_a, force_mean, do_init)
                if do_init:
                    do_init = False

                # Sending velocities to motors.
                self.motor_driver.sync_write_motors_velocities_in_legs(dq_c)

                # Enforce desired frequency.
                elapsed_time = time.perf_counter() - start_time
                # TODO: test this implementation of sleeping. It should ensure more precise loop period, but what happens if control loop already takes longer that period?
                try:
                    time.sleep(self.joints_velocity_controller.PERIOD - (elapsed_time % self.joints_velocity_controller.PERIOD))
                except ValueError:
                    time.sleep(0)

        self.motor_control_thread, self.motor_control_thread_kill_event = self.thread_manager.run(control_loop, config.CONTROL_THREAD_NAME, False, True, do_print = True)

    def spider_states_manager(self, state: str, do_wait: bool):
        """Managing spider's state.

        Args:
            state (str): Name of desired state.
            do_wait (bool): If True, wait for state to finish.
        """
        if state == config.WORKING_STATE:
            self.spider_state_thread = self.thread_manager.run(self.working, config.WORKING_THREAD_NAME, True, True, False, False)
        elif state == config.RESTING_STATE:
            self.spider_state_thread, self.spider_states_thread_kill_event = self.thread_manager.run(self.rest, config.RESTING_THREAD_NAME, True, True, True, False)
        elif state == config.TRANSITION_STATE:
            self.spider_state_thread = self.thread_manager.run(self.transition_to_rest_state, config.TRANSITION_THREAD_NAME, True, True, False, False)
        #TODO: Test joining.
        if do_wait:
            self.spider_state_thread.join()
    
    def working(self):
        """Working procedure, includes walking and watering the plants.
        """
        self.current_state = config.WORKING_STATE
        self.is_working_init = True

        with self.locker:
            self.is_gripper_error = False
        
        self.server_comm.send_message(cc.WORKING_PHASE_STARTED_MESSAGE)
        
        while True:
            spider_pose, _, start_legs_positions = self.json_file_manager.read_spider_state()
            #TODO: Check new implementation.
            poses, pins_instructions, watering_leg_id, plant_or_refill_position, volume = self.__get_movement_instructions(
                spider_pose,
                start_legs_positions,
            )

            for step, pose in enumerate(poses):
                current_pins_positions = pins_instructions[step, :, 1:]
                current_legs_moving_order = pins_instructions[step, :, 0].astype(int)
                with self.locker:
                    x_a = self.x_a

                if step == 0:
                    self.joints_velocity_controller.move_legs_sync(current_legs_moving_order, x_a, current_pins_positions, config.GLOBAL_ORIGIN, 5, config.MINJERK_TRAJECTORY, pose)
                    self.json_file_manager.update_whole_dict(pose, current_pins_positions, current_legs_moving_order)
                    if self.safety_kill_event.wait(timeout = 5.5):
                        return

                    if self.init_bno:
                        self.pumps_bno_arduino.reset_bno()
                        self.init_bno = False
                        if self.safety_kill_event.wait(timeout = 1.0):
                            return
                    if not self.__distribute_forces(spider.LEGS_IDS, config.FORCE_DISTRIBUTION_DURATION):
                        return
                    continue

                previous_pins_positions = np.array(pins_instructions[step - 1, :, 1:])
                self.joints_velocity_controller.move_legs_sync(current_legs_moving_order, x_a, previous_pins_positions, config.GLOBAL_ORIGIN, 2.5, config.MINJERK_TRAJECTORY, pose)
                self.json_file_manager.update_whole_dict(pose, previous_pins_positions, current_legs_moving_order)
                if self.safety_kill_event.wait(timeout = 3.0):
                    return

                #TODO: Check new implementation.
                if not self.__move_legs_on_next_pins(current_pins_positions, previous_pins_positions, current_legs_moving_order, pose):
                    return

            watering_success = self.__watering(watering_leg_id, plant_or_refill_position, poses[-1], watering_leg_id == spider.REFILLING_LEG_ID, volume)
            if not watering_success:
                return
        
    def rest(self, kill_event: threading.Event):
        """Resting procedure, includes option for manually correcting non-attached leg. Resting lasts until temperatures of all motors
        drop below working temperature.

        Args:
            kill_event (threading.Event): Event for killing a procedure.
        """
        self.server_comm.send_message(cc.RESTING_PHASE_STARTED_MESSAGE)
        self.current_state = config.RESTING_STATE

        #TODO: Check new implementation.
        with self.locker:
            is_motors_error = self.motors_errors.any()
        if is_motors_error:
            self.server_comm.send_message(cc.REBOOTING_MOTORS_MESSAGE)
            if not self.__reboot_motors_in_error(kill_event):
                return

        #TODO: Check new implementation.
        attached_legs = self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs()
        if len(attached_legs) != spider.NUMBER_OF_LEGS:
            unattached_legs = np.setdiff1d(spider.LEGS_IDS, attached_legs)
            if not self.__handle_unattached_legs(unattached_legs, kill_event):
                return

        #TODO: Check new implementation.
        with self.locker:
            temperatures = self.temperatures
        if (temperatures > self.motor_driver.MAX_WORKING_TEMPERATURE).any():
            self.server_comm.send_message(cc.TEMPERATURES_IN_MOTORS_TOO_HIGH_WARNING)
            if not self.__wait_for_temperature_drop(kill_event):
                return
        
        # Check for possible singularity of lower legs.
        _, _, legs_global_positions = self.json_file_manager.read_spider_state()
        if np.linalg.norm(legs_global_positions[2] - legs_global_positions[3]) <= 0.22:
            self.__handle_possible_spider_singularity()

        # Update last positions and enable torques in motors.
        with self.locker:
            x_a = self.x_a
        self.joints_velocity_controller.update_last_legs_positions(x_a)
        if kill_event.wait(timeout = 1.0):
            return
        self.motor_driver.enable_disable_legs(config.ENABLE_LEGS_COMMAND)
    
    def transition_to_rest_state(self):
        """Procedure for transition between working and resting state, by using force mode. 
        """
        #TODO: When breaks are implemented, this method will handle breaks activation.
        self.server_comm.send_message(cc.TRANSITIONING_TO_RESTING_PHASE_MESSAGE)
        self.current_state = config.TRANSITION_STATE
        self.joints_velocity_controller.clear_instruction_queues()
        f_d = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.joints_velocity_controller.start_force_mode(spider.LEGS_IDS, f_d)
        time.sleep(10)
        self.joints_velocity_controller.stop_force_mode()

    #TODO: Only for testing. Delete after.
    def test_prediction_model(self):
        init_spider_pose = np.array([3.0, 1.8, 0.3])
        pins = wall.create_grid(True)
        used_pins = [pins[204], pins[177], pins[187], pins[213], pins[216]]
        with self.locker:
            x_a = self.x_a
        self.joints_velocity_controller.move_legs_sync(spider.LEGS_IDS, x_a, used_pins, config.GLOBAL_ORIGIN, 5, config.MINJERK_TRAJECTORY, init_spider_pose)
        time.sleep(5.5)
        while True:
            with self.locker:
                x_a = self.x_a
            random_spider_move = np.random.uniform(low = -0.1, high = 0.1, size = (2, ))
            random_spider_move = np.append(random_spider_move, 0.0)
            spider_pose = init_spider_pose + random_spider_move
            self.joints_velocity_controller.move_legs_sync(spider.LEGS_IDS, x_a, used_pins, config.GLOBAL_ORIGIN, 1.5, config.MINJERK_TRAJECTORY, spider_pose)
            time.sleep(2)

            self.__pin_to_pin_leg_movement(0, used_pins[0], used_pins[0] + 1, spider_pose, True)
            self.__pin_to_pin_leg_movement(0, used_pins[0] + 1, used_pins[0], spider_pose, True)

    # region private methods
    def __init_layers(self):
        """Initialize application layers.
        """
        self.safety_layer()
        time.sleep(2)
        self.motor_control_layer()
    
    def __distribute_forces(self, legs_ids: list, duration: float) -> bool:
        """Run force distribution process in a loop for a given duration.

        Args:
            legs_ids (list): List of legs ids.
            duration (float): Distributing duration.

        Returns:
            bool: True if distribution was successful, False otherwise.
        """
        offload_leg_id = np.setdiff1d(spider.LEGS_IDS, legs_ids)
        if len(offload_leg_id) > 1:
            return False

        start_time = time.perf_counter()
        elapsed_time = 0
        while elapsed_time < duration:
            with self.locker:
                tau_a = self.tau_a
                q_a = self.q_a
            distributed_forces = dyn.calculate_distributed_forces(tau_a, q_a, legs_ids, offload_leg_id)

            if len(offload_leg_id):
                distributed_forces = np.insert(distributed_forces, offload_leg_id[0], np.zeros(3, dtype = np.float32), axis = 0)
            self.joints_velocity_controller.start_force_mode(spider.LEGS_IDS, distributed_forces)

            elapsed_time = time.perf_counter() - start_time
            if self.safety_kill_event.wait(timeout = self.joints_velocity_controller.PERIOD):
                self.joints_velocity_controller.stop_force_mode()
                return False
        
        self.joints_velocity_controller.stop_force_mode()
        if self.safety_kill_event.wait(timeout = 2.0):
            return False
        return True
    
    def __pin_to_pin_leg_movement(self, leg: int, current_pin_position: np.ndarray, goal_pin_position: np.ndarray, pose: np.ndarray, use_prediction_model: bool = False) -> bool:
        """Move leg from one pin to another.

        Args:
            leg (int): Leg id.
            current_pin_position (np.ndarray): Position of currently used pin.
            goal_pin_position (np.ndarray): Position of goal pin.
            pose (np.ndarray): Spider's pose.

        Returns:
            bool: True if movement was successful, False otherwise.
        """
        # Read legs positions before movement.
        if self.do_record_data:
            with self.locker:
                x_a_before = self.x_a

        # Distribute forces among other legs.
        if not self.__distribute_forces(np.delete(spider.LEGS_IDS, leg), config.FORCE_DISTRIBUTION_DURATION):
            return False

        if not self.__release_leg(leg, pose, goal_pin_position):
            return False

        # Calculate pin-to-pin direction.
        rpy = self.__read_spider_rpy(legs_ids_to_use = np.delete(spider.LEGS_IDS, leg))
        pin_to_pin_vector_in_local, leg_base_orientation_in_global = tf.get_pin_to_pin_vector_in_local(leg, rpy, current_pin_position, goal_pin_position)
        global_z_direction_in_local = np.dot(leg_base_orientation_in_global, np.array([0.0, 0.0, 1.0], dtype = np.float32))
        
        movement_success, leg_goal_position_in_local = self.__move_leg_pin_to_pin(leg, pin_to_pin_vector_in_local, goal_pin_position, rpy, use_prediction_model)
        if not movement_success:
            return False

        # Before closing the gripper, put leg in force mode to avoid pulling the spider with gripper.
        if not self.__attach_to_pin(leg):
            return False

        number_of_tries = 1
        # Correction in case of missed pin.
        if leg not in self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs():
            correction_success, number_of_corrections = self.__correction(leg, global_z_direction_in_local, leg_goal_position_in_local)
            if not correction_success:
                return False
            number_of_tries += number_of_corrections

        if self.do_record_data:
            with self.locker:
                x_a_after = self.x_a
            self.csv_file_manager.write_row(x_a_before, rpy, leg, leg_goal_position_in_local, number_of_tries, x_a_after)

        self.server_comm.send_message(cc.LEG_MOVE_MESSAGE)

        return True
    
    def __correction(self, leg_id: int, global_z_direction_in_local: np.ndarray, leg_goal_position_in_local: np.ndarray) -> tuple[bool, int]:
        """Correction procedure wrapper, first starts automatic correction, if that fails starts manual correction procedure.

        Args:
            leg_id (int): Leg id.
            global_z_direction_in_local (np.ndarray): Global z axis direction.
            leg_goal_position_in_local (np.ndarray): Leg's goal position.

        Returns:
            tuple[bool, int]: Correction success and number of tries.
        """
        self.server_comm.send_message(cc.LEG_MISSED_PIN_WARNING)
        auto_correction_success, number_of_corrections = self.__automatic_correction(leg_id, global_z_direction_in_local, leg_goal_position_in_local)
        if auto_correction_success:
            self.server_comm.send_message(cc.AUTO_CORRECTION_SUCCESSFUL_MESSAGE)
            return (auto_correction_success, number_of_corrections)
        self.server_comm.send_message(cc.AUTO_CORRECITON_ERROR)
        manual_corection_success = self.__manual_correction(leg_id)
        return (manual_corection_success, 10)

    def __automatic_correction(self, leg_id: int, global_z_direction_in_local: np.ndarray, leg_goal_position_in_local: np.ndarray) -> tuple[bool, int]:
        """Automatic correction procedure. Robot tries to grab a pin in 9 tries.

        Args:
            leg_id (int): Leg id.
            global_z_direction_in_local (np.ndarray): Global z axis direction.
            leg_goal_position_in_local (np.ndarray): Leg's goal position.

        Returns:
            tuple[bool, int]: Correction success and number of tries.
        """
        detach_z_offset = 0.08
        offset_value = 0.15
        offsets = [
            [0.0, 0.0, 0.0],
            [0.0, offset_value, 0.0],
            [0.0, -offset_value, 0.0],
            [offset_value, 0.0, 0.0],
            [-offset_value, 0.0, 0.0],
            [offset_value, offset_value, 0.0],
            [offset_value, -offset_value, 0.0],
            [-offset_value, -offset_value, 0.0],
            [-offset_value, offset_value, 0.0]
        ]
        
        detach_position = np.copy(leg_goal_position_in_local)
        detach_position[2] += detach_z_offset
        for idx, offset in enumerate(offsets):
            number_of_tries = idx + 1
            self.joints_velocity_controller.grippers_arduino.move_gripper(leg_id, self.joints_velocity_controller.grippers_arduino.OPEN_COMMAND)
            if self.safety_kill_event.wait(timeout = 1.5):
                return (False, number_of_tries)
 
            with self.locker:
                x_a = self.x_a
            self.joints_velocity_controller.move_leg_async(leg_id, x_a, detach_position, config.LEG_ORIGIN, 1, config.MINJERK_TRAJECTORY)
            if self.safety_kill_event.wait(timeout = 1.5):
                return (False, number_of_tries)
            
            velocity_direction = -(global_z_direction_in_local + offset)
            self.joints_velocity_controller.start_velocity_mode(leg_id, velocity_direction)
            if self.safety_kill_event.wait(timeout = 2.0):
                self.joints_velocity_controller.stop_velocity_mode()
                return (False, number_of_tries)
            self.joints_velocity_controller.stop_velocity_mode()

            self.joints_velocity_controller.grippers_arduino.move_gripper(leg_id, self.joints_velocity_controller.grippers_arduino.CLOSE_COMMAND)
            if self.safety_kill_event.wait(timeout = 2.0):
                return (False, number_of_tries)

            if leg_id in self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs():
                return (True, number_of_tries)

        return (False, number_of_tries)

    def __manual_correction(self, leg_id: int, use_safety: bool = True) -> bool:
        """Manual correction procedure. Human operator is required to manually moved leg on correct pin. 

        Args:
            leg_id (int): Leg id.
            use_safety (bool, optional): Whether or not to consider safety checking. Defaults to True.

        Returns:
            bool: True if correction was successful, False otherwise.
        """
        _, used_pins_ids, legs_global_positions = self.json_file_manager.read_spider_state()
        goal_pin_id = used_pins_ids[leg_id]

        self.server_comm.send_message(cc.create_instruction_message(cc.MANUALLY_MOVE_LEG_ON_PIN_INSTRUCTION, leg_id, goal_pin_id, ))
        self.joints_velocity_controller.grippers_arduino.move_gripper(leg_id, self.joints_velocity_controller.grippers_arduino.OPEN_COMMAND)
        with self.locker:
            x_a_leg = self.x_a[leg_id]
            q_a = self.q_a

        attached_legs = self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs()

        goal_pin_in_local = kin.get_goal_pin_in_local(leg_id, attached_legs, legs_global_positions, q_a, self.pumps_bno_arduino.get_rpy())

        distance = np.linalg.norm(goal_pin_in_local - x_a_leg)
        #TODO: Disabling legs here probably won't be needed after breaks implementation.
        do_disable_legs = not (spider.LEG_LENGTH_MIN_LIMIT < np.linalg.norm(goal_pin_in_local) < spider.LEG_LENGTH_MAX_LIMIT)
        if do_disable_legs:
            self.motor_driver.enable_disable_legs(attached_legs, config.DISABLE_LEGS_COMMAND)

        self.joints_velocity_controller.start_force_mode([leg_id], [np.zeros(3, dtype = np.float32)])
        self.server_comm.send_message(cc.LEG_IN_MANUAL_CORRECTION_MODE_MESSAGE)
        while True:
            with self.locker:
                x_a_leg = self.x_a[leg_id]
                if do_disable_legs:
                    q_a = self.q_a
            
            if do_disable_legs:
                goal_pin_in_local = kin.get_goal_pin_in_local(leg_id, attached_legs, legs_global_positions, q_a, self.pumps_bno_arduino.get_rpy())

            distance = np.linalg.norm(goal_pin_in_local - x_a_leg)
            switch_state = int(self.joints_velocity_controller.grippers_arduino.get_tools_states(self.joints_velocity_controller.grippers_arduino.SWITCH)[leg_id])

            if (switch_state == 0) and (distance < 0.1):
                self.joints_velocity_controller.grippers_arduino.move_gripper(leg_id, self.joints_velocity_controller.grippers_arduino.CLOSE_COMMAND)
                time.sleep(2)
                break

            if use_safety:
                if self.safety_kill_event.wait(timeout = 0.1):
                    self.joints_velocity_controller.stop_force_mode()
                    return False
            else:
                time.sleep(0.1)
        self.joints_velocity_controller.stop_force_mode()

        return True

    def __watering(self, watering_leg_id: int, plant_or_refill_position: np.ndarray, spider_pose: np.ndarray, do_refill: bool, volume: float) -> bool:
        """Watering the plant or refillint water tank.

        Args:
            watering_leg_id (int): Id of leg in which water pump will be activated.
            plant_or_refill_position (np.ndarray): Plant position or refill position.
            spider_pose (np.ndarray): Spider's pose.
            do_refill (bool): Whether or not to refill water tank.
            volume (bool): Volume of water whether for refilling or watering, in millilitres.

        Returns:
            bool: True if watering or refilling was successful, False otherwise.
        """
        #TODO: Convert desired volume of water into time that specific water pump needs to pump that volume.
        if not self.__distribute_forces(np.delete(spider.LEGS_IDS, watering_leg_id), config.FORCE_DISTRIBUTION_DURATION):
            return False

        # Move leg on watering position.
        _, _, legs_global_positions = self.json_file_manager.read_spider_state()
        with self.locker:
            q_a = self.q_a
            x_a_before = self.x_a
        spider_pose = kin.get_spider_pose(spider.LEGS_IDS, legs_global_positions, q_a)

        self.joints_velocity_controller.grippers_arduino.move_gripper(watering_leg_id, self.joints_velocity_controller.grippers_arduino.OPEN_COMMAND)
        if self.safety_kill_event.wait(timeout = 1.0):
            if not do_refill:
                return False

        self.joints_velocity_controller.move_leg_async(
            watering_leg_id,
            x_a_before,
            plant_or_refill_position,
            config.GLOBAL_ORIGIN,
            3,
            config.BEZIER_TRAJECTORY,
            spider_pose)
        if self.safety_kill_event.wait(timeout = 3.5):
            if not do_refill:
                return False
        
        # Close gripper to allow refilling.
        if do_refill:
            self.joints_velocity_controller.grippers_arduino.move_gripper(watering_leg_id, self.joints_velocity_controller.grippers_arduino.CLOSE_COMMAND)

        # Turn on water pump.
        pump_id = self.__select_water_pump_id(watering_leg_id)
        pumping_time = config.REFILL_TIME if do_refill else config.WATERING_TIME

        notify_message = cc.REFILLING_STARTED_MESSAGE if pump_id == spider.REFILLING_LEG_ID else cc.WATERING_STARTED_MESSAGE
        self.server_comm.send_message(notify_message)

        start_time = time.perf_counter()
        while True:
            elapsed_time = time.perf_counter() - start_time
            self.pumps_bno_arduino.water_pump_controll(self.pumps_bno_arduino.PUMP_ON_COMMAND, pump_id)
            if elapsed_time > pumping_time:
                if not do_refill:
                    self.pumps_bno_arduino.water_pump_controll(self.pumps_bno_arduino.PUMP_OFF_COMMAND, pump_id)
                    self.server_comm.send_message(cc.WATERING_FINISHED_MESSAGE)
                else:
                    self.joints_velocity_controller.grippers_arduino.move_gripper(watering_leg_id, self.joints_velocity_controller.grippers_arduino.OPEN_COMMAND)
                break
            if not do_refill:
                if self.safety_kill_event.wait(timeout = 0.05):
                    self.pumps_bno_arduino.water_pump_controll(self.pumps_bno_arduino.PUMP_OFF_COMMAND, pump_id)
                    self.server_comm.send_message(cc.WATERING_FINISHED_MESSAGE)
                    return False
            else:
                time.sleep(0.05)
        
        # Move leg back on starting position.
        with self.locker:
            x_a_after = self.x_a

        self.joints_velocity_controller.move_leg_async(watering_leg_id, x_a_after, x_a_before[watering_leg_id], config.LEG_ORIGIN, 3, config.BEZIER_TRAJECTORY)
        if self.safety_kill_event.wait(timeout = 3.5):
            if not do_refill:
                return False
            
        if do_refill:
            self.pumps_bno_arduino.water_pump_controll(self.pumps_bno_arduino.PUMP_OFF_COMMAND, pump_id)
            self.watering_counter = 0
            self.server_comm.send_message(cc.REFILLING_FINISHED_MESSAGE)
        else:
            self.watering_counter += 1
            
        self.joints_velocity_controller.grippers_arduino.move_gripper(watering_leg_id, self.joints_velocity_controller.grippers_arduino.CLOSE_COMMAND)
        if self.safety_kill_event.wait(1.0):
            if not do_refill:
                return False
        
        # Correct if leg did not reach starting pin successfully.
        if watering_leg_id not in self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs():
            rpy = self.__read_spider_rpy(np.delete(spider.LEGS_IDS, watering_leg_id))
            spider_orientation_in_global = tf.xyzrpy_to_matrix(rpy, True)
            leg_base_orientation_in_global = np.linalg.inv(np.dot(spider_orientation_in_global, spider.T_BASES[watering_leg_id][:3, :3]))
            global_z_direction_in_local = np.dot(leg_base_orientation_in_global, np.array([0.0, 0.0, 1.0], dtype = np.float32))

            return self.__correction(watering_leg_id, global_z_direction_in_local, x_a_before[watering_leg_id])

        return True
    
    def __get_watering_instructions(self, spider_pose: np.ndarray) -> tuple[int, np.ndarray, np.ndarray, float]:
        """Get instructions for watering or refilling.

        Args:
            spider_pose (np.ndarray): Spider's pose.

        Returns:
            tuple[int, np.ndarray, np.ndarray, float]: Leg id, spider's goal pose, plant or refill position, desired amount of water to be pumped in mL.
        """
        # If robot does not start from resting state, request new instructions.
        if not self.was_in_resting_state:
            plant_or_refill_position, do_refill_water_tank, volume = self.server_comm.request_watering_action_instructions()
            self.last_plant_or_refill_position = plant_or_refill_position
            watering_leg_id, goal_pose = tf.get_watering_leg_and_pose(spider_pose, plant_or_refill_position)

            # Calculate refill position and select leg.
            if do_refill_water_tank:
                watering_leg_id, goal_pose = tf.get_watering_leg_and_pose(spider_pose, do_refill = True)
                #TODO: Include refilling leg position in server-side code.
                plant_or_refill_position = goal_pose[:3] + spider.REFILLING_LEG_OFFSET

                return watering_leg_id, goal_pose, plant_or_refill_position, volume

            return watering_leg_id, goal_pose, plant_or_refill_position, volume

        # If robot starts from resting state, continue movement towards last commanded plant / refill position.
        #TODO: Handle the case if program is shut down during movement.
        plant_or_refill_position = self.last_plant_or_refill_position
        self.was_in_resting_state = False
        watering_leg_id, goal_pose = tf.get_watering_leg_and_pose(spider_pose, plant_or_refill_position)

        return watering_leg_id, goal_pose, plant_or_refill_position, volume

    def __error_case_procedure(self):
        """Procedure in case of error.
        """
        self.safety_kill_event.set()

        # Wait for working thread to stop.
        if config.WORKING_THREAD_NAME in self.spider_state_thread.name:
            self.spider_state_thread.join()
            self.server_comm.send_message(cc.WORKING_PHASE_FINISHED_MESSAGE)

        # Start transition state and wait for it to finish.
        self.spider_states_manager(config.TRANSITION_STATE, True)

        # Start resting state and wait for it to finish.
        self.spider_states_manager(config.RESTING_STATE, True)
        self.server_comm.send_message(cc.RESTING_PHASE_FINISHED_MESSAGE)
        
        self.was_in_resting_state = True

        self.safety_kill_event.clear()
        time.sleep(1)

        # Continue with working.
        self.spider_states_manager(config.WORKING_STATE, False)
    
    def __is_microswitch_error(self) -> bool:
        """Check if any of microswitches is in error.

        Returns:
            bool: True if one or more microswitches are in error, False otherwise.
        """
        switches_states = self.joints_velocity_controller.grippers_arduino.get_tools_states(self.joints_velocity_controller.grippers_arduino.SWITCH)
        return switches_states[self.moving_leg_id] == self.joints_velocity_controller.grippers_arduino.IS_CLOSE_RESPONSE
    
    def __get_movement_instructions(self, spider_pose: np.ndarray, start_legs_positions: np.ndarray) -> tuple[list, list, int, np.ndarray, float]:
        """Get all neccessary instructions for moving between two points on the wall.

        Args:
            spider_pose (np.ndarray): Spider's current pose.
            start_legs_positions (np.ndarray): Array of legs' current positions.

        Returns:
            tuple[list, list, int, np.ndarray, float]: List of poses, list of pins, id of leg for watering, plant or refill position, desired volume of water to be pumped in mL.
        """
        while True:
            try:
                watering_leg_id, goal_pose, plant_or_refill_position, volume = self.__get_watering_instructions(spider_pose)
                    
                if self.is_working_init:
                    planning_result = pathplanner.modified_walking_instructions(start_legs_positions, goal_pose)
                    if not planning_result:
                        os._exit(0)
                    else:
                        poses, pins_instructions = planning_result
                    self.is_working_init = False
                    break
                    
                poses, pins_instructions = pathplanner.create_walking_instructions(spider_pose, goal_pose)
                break
            except Exception:
                #TODO: Include in error/warning/message logic.
                print("Error in path planning. Trying again... ")
            time.sleep(0.05)
        
        return poses, pins_instructions, watering_leg_id, plant_or_refill_position, volume

    def __move_legs_on_next_pins(self, current_pins_positions:np.ndarray, previous_pins_positions: np.ndarray, legs_moving_order: list, pose: list) -> bool:
        """Execute legs' pin to pin movements.

        Args:
            current_pins_positions (np.ndarray): Array of currently used pins' positions.
            previous_pins_positions (np.ndarray): Array of previously used pins' positions.
            legs_moving_order (list): Moving order of legs.
            pose (list): Spider's pose.

        Returns:
            bool: True if movements were successfull, False otherwise.
        """
        previous_to_current_pins_offsets = current_pins_positions - previous_pins_positions
        for idx, leg in enumerate(legs_moving_order):
            if previous_to_current_pins_offsets[idx].any():
                movement_success = self.__pin_to_pin_leg_movement(leg, previous_pins_positions[idx], current_pins_positions[idx], pose)
                if not movement_success:
                    return False
        
        return True
    
    def __reboot_motors_in_error(self, kill_event: threading.Event) -> bool:
        """Reboot all motors that are in error state.

        Args:
            kill_event (threading.Event): Event for killing the resting thread.

        Returns:
            bool: True if rebooting was successfull, False otherwise.
        """
        with self.locker:
            hw_errors = self.motors_errors
        motors_in_error = self.motor_driver.motors_ids[np.where(hw_errors != 0)]
        self.motor_driver.reboot_motors(motors_in_error)
        while hw_errors.any():
            with self.locker:
                hw_errors = self.motors_errors
            if kill_event.wait(timeout = 0.1):
                return False
        
        return True
    
    def __handle_unattached_legs(self, unattached_legs: np.ndarray, kill_event: threading.Event) -> bool:
        """Handle situation where one or more legs are not attached to the wall (but they should be).

        Args:
            unattached_legs (np.ndarray): Array of unattached legs' ids.
            kill_event (threading.Event): Event for killing the resting thread.

        Returns:
            bool: Whether or not handling was successfull.
        """
        #TODO: This case will be probably solved with breaks.
        if unattached_legs.size > 1:
            #TODO: Include in error/warning/message logic.
            print("UNATTACHED LEGS IDS: ", unattached_legs)
        while unattached_legs.size > 1:
            unattached_legs = np.setdiff1d(spider.LEGS_IDS, self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs())
            if kill_event.wait(timeout = 0.1):
                return False

        if unattached_legs.size == 1:
            self.server_comm.send_message(cc.LEG_IN_MANUAL_CORRECTION_MODE_MESSAGE)
            correction_success = self.__manual_correction(unattached_legs[0], use_safety = False)
            if correction_success:
                #TODO: Include in error/warning/message logic.
                print("ALL LEGS ATTACHED.")
        return correction_success
    
    def __wait_for_temperature_drop(self, kill_event: threading.Event) -> bool:
        """Wait until temperatures in all motors drop below allowed value.

        Args:
            kill_event (threading.Event): Event for killng the resting thread.

        Returns:
            bool: Whether or not waiting was successfull.
        """
        while True:
            with self.locker:
                temperatures = self.temperatures
            if kill_event.wait(timeout = 1.0) or (temperatures < self.motor_driver.MAX_WORKING_TEMPERATURE).all():
                break

        return not kill_event.is_set()
    
    def __handle_possible_spider_singularity(self):
        """Handle situation where robot singularity is possible. Requires human's input.
        """
        self.server_comm.send_message(cc.STARTUP_SINGULARITY_ERROR)

        #TODO: See if that will still be necessary when breaks are implemented.
        self.motor_driver.enable_disable_legs(config.DISABLE_LEGS_COMMAND)
        confirm_input = input("PRESS ENTER TO CONTINUE OR K + ENTER TO KILL A PROGRAM.")
        if confirm_input == config.PROGRAM_KILL_KEY:
            print("KILLING A PROGRAM...")
            os._exit(0)

    def __start_pushing_to_prevent_slipping(self, leg: int, pose: list, pin_position: np.ndarray) -> bool:
        """Use force mode to push with selected leg into the wall. This maneuver is used to prevent slipping when detaching.

        Args:
            leg (int): Leg id.
            pose (list): Spider's pose.
            pin_position (np.ndarray): Position of currently used pin.

        Returns:
            bool: Whether or not pushing was successfull.
        """
        with self.locker:
            q_a_leg = self.q_a[leg]
        last_joint_position_in_local = kin.leg_base_to_third_joint_forward_kinematics(q_a_leg)[:,3][:3]
        last_joint_to_goal_pin_in_spider_unit = tf.get_last_joint_to_goal_pin_vector_in_spider(leg, last_joint_position_in_local, pin_position, pose)
        
        self.joints_velocity_controller.start_force_mode([leg], [last_joint_to_goal_pin_in_spider_unit * config.F_D_PUSHING])
        if self.safety_kill_event.wait(timeout = 1.0):
            self.joints_velocity_controller.stop_force_mode()
            return False
        return True
    
    def __release_leg(self, leg: int, pose: list, goal_pin_position: np.ndarray) -> bool:
        """Release the leg from pin, before moving it to the next one.

        Args:
            leg (int): Leg id.
            pose (list): Spider's pose.
            goal_pin_position (np.ndarray): Position of currently used pin.

        Returns:
            bool: True if releasing was successfull, False otherwise.
        """
        if not self.__start_pushing_to_prevent_slipping(leg, pose, goal_pin_position):
            return False

        # Prevent leg movement if not all legs are attached to the wall.
        if len(self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs()) != spider.NUMBER_OF_LEGS:
            self.server_comm.send_message(cc.ALL_LEGS_NOT_ATTACHED_ERROR)
            return False
        
        # Open the gripper.
        self.joints_velocity_controller.grippers_arduino.move_gripper(leg, self.joints_velocity_controller.grippers_arduino.OPEN_COMMAND)
        if self.safety_kill_event.wait(timeout = 3.5):
            self.joints_velocity_controller.stop_force_mode()
            return False

        #TODO: Check new implementation.
        gripper_state = self.joints_velocity_controller.grippers_arduino.get_tools_states(self.joints_velocity_controller.grippers_arduino.GRIPPER)[leg]
        if gripper_state == self.joints_velocity_controller.grippers_arduino.IS_CLOSE_RESPONSE:
            with self.locker:
                self.is_gripper_error = True
            self.server_comm.send_message(cc.GRIPPER_ERROR)
            self.joints_velocity_controller.stop_force_mode()
            return False

        self.joints_velocity_controller.stop_force_mode()
        if self.safety_kill_event.wait(timeout = 1.0):
            return False
        
        return True
    
    def __attach_to_pin(self, leg: int) -> bool:
        """Attach leg to a pin.

        Args:
            leg (int): Leg id.

        Returns:
            bool: True if attaching was successfull, False otherwise.
        """
        self.joints_velocity_controller.start_force_mode(np.array([leg]), np.array([np.zeros(3, dtype = np.float32)]))
        self.joints_velocity_controller.grippers_arduino.move_gripper(leg, self.joints_velocity_controller.grippers_arduino.CLOSE_COMMAND)
        if self.safety_kill_event.wait(timeout = 3.0):
            self.joints_velocity_controller.stop_force_mode()
            return False
        self.joints_velocity_controller.stop_force_mode()

        return True
    
    def __read_spider_rpy(self, legs_ids_to_use: np.ndarray = spider.LEGS_IDS, use_state_dict: bool = True, legs_global_positions: np.ndarray = None) -> list:
        """Read spider's rpy values.

        Args:
            legs_ids_to_use (list, np.ndarray): List of legs' ids to use in pose calculation. Defaults to spider.LEGS_IDS.
            use_state_dict (bool, optional): Whether or not to read legs' positions from state dictionary. Defaults to True.
            legs_global_positions (List, np.ndarray): Array of legs' global positions. It has to be given, if use_state_dict is False. Defaults to None.

        Raises:
            ValueError: If use_state_dict is False and legs_global_positions is None.

        Returns:
            list: 1x3 array of roll, pitch and yaw values.
        """
        if not use_state_dict and legs_global_positions is None:
            raise ValueError("If legs positions are not read from state dictionary, they have to be passed as parameter.")
        if use_state_dict:
            _, _, legs_global_positions = self.json_file_manager.read_spider_state()
        with self.locker:
            q_a = self.q_a
        spider_pose = kin.get_spider_pose(legs_ids_to_use, legs_global_positions[legs_ids_to_use], q_a)
        rpy = spider_pose[3:]

        return rpy
        
    def __move_leg_pin_to_pin(self, leg: int, pin_to_pin_vector_in_local: np.ndarray, goal_pin_position: np.ndarray, rpy: list = None, use_prediction_model: bool = False) -> tuple[bool, np.ndarray]:
        """Move leg from current pin to the next one.

        Args:
            leg (int): Leg id.
            pin_to_pin_vector_in_local (np.ndarray): Vector from current to the next pin, given in leg's local origin.
            goal_pin_position (np.ndarray): Position of goal pin.

        Returns:
            (bool, np.ndarray): Movement success and leg's goal position in local origin. 
        """
        # Move leg and update spider state.
        with self.locker:
            x_a = self.x_a
        leg_goal_position_in_local = self.joints_velocity_controller.move_leg_async(
            leg,
            x_a,
            pin_to_pin_vector_in_local,
            config.LEG_ORIGIN,
            3,
            config.BEZIER_TRAJECTORY,
            use_prediction_model = use_prediction_model,
            rpy = rpy,
            is_offset = True,
        )
        # Wait 1 second for leg to start moving.
        if self.safety_kill_event.wait(timeout = 1.0):
            return (False, leg_goal_position_in_local)
        # Start microswitch safety-checking.
        #TODO: Check microswitch checking.
        with self.locker:
            self.moving_leg_id = leg

        self.json_file_manager.update_pins(leg, goal_pin_position)
        if self.safety_kill_event.wait(timeout = 3.0):
            with self.locker:
                self.moving_leg_id = None
            return (False, leg_goal_position_in_local)
        
        # Stop microswitch safety-checking.
        with self.locker:
            self.moving_leg_id = None

        return (True, leg_goal_position_in_local)
    
    def __select_water_pump_id(self, watering_leg_id: int) -> int:
        """Select id of water pump.

        Args:
            watering_leg_id (int): Id of leg that will be used for watering.

        Returns:
            int: Id of water pump.
        """
        if watering_leg_id == 1:
            return 1
        if watering_leg_id == 4:
            return 0
        return 2
    #endregion

#TODO: Create new entry point in separate file.
if __name__ == '__main__':
    app = App()
    time.sleep(1)
    # app.spider_states_manager(config.WORKING_STATE, False)
    app.test_prediction_model()
