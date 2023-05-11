import numpy as np
import threading
import time
import os

import config
import controllers
from utils import threadmanager
from utils import jsonfilemanager
from utils import csvfilemanager
from periphery import dynamixel as dmx
from periphery import waterpumpsbno
from environment.comunication import CommunicationWithServer
from environment import spider
from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import mathtools
from calculations import transformations as tf
from planning import pathplanner

class App:
    def __init__(self):
        self.q_a = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.x_a = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.tau_a = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.hw_errors = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.temperatures = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.dq_c = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)

        self.joints_velocity_controller = controllers.VelocityController()
        self.motor_driver = dmx.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]])
        self.pumps_bno_arduino = waterpumpsbno.PumpsBnoArduino()
        self.thread_manager = threadmanager.CustomThread()
        self.json_file_manager = jsonfilemanager.JsonFileManager()
        self.csv_file_manager = csvfilemanager.CsvFileManager()
        self.server_comm = CommunicationWithServer(self.json_file_manager.FILENAME)

        self.states_object_locker = threading.Lock()
        self.safety_kill_event = threading.Event()

        self.current_state = None
        self.watering_counter = 0

        self.was_in_resting_state = False
        self.last_plant_or_refill_position = None

        self.init_bno = True

        self.__init_layers()

    def safety_layer(self):
        """Continuously checking for hardware errors on the motors. If error occurs, inititate resting procedure and then continue with working. 
        """
        def safety_checking(kill_event):
            self.safety_kill_event.clear()
            while True:
                if kill_event.is_set():
                    break
                with self.states_object_locker:
                    hw_errors = self.hw_errors

                is_hw_error = hw_errors.any() and self.current_state == config.WORKING_STATE

                if is_hw_error:
                    self.safety_kill_event.set()
                    # Wait for working thread to stop.
                    if config.WORKING_THREAD_NAME in self.spider_state_thread.name:
                        self.spider_state_thread.join()
                        print("WORKING STOPED.")

                    # Start transition state and wait for it to finish.
                    self.spider_states_manager(config.TRANSITION_STATE)
                    if config.TRANSITION_THREAD_NAME in self.spider_state_thread.name:
                        self.spider_state_thread.join()
                        print("TRANSITION FINISHED")

                    # Start resting state and wait for it to finish.
                    self.spider_states_manager(config.RESTING_STATE)
                    if config.RESTING_THREAD_NAME in self.spider_state_thread.name:
                        self.spider_state_thread.join()
                        print("RESTING FINISHED")
                    
                    self.was_in_resting_state = True

                    self.safety_kill_event.clear()
                    time.sleep(1)

                    # Continue with working.
                    self.spider_states_manager(config.WORKING_STATE)

                time.sleep(0.1)
        self.safety_thread, self.safety_thread_kill_event = self.thread_manager.run(safety_checking, config.SAFETY_THREAD_NAME, False, True, do_print = True)
    
    def motor_control_layer(self):
        """Motors control layer with reading, recalculating and sending data to the motors.
        """
        def control_loop(kill_event):
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
                tau, f = dyn.get_torques_and_forces_on_legs_tips(q_a, i_a, self.pumps_bno_arduino.getGravityVector())
                tau_mean, tau_buffer, tau_counter = mathtools.running_average(tau_buffer, tau_counter, tau)
                force_mean, force_buffer, force_counter = mathtools.running_average(force_buffer, force_counter, f)

                with self.states_object_locker:
                    self.q_a = q_a
                    self.hw_errors = hw_errors
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
                while elapsed_time < (1 / config.CONTROLLER_FREQUENCY):
                    elapsed_time = time.perf_counter() - start_time
                    time.sleep(0)

        self.motor_control_thread, self.motor_control_thread_kill_event = self.thread_manager.run(control_loop, config.CONTROL_THREAD_NAME, False, True, do_print = True)

    def spider_states_manager(self, state):
        """Managing spider's state.

        Args:
            state (string): Name of desired state.
            workingArs (tuple, optional): Neede parameters for desired state. Defaults to None.
        """
        if state == config.WORKING_STATE:
            self.spider_state_thread = self.thread_manager.run(self.working, config.WORKING_THREAD_NAME, True, True, False, False)
        elif state == config.RESTING_STATE:
            self.spider_state_thread, self.spider_states_thread_kill_event = self.thread_manager.run(self.rest, config.RESTING_THREAD_NAME, True, True, True, False)
        elif state == config.TRANSITION_STATE:
            self.spider_state_thread = self.thread_manager.run(self.transition_to_rest_state, config.TRANSITION_THREAD_NAME, True, True, False, False)
    
    def working(self):
        """Working procedure, includes walking and watering the plants.
        """
        self.current_state = config.WORKING_STATE
        is_init = True
        
        print("WORKING...") 
        
        while True:
            spider_pose, _, start_legs_positions = self.json_file_manager.read_spider_state()
            do_refill_water_tank = self.watering_counter >= config.NUMBER_OF_WATERING_BEFORE_REFILL
            while True:
                try:
                    if do_refill_water_tank:
                        watering_leg_id, goal_pose = tf.get_watering_leg_and_pose(spider_pose, do_refill = True)
                        plant_or_refill_position = goal_pose[:3] + spider.REFILLING_LEG_OFFSET
                        print("GOING TO REFILL POSITION.")
                        self.server_comm.post_refilling()
                    else:
                        if not self.was_in_resting_state:
                            plant_or_refill_position = self.server_comm.getGoalPos()
                            self.last_plant_or_refill_position = plant_or_refill_position
                        else:
                            plant_or_refill_position = self.last_plant_or_refill_position
                            self.was_in_resting_state = False
                        print(f"PLANT POSITION {plant_or_refill_position}.")
                        watering_leg_id, goal_pose = tf.get_watering_leg_and_pose(spider_pose, plant_or_refill_position)
                    if is_init:
                        planning_result = pathplanner.modified_walking_instructions(start_legs_positions, goal_pose)
                        if not planning_result:
                            os._exit(0)
                        else:
                            poses, pins_instructions = planning_result
                        is_init = False
                        break
                        
                    poses, pins_instructions = pathplanner.create_walking_instructions(spider_pose, goal_pose)
                    break
                except Exception as e:
                    print("Error in path planning. Trying again... ")
                    print(f"EXCEPTION {e}")
                time.sleep(0.05)

            print(f"NEW GOAL POINT {goal_pose[:3]}.")
            for step, pose in enumerate(poses):
                current_pins_positions = pins_instructions[step, :, 1:]
                current_legs_moving_order = pins_instructions[step, :, 0].astype(int)
                with self.states_object_locker:
                    x_a = self.x_a

                if step == 0:
                    self.joints_velocity_controller.move_legs_sync(current_legs_moving_order, x_a, current_pins_positions, config.GLOBAL_ORIGIN, 5, config.MINJERK_TRAJECTORY, pose)
                    self.json_file_manager.update_whole_dict(pose, current_pins_positions, current_legs_moving_order)
                    if self.safety_kill_event.wait(timeout = 5.5):
                        return

                    if self.init_bno:
                        self.pumps_bno_arduino.resetBno()
                        self.init_bno = False
                        if self.safety_kill_event.wait(timeout = 1.0):
                            return
                    self.__distribute_forces(spider.LEGS_IDS, config.FORCE_DISTRIBUTION_DURATION)
                    continue

                previous_pins_positions = np.array(pins_instructions[step - 1, :, 1:])
                self.joints_velocity_controller.move_legs_sync(current_legs_moving_order, x_a, previous_pins_positions, config.GLOBAL_ORIGIN, 2.5, config.MINJERK_TRAJECTORY, pose)
                self.json_file_manager.update_whole_dict(pose, previous_pins_positions, current_legs_moving_order)
                if self.safety_kill_event.wait(timeout = 3.0):
                    print("UNSUCCESSFULL BODY MOVEMENT.")
                    return

                previous_to_current_pins_offsets = current_pins_positions - previous_pins_positions
                for idx, leg in enumerate(current_legs_moving_order):
                    if previous_to_current_pins_offsets[idx].any():
                        movement_success = self.__pin_to_pin_leg_movement(leg, previous_pins_positions[idx], current_pins_positions[idx], pose)
                        if not movement_success:
                            print("UNSUCCESSFULL PIN TO PIN MOVEMENT.")
                            return
            
            watering_success = self.__watering(watering_leg_id, plant_or_refill_position, pose, do_refill_water_tank)
            if not watering_success:
                return
        
    def rest(self, kill_event):
        """Resting procedure, includes option for manually correcting non-attached leg. Resting lasts until temperatures of all motors
        drop below working temperature.

        Args:
            kill_event (event): Event for killing a procedure.
        """
        print("RESTING...")
        self.current_state = config.RESTING_STATE

        with self.states_object_locker:
            hw_errors = self.hw_errors

        attached_legs = self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs()
        motors_in_error = self.motor_driver.motors_ids[np.where(hw_errors != 0)]
        print("REBOOTING MOTORS WITH IDS: ", motors_in_error, "...")
        self.motor_driver.reboot_motors(motors_in_error)
        while hw_errors.any():
            with self.states_object_locker:
                hw_errors = self.hw_errors
            if kill_event.wait(timeout = 0.1):
                return

        unattached_legs = np.setdiff1d(spider.LEGS_IDS, attached_legs)
        while unattached_legs.size > 1:
            print("MORE THAN ONE UNATTACHED LEG: ", unattached_legs)
            unattached_legs = np.setdiff1d(spider.LEGS_IDS, self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs())
            if kill_event.wait(timeout = 0.1):
                return

        if unattached_legs.size == 1:
            self.__manual_correction(unattached_legs[0], use_safety = False)   

        # Rest until temperature drops below working temperature.
        print("WAITING ON TEMPERATURES TO DROP BELOW WORKING TEMPERATURE...")
        while True:
            with self.states_object_locker:
                temperatures = self.temperatures
            if kill_event.wait(timeout = 1.0) or (temperatures < self.motor_driver.MAX_WORKING_TEMPERATURE).all():
                break
        if kill_event.is_set():
            return
        
        # Check for possible singularity of lower legs.
        _, _, legs_global_positions = self.json_file_manager.read_spider_state()
        if np.linalg.norm(legs_global_positions[2] - legs_global_positions[3]) <= 0.22:
            print("CAUTION! POSSIBLE SINGULARITY LOCK - CHECK BEFORE CONTINUING!!!")
            self.motor_driver.enable_disable_legs(config.DISABLE_LEGS_COMMAND)
            confirm_input = input("PRESS ENTER TO CONTINUE OR K + ENTER TO KILL A PROGRAM.")
            if confirm_input == config.PROGRAM_KILL_KEY:
                print("KILLING A PROGRAM...")
                os._exit(0)

        # Update last positions and enable torques in motors.
        with self.states_object_locker:
            x_a = self.x_a
        self.joints_velocity_controller.update_last_legs_positions(x_a)
        if kill_event.wait(timeout = 1.0):
            return
        self.motor_driver.enable_disable_legs(config.ENABLE_LEGS_COMMAND)
    
    def transition_to_rest_state(self):
        """Procedure for transition between working and resting state, by using force mode. 
        """
        print("TRANSITION INTO RESTING STATE...")
        self.current_state = config.TRANSITION_STATE
        self.joints_velocity_controller.clear_instruction_queues()
        f_d = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.joints_velocity_controller.start_force_mode(spider.LEGS_IDS, f_d)
        time.sleep(10)
        self.joints_velocity_controller.stop_force_mode()

    # region private methods
    def __init_layers(self):
        self.safety_layer()
        time.sleep(2)
        self.motor_control_layer()
    
    def __distribute_forces(self, legs_ids, duration):
        """Run force distribution process in a loop for a given duration.
        """
        offload_leg_id = np.setdiff1d(spider.LEGS_IDS, legs_ids)
        if len(offload_leg_id) > 1:
            print("Cannot offload more than one leg at the same time.")
            return False

        start_time = time.perf_counter()
        elapsed_time = 0
        while elapsed_time < duration:
            with self.states_object_locker:
                tau_a = self.tau_a
                q_a = self.q_a
            distributed_forces = dyn.calculate_distributed_forces(tau_a, q_a, legs_ids, offload_leg_id)

            if len(offload_leg_id):
                distributed_forces = np.insert(distributed_forces, offload_leg_id[0], np.zeros(3, dtype = np.float32), axis = 0)
            self.joints_velocity_controller.start_force_mode(spider.LEGS_IDS, distributed_forces)

            elapsed_time = time.perf_counter() - start_time
            if self.safety_kill_event.wait(timeout = self.joints_velocity_controller.period):
                break
        
        self.joints_velocity_controller.stop_force_mode()
        time.sleep(2.0)
    
    def __pin_to_pin_leg_movement(self, leg, current_pin_position, goal_pin_position, pose):
        # Read legs positions before movement.
        with self.states_object_locker:
            x_a_before = self.x_a
        leg_movement_data = x_a_before.flatten()

        # Distribute forces among other legs.
        other_legs = np.delete(spider.LEGS_IDS, leg)
        self.__distribute_forces(other_legs, config.FORCE_DISTRIBUTION_DURATION)

        with self.states_object_locker:
            q_a_leg = self.q_a[leg]
        last_joint_position_in_local = kin.leg_base_to_third_joint_forward_kinematics(q_a_leg)[:,3][:3]
        last_joint_to_goal_pin_in_spider_unit = tf.get_last_joint_to_goal_pin_vector_in_spider(leg, last_joint_position_in_local, goal_pin_position, pose)
        
        self.joints_velocity_controller.start_force_mode([leg], [last_joint_to_goal_pin_in_spider_unit * 2.5])
        if self.safety_kill_event.wait(timeout = 1.0):
            self.joints_velocity_controller.stop_force_mode()
            return False

        self.joints_velocity_controller.grippers_arduino.move_gripper(leg, self.joints_velocity_controller.grippers_arduino.OPEN_COMMAND)
        if self.safety_kill_event.wait(timeout = 3.5):
            self.joints_velocity_controller.stop_force_mode()
            return False
        if leg in self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs():
            print(f"GRIPPER {leg} DID NOT OPEN.")
            self.joints_velocity_controller.stop_force_mode()
            return

        # Read spider's rpy after releasing the leg.
        _, _, legs_global_positions = self.json_file_manager.read_spider_state()
        with self.states_object_locker:
            q_a = self.q_a
        spider_pose = kin.get_spider_pose(other_legs, legs_global_positions[other_legs], q_a)
        rpy = spider_pose[3:]
        rpy_bno = self.pumps_bno_arduino.getRpy()

        leg_movement_data = np.append(leg_movement_data, rpy_bno)
        leg_movement_data = np.append(leg_movement_data, rpy)
        leg_movement_data = np.append(leg_movement_data, leg)

        pin_to_pin_vector_in_local, leg_base_orientation_in_global = tf.get_pin_to_pin_vector_in_local(leg, rpy, current_pin_position, goal_pin_position)
        global_z_direction_in_local = np.dot(leg_base_orientation_in_global, np.array([0.0, 0.0, 1.0], dtype = np.float32))

        self.joints_velocity_controller.stop_force_mode()
        if self.safety_kill_event.wait(timeout = 1.0):
            return False
        
        # Move leg and update spider state.
        with self.states_object_locker:
            x_a_leg = self.x_a[leg]
        leg_goal_position_in_local = self.joints_velocity_controller.move_leg_async(leg, x_a_leg, pin_to_pin_vector_in_local, config.LEG_ORIGIN, 3, config.BEZIER_TRAJECTORY, is_offset = True)
        leg_movement_data = np.append(leg_movement_data, leg_goal_position_in_local.flatten())
        self.json_file_manager.update_pins(leg, goal_pin_position)
        if self.safety_kill_event.wait(timeout = 4.0):
            return False

        # Before closing the gripper, put leg in force mode to avoid pulling the spider with gripper.
        self.joints_velocity_controller.start_force_mode(np.array([leg]), np.array([np.zeros(3, dtype = np.float32)]))
        self.joints_velocity_controller.grippers_arduino.move_gripper(leg, self.joints_velocity_controller.grippers_arduino.CLOSE_COMMAND)
        if self.safety_kill_event.wait(timeout = 3.0):
            self.joints_velocity_controller.stop_force_mode()
            return False
        self.joints_velocity_controller.stop_force_mode()

        number_of_tries = 1
        # Correction in case of missed pin.
        if leg not in self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs():
            correction_success, number_of_corrections = self.__correction(leg, global_z_direction_in_local, leg_goal_position_in_local)
            print("CORRECTION SUCCESS: ", correction_success)
            if not correction_success:
                return False
            number_of_tries += number_of_corrections

        leg_movement_data = np.append(leg_movement_data, number_of_tries)

        with self.states_object_locker:
            x_a_after = self.x_a
        leg_movement_data = np.append(leg_movement_data, x_a_after.flatten())

        self.csv_file_manager.writeRow(np.round(leg_movement_data, 4))

        return True
    
    def __correction(self, leg_id, global_z_direction_in_local, leg_goal_position_in_local):
        print(f"LEG {leg_id} IS NOT ATTACHED.")
        auto_correction_success, number_of_corrections = self.__automatic_correction(leg_id, global_z_direction_in_local, leg_goal_position_in_local)
        print("AUTO CORRECTION SUCCESS: ", auto_correction_success)
        if auto_correction_success:
            return (auto_correction_success, number_of_corrections)
        manual_corection_success = self.__manual_correction(leg_id)
        print("MANUAL CORRECTION SUCCESS: ", manual_corection_success)
        return (manual_corection_success, 10)

    def __automatic_correction(self, leg_id, global_z_direction_in_local, leg_goal_position_in_local):
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
 
            with self.states_object_locker:
                x_a_leg = self.x_a[leg_id]
            self.joints_velocity_controller.move_leg_async(leg_id, x_a_leg, detach_position, config.LEG_ORIGIN, 1, config.MINJERK_TRAJECTORY)
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

    def __manual_correction(self, leg_id, use_safety = True):
        _, used_pins_ids, legs_global_positions = self.json_file_manager.read_spider_state()
        goal_pin_id = used_pins_ids[leg_id]

        print(f"MANUALLY CORRECT LEG {leg_id} ON PIN {goal_pin_id}.")
        self.joints_velocity_controller.grippers_arduino.move_gripper(leg_id, self.joints_velocity_controller.grippers_arduino.OPEN_COMMAND)
        with self.states_object_locker:
            x_a_leg = self.x_a[leg_id]
            q_a = self.q_a

        attached_legs = self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs()

        print("ATTACHED LEGS: ", attached_legs)
        goal_pin_in_local = kin.get_goal_pin_in_local(leg_id, attached_legs, legs_global_positions, q_a, self.pumps_bno_arduino.getRpy())

        distance = np.linalg.norm(goal_pin_in_local - x_a_leg)
        disable_legs = not (spider.LEG_LENGTH_MIN_LIMIT < np.linalg.norm(goal_pin_in_local) < spider.LEG_LENGTH_MAX_LIMIT)
        if disable_legs:
            print(f"DISABLE LEGS: {attached_legs}.")
            self.motor_driver.enable_disable_legs(attached_legs, config.DISABLE_LEGS_COMMAND)

        self.joints_velocity_controller.start_force_mode([leg_id], [np.zeros(3, dtype = np.float32)])
        while True:
            with self.states_object_locker:
                x_a_leg = self.x_a[leg_id]
                if disable_legs:
                    q_a = self.q_a
            
            if disable_legs:
                goal_pin_in_local = kin.get_goal_pin_in_local(leg_id, attached_legs, legs_global_positions, q_a, self.pumps_bno_arduino.getRpy())

            distance = np.linalg.norm(goal_pin_in_local - x_a_leg)
            switch_state = int(self.joints_velocity_controller.grippers_arduino.getSwitchesStates()[leg_id])

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

    def __watering(self, watering_leg_id, plant_position, spider_pose, do_refill): 
        self.__distribute_forces(np.delete(spider.LEGS_IDS, watering_leg_id), config.FORCE_DISTRIBUTION_DURATION)
        with self.states_object_locker:
            x_a_leg_before_watering = self.x_a[watering_leg_id]

        # Move leg on watering position.
        _, _, legs_global_positions = self.json_file_manager.read_spider_state()
        with self.states_object_locker:
            q_a = self.q_a
        spider_pose = kin.get_spider_pose(spider.LEGS_IDS, legs_global_positions, q_a)

        self.joints_velocity_controller.grippers_arduino.move_gripper(watering_leg_id, self.joints_velocity_controller.grippers_arduino.OPEN_COMMAND)
        if self.safety_kill_event.wait(timeout = 1.0):
            if not do_refill:
                return False

        self.joints_velocity_controller.move_leg_async(watering_leg_id, x_a_leg_before_watering, plant_position, config.GLOBAL_ORIGIN, 3, config.BEZIER_TRAJECTORY, spider_pose)
        if self.safety_kill_event.wait(timeout = 3.5):
            if not do_refill:
                return False
        
        # Close gripper to allow refilling.
        if do_refill:
            self.joints_velocity_controller.grippers_arduino.move_gripper(watering_leg_id, self.joints_velocity_controller.grippers_arduino.CLOSE_COMMAND)

        # Turn on water pump.
        if watering_leg_id == 1:
            pump_id = 1
        elif watering_leg_id == 4:
            pump_id = 0
        else:
            pump_id = 2

        pumping_time = config.REFILL_TIME if do_refill else config.WATERING_TIME

        print(f"PUMP {pump_id} ON.")
        start_time = time.perf_counter()
        while True:
            elapsed_time = time.perf_counter() - start_time
            self.pumps_bno_arduino.water_pump_controll(self.pumps_bno_arduino.PUMP_ON_COMMAND, pump_id)
            if elapsed_time > pumping_time:
                if not do_refill:
                    self.pumps_bno_arduino.water_pump_controll(self.pumps_bno_arduino.PUMP_OFF_COMMAND, pump_id)
                    print(f"PUMP {pump_id} OFF.")
                else:
                    self.joints_velocity_controller.grippers_arduino.move_gripper(watering_leg_id, self.joints_velocity_controller.grippers_arduino.OPEN_COMMAND)
                break
            if not do_refill:
                if self.safety_kill_event.wait(timeout = 0.05):
                    self.pumps_bno_arduino.water_pump_controll(self.pumps_bno_arduino.PUMP_OFF_COMMAND, pump_id)
                    print(f"PUMP {pump_id} OFF.")
                    return False
            else:
                time.sleep(0.05)
        
        # Move leg back on starting position.
        with self.states_object_locker:
            x_a_leg_after_watering = self.x_a[watering_leg_id]

        self.joints_velocity_controller.move_leg_async(watering_leg_id, x_a_leg_after_watering, x_a_leg_before_watering, config.LEG_ORIGIN, 3, config.BEZIER_TRAJECTORY)
        if self.safety_kill_event.wait(timeout = 3.5):
            if not do_refill:
                return False
            
        if do_refill:
            self.pumps_bno_arduino.water_pump_controll(self.pumps_bno_arduino.PUMP_OFF_COMMAND, pump_id)
            print(f"PUMP {pump_id} OFF.")
            self.watering_counter = 0
            self.server_comm.postStop()
        else:
            self.watering_counter += 1
            
        self.joints_velocity_controller.grippers_arduino.move_gripper(watering_leg_id, self.joints_velocity_controller.grippers_arduino.CLOSE_COMMAND)
        if self.safety_kill_event.wait(1.0):
            if not do_refill:
                return False
        
        # Correct if leg does not reach starting pin successfully.
        if watering_leg_id not in self.joints_velocity_controller.grippers_arduino.get_ids_of_attached_legs():
            rpy = self.pumps_bno_arduino.getRpy()
            spider_orientation_in_global = tf.xyzrpy_to_matrix(rpy, True)
            leg_base_orientation_in_global = np.linalg.inv(np.dot(spider_orientation_in_global, spider.T_ANCHORS[watering_leg_id][:3, :3]))
            global_z_direction_in_local = np.dot(leg_base_orientation_in_global, np.array([0.0, 0.0, 1.0], dtype = np.float32))

            return self.__correction(watering_leg_id, global_z_direction_in_local, x_a_leg_before_watering)

        return True
    #endregion

if __name__ == '__main__':
    app = App()
    time.sleep(1)
    app.spider_states_manager(config.WORKING_STATE)
