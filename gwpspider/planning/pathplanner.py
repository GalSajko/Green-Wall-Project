"""Module for planning spider's path.
"""
import numpy as np
import math
import itertools
from gwpwall import wall

import spider
from calculations import mathtools as mt
from calculations import transformations as tf
from calculations import kinematics as kin
from calculations import dynamics as dyn


@property
def MAX_LINEAR_STEP():
    return 0.06

#region public methods
def calculate_spider_body_path(start_pose, goal_pose):
    """Calculate steps of spider's path. Path's segments are as follow:
    - lift spider on the walking height (if necessary),
    - walk towards the goal point,

    Args:
        start_pose (list): 1x4 array of starting pose, given as x, y, z, rotZ values in global origin, where rotZ is toration around global z axis. 
        goal_pose (_type_): 1x4 array of goal pose, given as x, y, z, rotZ values in global origin, where rotZ is toration around global z axis.

    Returns:
        numpy.ndarray: nx4 array of poses on each step of movenet, where n is number of steps.
    """
    path = [start_pose]

    # Move towards goal point.
    distance_to_travel = np.linalg.norm(np.array(goal_pose[:2]) - np.array(start_pose[:2]))
    if distance_to_travel != 0.0:
        number_of_steps = math.ceil(distance_to_travel / MAX_LINEAR_STEP) + 1
        linear_path = np.linspace(start_pose, goal_pose, number_of_steps)
        for pose in linear_path:
            if (pose - start_pose).any():
                path.append(pose)

    return np.array(path)

def calculate_selected_pins_over_max_y_distance(path):
    """Calculate legs positions in global origin, for each step of the spider's path. Legs should be as stretched as posible in gravity direction.

    Args:
        path (list): nx4 array of poses on each step of the path, where n is number of steps.

    Returns:
        numpy.ndarray: nx5x3 array of positions of selected pins on each step of the path.
    """
    pins = wall.createGrid(True)
    selected_pins = np.zeros((len(path), 5, 3))
    search_radius = 1.0
    
    def x_criterion(pin_position, leg_idx):
        x_distance = pin_position[0] - legs_bases_positions[leg_idx].flatten()[0]

        if leg_idx == upper_middle_leg:
            return 1 / (abs(pin[0] - pose[0]) + 10e-5)
        if leg_idx == upper_left_leg:
            if pin_position[0] == selected_middle_leg_pin[0]:
                return -1000
            return 0 if x_distance > 0.0 else 1 / (abs(x_distance) + 10e-5)
        if leg_idx == upper_right_leg:
            if pin_position[0] == selected_middle_leg_pin[0]:
                return -1000
            return 0 if x_distance < 0.0 else 1 / (abs(x_distance) + 10e-5)
        
        if leg_idx == lower_left_leg:
            return -100 if x_distance > 0.0 else 1 / (abs(x_distance) + 10e-5)
        if leg_idx == lower_right_leg:
            return -100 if x_distance < 0.0 else 1 / (abs(x_distance) + 10e-5)

    for step, pose in enumerate(path):
        pins_in_search_radius = pins[(np.sum(np.abs(pins - pose[:3])**2, axis = -1))**(0.5) < search_radius]
        T_GS = tf.xyzrpy_to_matrix(pose)
        legs_bases_positions = np.array([np.dot(T_GS, t)[:,3][:3] for t in spider.T_BASES])
        selected_pins_on_step = np.zeros((5, 3))

        upper_left_leg, upper_right_leg, upper_middle_leg, lower_left_leg, lower_right_leg = _get_legs_roles(legs_bases_positions, pose)

        selected_middle_leg_pin = None
        for idx, leg_base_position in enumerate(legs_bases_positions):
            potential_pins_for_leg = []
            for pin in pins_in_search_radius:
                if not (spider.CONSTRAINS[0] < np.linalg.norm(leg_base_position[:2] - pin[:2]) < spider.CONSTRAINS[1]):
                    continue
                rotated_ideal_leg_vector = np.dot(T_GS[:3,:3], np.append(spider.IDEAL_LEG_VECTORS[idx], 0))
                angle_between_ideal_vector_and_pin = mt.calculate_signed_angle_between_two_vectors(
                    rotated_ideal_leg_vector[:2],
                    np.array(np.array(pin) - np.array(leg_base_position))[:2])               
                if not (abs(angle_between_ideal_vector_and_pin) < spider.CONSTRAINS[2]):
                    continue
                y_amp = 5 if idx in (upper_left_leg, upper_middle_leg, upper_right_leg) else -1
                criterion = y_amp * (pin[1] - leg_base_position[1]) + \
                     (1 / (abs(leg_base_position[0] - pin[0]) + 10e-5)) + x_criterion(pin, idx)

                potential_pins_for_leg.append([pin, criterion])

            potential_pins_for_leg = np.array(potential_pins_for_leg, dtype = np.object)
            selected_pins_on_step[idx] = potential_pins_for_leg[potential_pins_for_leg[:,1].argmax()][0]
            if idx == 0:
                selected_middle_leg_pin = selected_pins_on_step[idx]

        selected_pins[step] = selected_pins_on_step

    return selected_pins

def create_walking_instructions(start_pose, goal_pose, pins_selection_method = calculate_selected_pins_over_max_y_distance):
    """Generate walking instructions to move spider from start to goal pose on the wall.

    Args:
        start_pose (list): 1x4 array of start pose, given as x, y, z and yaw values in global origin.
        goal_pose (lsit): 1x4 array of goal pose, given as x, y, z and yaw values in global origin.
        pins_selection_method (function, optional): Method used to calculate selected pins along the way. Defaults to calculate_selected_pins_over_max_y_distance.

    Returns:
        tuple: Lists of poses and pins instructions. Pin instruction consists of leg id and pin's position.
    """
    start_pose = np.array(start_pose)
    goal_pose = np.array(goal_pose)

    path = calculate_spider_body_path(start_pose, goal_pose)
    selected_pins = pins_selection_method(path)
    poses = [np.array(start_pose)]
    pins_instructions = [np.c_[spider.LEGS_IDS, np.array(selected_pins[0])]]

    moving_direction = math.atan2(goal_pose[0] - start_pose[0], goal_pose[1] - start_pose[1])
    legs_moving_order = np.array([4, 3, 0, 2, 1]) if moving_direction >= 0.0 else np.array([1, 2, 0, 3, 4])
    if moving_direction == math.pi:
        legs_moving_order = np.array([0, 1, 4, 2, 3])
    elif moving_direction == 0.0:
        legs_moving_order = np.array([2, 3, 1, 4, 0])
    legs_moving_order = legs_moving_order.astype(int)

    def append_to_pose_and_pins_instructions(i, pins):
        poses.append(path[i])
        pins_instructions.append(pins[legs_moving_order])
        pins_instructions[-1] = np.c_[legs_moving_order, pins_instructions[-1]]

    for idx, pins in enumerate(selected_pins):
        if idx == 0:
            pins_instructions[-1] = pins_instructions[-1][legs_moving_order]
            continue
        if (np.array(pins) - np.array(selected_pins[idx - 1])).any() or idx == (len(selected_pins) - 1):
            append_to_pose_and_pins_instructions(idx, pins)

    return np.array(poses), np.array(pins_instructions)

def modified_walking_instructions(start_legs_positions, goal_pose):
    """Change first pose and first set of pins instructions with given values.

    Args:
        start_legs_positions (list): 5x3 array of legs positions.
        goal_pose (list): 1x4 array of goal pose, given as x, y, z and yaw values in global origin.

    Returns:
        tuple: Modified poses and pins instructions.
    """
    # Create new start pose as mean value of all current legs positions (from json file).
    # Adjust the pose to avoid over-extension of any leg if necessary.
    start_pose = np.mean(start_legs_positions, axis = 0)
    start_pose[2] = spider.SPIDER_WALKING_HEIGHT
    start_pose = np.append(start_pose, 0.0)
    correction_offset = 0.005
    print("CALCULATING INITIAL POSE...")
    counter = 0
    while True:
        potential_legs_lengths = np.zeros(len(spider.LEGS_IDS), dtype = np.float32)
        for leg in spider.LEGS_IDS:
            leg_base_position_in_global = np.dot(tf.xyzrpy_to_matrix(start_pose), spider.T_BASES[leg])[:, 3][:3]
            potential_legs_lengths[leg] = np.linalg.norm(start_legs_positions[leg] - leg_base_position_in_global)
        if (not (potential_legs_lengths > spider.LEG_LENGTH_MAX_LIMIT).any()) or counter > 100:
            if (potential_legs_lengths > 0.6).any():
                print("CALCULATED INITIAL POSE WOULD COSE OVER-EXTENSION OF AT LEAST ONE LEG. CHANGE LEGS' POSITIONS IN A WAY THAT WOULD NOT COUSE OVER-EXTENSION. SAFETY KILLING A PROGRAM...")
                return False
            break
        over_extended_legs_ids = np.where(potential_legs_lengths > spider.LEG_LENGTH_MAX_LIMIT)[0]
        if 2 in over_extended_legs_ids or 3 in over_extended_legs_ids:
            start_pose[1] -= correction_offset
        else:
            start_pose[1] += correction_offset
        counter += 1

    # Create path from start pose to goal pose. Prepend modified pins instructions for first step, where legs are not on same positions as calculated by path planner.
    poses, pins_instructions = create_walking_instructions(start_pose, goal_pose)
    legs_moving_order = np.array(pins_instructions[0, :, 0]).astype(int)

    modified_poses_shape = list(poses.shape)
    modified_poses_shape[0] += 1
    modified_poses = np.zeros(modified_poses_shape)
    modified_poses[0] = start_pose
    modified_poses[1:] = poses

    modified_pins_instructions_shape = list(pins_instructions.shape)
    modified_pins_instructions_shape[0] += 1
    modified_pins_instructions = np.zeros(modified_pins_instructions_shape)

    start_pins_instructions = np.zeros_like(pins_instructions[0])
    start_pins_instructions[:, 0] = legs_moving_order
    start_pins_instructions[:, 1:] = start_legs_positions[legs_moving_order]

    modified_pins_instructions[0] = start_pins_instructions 
    modified_pins_instructions[1:] = pins_instructions

    return modified_poses, modified_pins_instructions
#endregion

#region private methods
def _get_legs_roles(legs_bases_positions, pose):
    """Define legs roles, based on their positions - upper/lower, right/left or middle.

    Args:
        legs_bases_positions (list): 5x3 array of positions of legs' anchors in global origin.
        pose (list): Array of spider's pose in global origin.

    Returns:
        tuple: Indexes of upper-left, upper-right, upper-middle, lower-left and lower-right legs.
    """
    upper_legs = np.where(legs_bases_positions[:,1] > pose[1])[0]
    upper_left_leg_id = np.where(legs_bases_positions[upper_legs,0] == legs_bases_positions[upper_legs,0].min())[0]
    upper_right_leg_id = np.where(legs_bases_positions[upper_legs,0] == legs_bases_positions[upper_legs,0].max())[0]
    upper_left_leg = upper_legs[upper_left_leg_id]
    upper_right_leg = upper_legs[upper_right_leg_id]
    upper_middle_leg = upper_legs[np.delete(upper_legs, [upper_left_leg_id, upper_right_leg_id])]


    lower_legs = np.delete(spider.LEGS_IDS, upper_legs)
    lower_left_leg = lower_legs[np.where(legs_bases_positions[lower_legs, 0] == legs_bases_positions[lower_legs, 0].min())[0]]
    lower_right_leg = lower_legs[np.where(legs_bases_positions[lower_legs, 0] == legs_bases_positions[lower_legs, 0].max())[0]]

    return upper_left_leg, upper_right_leg, upper_middle_leg, lower_left_leg, lower_right_leg
#endregion