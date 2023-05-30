"""Module for calculating transformations between different origins.
"""
import numpy as np
import math
from gwpconfig import wall

import config
import spider

def xyzrpy_to_matrix(xyzrpy: list, rotation_only: bool = False) -> np.ndarray:
    """Calculate global transformation matrix for global origin - spider relation.

    Args:
        xyzrpy (list): Global spider's pose to be transformed into matrix. Could be given as 1x4 array, representing xyzy values or 
        1x6 array, representing xyzrpy values.
        rotation_only (bool): If True, calculate 3x3 rotation matrix from given rpy angles, otherwise calculate 4x4 full transformation matrix. Defaults to False.

    Returns:
        np.ndarray: 4x4 transformation matrix from global origin to spider or 3x3 rotation matrix from given rpy angles.
    """
    if rotation_only:
        if len(xyzrpy) == 3:
            roll_angle, pitch_angle, yaw_angle = xyzrpy
        else:
            raise ValueError(f"Length of xyzrpy parameter should be 3, but it is {len(xyzrpy)}.")
    else:
        if len(xyzrpy) == 4:
            xyzrpy = [xyzrpy[0], xyzrpy[1], xyzrpy[2], 0, 0, xyzrpy[3]]
        position = xyzrpy[:3]
        roll_angle, pitch_angle, yaw_angle = xyzrpy[3:]

    roll = np.array([
        [math.cos(roll_angle), 0, math.sin(roll_angle)],
        [0, 1, 0],
        [-math.sin(roll_angle), 0, math.cos(roll_angle)]
    ], dtype = np.float32)
    pitch = np.array([
        [1, 0, 0],
        [0, math.cos(pitch_angle), -math.sin(pitch_angle)],
        [0, math.sin(pitch_angle), math.cos(pitch_angle)]
    ], dtype = np.float32)
    yaw = np.array([
        [math.cos(yaw_angle), -math.sin(yaw_angle), 0],
        [math.sin(yaw_angle), math.cos(yaw_angle), 0],
        [0, 0, 1]
    ], dtype = np.float32)
    rotation_matrix = np.dot(roll, np.dot(pitch, yaw))

    if not rotation_only:
        transform_matrix = np.c_[rotation_matrix, position]
        transform_matrix = np.r_[transform_matrix, [np.array([0, 0, 0, 1], dtype = np.float32)]]
        
        return transform_matrix
    
    return rotation_matrix

def get_pin_to_pin_vector_in_local(leg_id: int, rpy: list, current_pin_position: np.ndarray, goal_pin_position: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Calculate pin-to-pin vector in leg's local origin.

    Args:
        leg_id (int): Leg id.
        rpy (list): 1x3 array of roll, pitch and yaw values of spider's orientation.
        current_pin_position (np.ndarray): 1x3 array of current pin's position in global origin
        goal_pin_position (np.ndarray): 1x3 array of goal pin's position in global origin.

    Returns:
        tuple: 1x3 pin-to-pin vector in leg's local origin and 3x3 orientation matrix of leg's anchor in global origin.
    """
    spider_rotation_in_global = xyzrpy_to_matrix(rpy, True)
    leg_base_orientation_in_global = np.linalg.inv(np.dot(spider_rotation_in_global, spider.T_BASES[leg_id][:3, :3]))
    pin_to_pin_in_global = goal_pin_position - current_pin_position
    pin_to_pin_in_local = np.dot(leg_base_orientation_in_global, pin_to_pin_in_global)

    return pin_to_pin_in_local, leg_base_orientation_in_global

def get_leg_position_in_local(leg_id: int, leg_position_in_global: np.ndarray, spider_pose: np.ndarray) -> np.ndarray:
    """Calculate local leg's position from given global position.

    Args:
        leg_id (int): Leg id.
        leg_position_in_global (np.ndarray): Global position of leg.
        spider_pose (np.ndarray): Global spider's pose. Could be given as 1x4 array, representing xyzy values or 1x6 array, representing xyzrpy values.

    Returns:
        np.ndarray: 1x3 array of with x, y and z leg's positions in leg-local origin.
    """
    T_GS = xyzrpy_to_matrix(spider_pose)
    T_GA = np.dot(T_GS, spider.T_BASES[leg_id])
    leg_position_in_global = np.append(leg_position_in_global, 1)

    return np.dot(np.linalg.inv(T_GA), leg_position_in_global)[:3]

def get_global_direction_in_local(leg_id: int, spider_pose: np.ndarray, global_direction: np.ndarray) -> np.ndarray:
    """Transform vector given in global origin into local.

    Args:
        leg_id (int): Leg id.
        spider_pose (np.ndarray): Spider's pose.
        global_direction (np.ndarray): Vector in global origin.

    Returns:
        np.ndarray: Vector in local origin.
    """
    T_GS = xyzrpy_to_matrix(spider_pose)
    T_GA = np.dot(T_GS, spider.T_BASES[leg_id])[:3,:3]
    local_direction = np.dot(np.linalg.inv(T_GA), global_direction)

    return local_direction

def convert_in_local_goal_positions(
    leg_id: int,
    leg_current_position_in_local: np.ndarray,
    leg_goal_position_or_offset: np.ndarray,
    origin: str,
    is_offset: bool,
    spider_pose: np.ndarray) -> np.ndarray:
    """Transform given leg's goal position into local origin.

    Args:
        leg_id (int): Leg id.
        leg_current_position_in_local (np.ndarray): 1x3 array of current leg's position, given in local origin
        leg_goal_position_or_offset (np.ndarray): 1x3 array of leg's goal position given as absolute position or offset. 
        origin (str): Origin that goal position or offset is given in.
        is_offset (bool): If True, goal position is given as an offset, otherwise as an absolute position.
        spider_pose (np.ndarray): Spider's pose given in global origin.

    Returns:
        np.ndarray: 1x3 array of leg's goal position, given in local origin.
    """
    if origin == config.LEG_ORIGIN:
        leg_goal_position_in_local = np.copy(leg_goal_position_or_offset)
        if is_offset:
            leg_goal_position_in_local += leg_current_position_in_local
        return leg_goal_position_in_local
    if not is_offset:
        return get_leg_position_in_local(leg_id, leg_goal_position_or_offset, spider_pose)
    return np.array(leg_current_position_in_local + get_global_direction_in_local(leg_id, spider_pose, leg_goal_position_or_offset), dtype = np.float32)

def transform_vector(vector: np.ndarray, given_in_origin: str, transform_to_origin: str, leg_id: int) -> np.ndarray:
    """Transform (x, y, z) vector between spider's and leg's origins.

    Args:
        vector (np.ndarray): Vector.
        given_in_origin (str): Origin that vector is given in.
        transform_to_origin (str): Origin that vector should be transformed into.
        leg_id (int): Leg id.

    Raises:
        ValueError: If parameter given_in_origin is not valid.
        ValueError: If parameter transform_to_origin is not valid.

    Returns:
        np.ndarray: Transformed vector.
    """
    if given_in_origin not in (config.LEG_ORIGIN, config.SPIDER_ORIGIN):
        raise ValueError(f"{given_in_origin} is not recognized as known origin.")
    if transform_to_origin not in (config.LEG_ORIGIN, config.SPIDER_ORIGIN):
        raise ValueError(f"{transform_to_origin} is not recognized as known origin.")
    
    original_vector = np.copy(vector)
    if len(vector) == 3:
        original_vector = np.append(vector, 1)

    if given_in_origin == config.LEG_ORIGIN and transform_to_origin == config.SPIDER_ORIGIN:
        return np.dot(spider.T_BASES[leg_id], original_vector)[:3]

    if given_in_origin == config.SPIDER_ORIGIN and transform_to_origin == config.LEG_ORIGIN:
        return np.dot(np.linalg.inv(spider.T_BASES[leg_id]), original_vector)[:3]
    
    if given_in_origin == transform_to_origin:
        return vector


def get_watering_leg_and_pose(spider_pose: np.ndarray, plant_position: np.ndarray = None, do_refill: bool = False) -> tuple[int, np.ndarray]:
    """Calculate spider's pose for watering the plant or refilling water tank and leg used for the task.

    Args:
        spider_pose (np.ndarray): Spider's current pose in global origin.
        plant_position (np.ndarray, optional): 1x3 array of plant's position in global origin. Should be given, if task is to water a plant. Defaults to None.
        do_refill(bool, otpional): If True, calculate refill position, otherwise calculate watering position, defaults to False.

    Returns:
        tuple: Leg id and spider's pose used for watering the plant.
    """
    if not do_refill and plant_position is None:
        raise ValueError("If task is watering the plant, plant position should be given.")
    
    if not do_refill:
        first_leg_watering_pose = np.array([
            plant_position[0] + spider.WATERING_XY_OFFSET_ABS[0],
            plant_position[1] - spider.WATERING_XY_OFFSET_ABS[1],
            spider.SPIDER_WALKING_HEIGHT,
            0.0
        ])
        fourth_leg_watering_pose = np.array([
            plant_position[0] - spider.WATERING_XY_OFFSET_ABS[0],
            plant_position[1] - spider.WATERING_XY_OFFSET_ABS[1],
            spider.SPIDER_WALKING_HEIGHT,
            0.0
        ])
    
        max_x = wall.WALL_SIZE[0] - 0.8
        min_x = 1.8
        if spider_pose[0] <= plant_position[0]:
            if spider_pose[0] <= min_x:
                watering_leg = spider.WATERING_LEGS_IDS[0]
                watering_pose = first_leg_watering_pose
            else:
                watering_leg = spider.WATERING_LEGS_IDS[1]
                watering_pose = fourth_leg_watering_pose
        else:
            if spider_pose[0] >= max_x:
                watering_leg = spider.WATERING_LEGS_IDS[1]
                watering_pose = fourth_leg_watering_pose
            else:
                watering_leg = spider.WATERING_LEGS_IDS[0]
                watering_pose = first_leg_watering_pose       

        return watering_leg, watering_pose
    
    watering_leg = spider.REFILLING_LEG_ID
    watering_pose = np.array([
        abs(spider.REFILLING_LEG_OFFSET[0]) + wall.WALL_SIZE[0] / 2,
        spider.REFILLING_Y_WALL_POSITION,
        spider.SPIDER_WALKING_HEIGHT,
        0.0
    ])

    return watering_leg, watering_pose

def get_last_joint_to_goal_pin_vector_in_spider(
    leg_id: int,
    last_joint_position_in_local: np.ndarray,
    goal_pin_position_in_global: np.ndarray,
    spider_pose: np.ndarray) -> np.ndarray:
    """Calculate vector from last joint to goal pin in spider's origin.

    Args:
        leg_id (int): Leg id.
        last_joint_position_in_local (np.ndarray): 1x3 vector of x, y and z position of last joint in local origin.
        goal_pin_position_in_global (np.ndarray): 1x3 vector of x, y and z position of goal pin in global origin.
        spider_pose (np.ndarray): Spider's pose.

    Returns:
        np.ndarray: 1x3 unit vector from last joint to goal pin, given in spider's origin.
    """
    leg_goal_position_in_local = get_leg_position_in_local(leg_id, goal_pin_position_in_global, spider_pose)
    last_joint_to_goal_pin_in_local = np.array(leg_goal_position_in_local - last_joint_position_in_local)
    last_joint_to_goal_pin_in_spider = np.dot(spider.T_BASES[leg_id][:3, :3], last_joint_to_goal_pin_in_local)

    return last_joint_to_goal_pin_in_spider / np.linalg.norm(last_joint_to_goal_pin_in_spider) 

def R_B1(q_b: float, q_1: float) -> np.ndarray:
    """Rotation matrix from spider's to 1st segment's origin.

    Args:
        q_b (float): Angle from spider's origin to leg-base origin, in radians. 
        q_1 (float): Angle in first joint, in radians.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(q_1 + q_b), -math.sin(q_1 + q_b), 0.0],
        [math.sin(q_1 + q_b), math.cos(q_1 + q_b), 0.0],
        [0.0, 0.0, 1.0]
    ], dtype = np.float32)

def R_12(q_2: float) -> np.ndarray:
    """Rotation matrix from 1st to 2nd leg-segment.

    Args:
        q_2 (float): Angle in second joint, in radians.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(q_2), -math.sin(q_2), 0.0],
        [0.0, 0.0, -1.0],
        [math.sin(q_2), math.cos(q_2), 0.0]
    ], dtype = np.float32)

def R_23(q_3: float):
    """Rotation matrix from 2nd to 3rd leg-segment.

    Args:
        q_3 (float): Angle in third joint, in radians.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(q_3), -math.sin(q_3), 0.0],
        [math.sin(q_3), math.cos(q_3), 0.0],
        [0.0, 0.0, 1.0]
    ], dtype = np.float32)

def R_B2(q_b: float, q_1: float, q_2: float) -> np.ndarray:
    """Rotation matrix from spider's to 2nd segment's origin.

    Args:
        q_b (float): Angle from spider's origin to leg-base origin, in radians.
        q_1 (float): Angle in first joint, in radians.
        q_2 (float): Angle in second joint, in radians.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(q_2) * math.cos(q_1 + q_b), -math.cos(q_1 + q_b) * math.sin(q_2), math.sin(q_1 + q_b)],
        [math.cos(q_2) * math.sin(q_1 + q_b), -math.sin(q_2) * math.sin(q_1 + q_b), -math.cos(q_1 + q_b)],
        [math.sin(q_2), math.cos(q_2), 0.0]
    ], dtype = np.float32)

def R_B3(q_b: float, q_1: float, q_2: float, q_3: float) -> np.ndarray:
    """Rotation matrix from spider's to 3rd segment's origin.

    Args:
        q_b (float): Angle from spider's origin to leg-base origin, in radians.
        q_1 (float): Angle in first joint, in radians.
        q_2 (float): Angle in second joint, in radians.
        q_3 (float): Angle in third joint, in radians.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(q_2 + q_3) * math.cos(q_1 + q_b), -math.cos(q_1 + q_b) * math.sin(q_2 + q_3), math.sin(q_1 + q_b)],
        [math.cos(q_2 + q_3) * math.sin(q_1 + q_b), -math.sin(q_2 + q_3) * math.sin(q_1 + q_b), -math.cos(q_1 + q_b)],
        [math.sin(q_2 + q_3), math.cos(q_2 + q_3), 0.0],
    ], dtype = np.float32)
