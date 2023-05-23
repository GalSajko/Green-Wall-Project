"""Module for all kinematics calculations.
"""
import math
import numpy as np
import itertools as itt
import numba

import spider
from config import SPIDER_ORIGIN, LEG_ORIGIN
from calculations import transformations as tf
from calculations import mathtools as mathTools

#region forward kinematics
@numba.jit(nopython = True, cache = False)
def all_legs_positions(joints_values: np.ndarray, fk_type: str, legs: list = spider.LEGS_IDS) -> np.ndarray:
    """Get positions of all legs in given origin.

    Args:
        joints_values (list): 5x3 array of all angles in joints in radians.
        fk_type (char): Origin in which positions will be calculated - either spider's or leg's origin.
        legs(list, optional): List of legs ids. Defaults to spider.LEGS_IDS.

    Raises:
        ValueError: If given origin is not valid.

    Returns:
        np.ndarray: 5x3 array of (x, y, z) positions of all legs, given in selected origin.
    """
    if fk_type not in (SPIDER_ORIGIN, LEG_ORIGIN):
        raise ValueError("Invalid value of FK type parameter.")

    x_a = np.zeros((len(legs), 3), dtype = np.float32)
    for leg in legs:
        if fk_type == LEG_ORIGIN:
            x_a[leg] = leg_forward_kinematics(joints_values[leg])[:,3][:3]
            continue
        x_a[leg] = spider_base_to_leg_tip_forward_kinematics(leg, joints_values[leg])[:,3][:3]
    
    return x_a

@numba.jit(nopython = True, cache = False)
def leg_forward_kinematics(joints_values_in_leg: np.ndarray, legs_dimensions: np.ndarray = spider.LEGS_DIMENSIONS) -> np.ndarray:
    """Calculate forward kinematics for spiders leg, using transformation matrices.  

    Args:
        joints_values_in_leg (np.ndarray): 1x3 array of joints values in leg, in radians.
        legs_dimensions (np.ndarray, optiona): Dimensions of legs' segments. Defaults to spider.LEGS_DIMENSIONS.

    Returns:
        np.ndarray: 4x4 transformation matrix from base to end effector.
    """
    q_1, q_2, q_3 = joints_values_in_leg
    l_1 = legs_dimensions[0]
    l_2 = legs_dimensions[1]
    l_3 = legs_dimensions[2]

    return np.array([
        [math.cos(q_1) * math.cos(q_2 + q_3) * 1.0, -math.cos(q_1) * math.sin(q_2 + q_3) * 1.0, math.sin(q_1) * 1.0, math.cos(q_1) * (l_1 + l_2 * math.cos(q_2) + l_3 * math.cos(q_2 + q_3)) * 1.0],
        [math.cos(q_2 + q_3) * math.sin(q_1) * 1.0, -math.sin(q_1) * math.sin(q_2 + q_3) * 1.0, -math.cos(q_1) * 1.0, (l_1 + l_2 * math.cos(q_2) + l_3 * math.cos(q_2 + q_3)) * math.sin(q_1) * 1.0],
        [math.sin(q_2 + q_3) * 1.0, math.cos(q_2 + q_3) * 1.0, 0.0, l_2 * math.sin(q_2) + l_3 * math.sin(q_2 + q_3) * 1.0],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype = np.float32)

def leg_base_to_third_joint_forward_kinematics(joints_values_in_leg: np.ndarray) -> np.ndarray:
    """Calculate forward kinematics from leg base to third joint.

    Args:
        joints_values_in_leg (np.ndarray): 1x3 array of joints values in leg, in radians.

    Returns:
        np.ndarray: 4x4 transformation matrix from leg-base to third joint.
    """
    q_1, q_2, _ = joints_values_in_leg
    l_1 = spider.LEGS_DIMENSIONS[0]
    l_2 = spider.LEGS_DIMENSIONS[1]

    return np.array([
        [math.cos(q_1) * math.cos(q_2), -math.cos(q_1) * math.sin(q_2), math.sin(q_1), math.cos(q_1) * (l_1 + l_2 * math.cos(q_2))],
        [math.cos(q_2) * math.sin(q_1), -math.sin(q_1) * math.sin(q_2), -math.cos(q_1), (l_1 + l_2 * math.cos(q_2)) * math.sin(q_1)],
        [math.sin(q_2), math.cos(q_2), 0.0, l_2 * math.sin(q_2)],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype = np.float32)

@numba.jit(nopython = True, cache = False)
def spider_base_to_leg_tip_forward_kinematics(
    leg_id: int,
    joints_values_in_leg: np.ndarray,
    angle_between_legs: float = spider.ANGLE_BETWEEN_LEGS,
    radius: float = spider.BODY_RADIUS,
    legs_dimensions: np.ndarray = spider.LEGS_DIMENSIONS) -> np.ndarray:
    """Calculate forward kinematics from spider base to leg-tip.

    Args:
        leg_id (int): Leg id.
        joints_values_in_leg (np.ndarray): 1x3 array of joints values in leg, in radians
        angle_between_legs (float, optional): Angle between two legs. Defaults to spider.ANGLE_BETWEEN_LEGS.
        radius (float, optional): Radius of robot's body. Defaults to spider.BODY_RADIUS.
        legs_dimensions (np.ndarray, optional): Dimensions of legs' segments. Defaults to spider.LEGS_DIMENSIONS.

    Returns:
        np.ndarray: 4x4 transformation matrix.
    """
    q_b = leg_id * angle_between_legs + math.pi / 2
    q_1, q_2, q_3 = joints_values_in_leg
    r = radius
    l_1 = legs_dimensions[0]
    l_2 = legs_dimensions[1]
    l_3 = legs_dimensions[2]

    return np.array([
        [math.cos(q_2 + q_3) * math.cos(q_1 + q_b), -math.cos(q_1 + q_b) * math.sin(q_2 + q_3), math.sin(q_1 + q_b), r * math.cos(q_b) + (l_1 + l_2 * math.cos(q_2) + l_3 * math.cos(q_2 + q_3)) * math.cos(q_1 + q_b)],
        [math.cos(q_2 + q_3) * math.sin(q_1 + q_b), -math.sin(q_2 + q_3) * math.sin(q_1 + q_b), -math.cos(q_1 + q_b), r * math.sin(q_b) + (l_1 + l_2 * math.cos(q_2) + l_3 * math.cos(q_2 + q_3)) * math.sin(q_1 + q_b)],
        [math.sin(q_2 + q_3), math.cos(q_2 + q_3), 0.0, l_2 * math.sin(q_2) + l_3 * math.sin(q_2 + q_3)],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype = np.float32)

def platform_forward_kinematics(legs_ids: list, legs_positions_in_global: np.ndarray, legs_poses_in_spider: np.ndarray) -> list:
    """Calculate forward kinematics of platform. Use only those legs, that are in contact with pins.

    Args:
        legs_ids (list): Ids of used legs.
        legs_positions_in_global (np.ndarray): Global positions of used legs.
        legs_poses_in_spider (np.ndarray): 4x4 transformation matrices, representing a legs poses in spider's origin.

    Returns:
        list: 1x6 list with global xyzrpy values.
    """

    if len(legs_ids) != 3 or len(legs_positions_in_global) != 3 or len(legs_poses_in_spider) != 3:
        print("Use exactly three legs for calculations of platform forward kinematics.")
        return False
    
    legs_poses_in_spider = np.array(legs_poses_in_spider)
    legs_positions_in_global = np.array(legs_positions_in_global)
    l_1, l_2, l_3 = legs_poses_in_spider
    p_1, p_2, _ = legs_positions_in_global

    # Compute coordinate system of a wall-plane (in spider's origin)
    l_1_2 = l_2[:,3][:3] - l_1[:,3][:3]
    l_1_3 = l_3[:,3][:3] - l_1[:,3][:3]
    l_2_3 = l_3[:,3][:3] - l_2[:,3][:3]
    n = [
        np.cross(l_1_2, l_1_3) if np.cross(l_1_2, l_1_3)[2] >= 0.0 else np.cross(l_1_3, l_1_2),
        np.cross(l_1_2, l_2_3) if np.cross(l_1_2, l_2_3)[2] >= 0.0 else np.cross(l_2_3, l_1_2),
        np.cross(-l_1_3, -l_2_3) if np.cross(-l_1_3, -l_2_3)[2] >= 0.0 else np.cross(-l_2_3, -l_1_3)
    ]
    n = np.mean(n, axis = 0)
    e_z = n / np.linalg.norm(n)
    e_x = l_1_2 / np.linalg.norm(l_1_2)
    e_y = np.cross(e_z, e_x)
    P = np.array([e_x, e_y, e_z])

    p_1_2 = p_2 - p_1
    phi = mathTools.calculate_signed_angle_between_two_vectors(p_1_2[:2], np.array([1, 0]))

    # Rotate P for angle phi aroud z axis to align it with global origin.
    rot = np.array([
        [math.cos(phi), -math.sin(phi), 0],
        [math.sin(phi), math.cos(phi), 0],
        [0, 0, 1]
    ])
    P_global = np.dot(np.transpose(rot), P)
    P_global = np.c_[P_global, np.zeros(3)]
    P_global = np.r_[P_global, [[0, 0, 0, 1]]]

    positions = np.zeros([len(legs_poses_in_spider), 3])
    for idx, leg in enumerate(legs_poses_in_spider):
        positions[idx] = legs_positions_in_global[idx] + np.dot(P_global[:3, :3], -leg[:, 3][:3])
    P_global[:,3][:3] = np.mean(positions, axis = 0)

    yaw = math.atan2(P_global[1, 0], P_global[0, 0])
    # Note that roll and pitch are swapped because of spider's axis definition.
    roll = math.atan2(-P_global[2, 0], math.sqrt(math.pow(P_global[2, 1], 2) + math.pow(P_global[2, 2], 2)))
    pitch = math.atan2(P_global[2, 1], P_global[2, 2])

    x, y, z = P_global[:,3][:3]
    xyzrpy = [x, y, z, roll, pitch, yaw]
    
    return xyzrpy

def get_spider_pose(legs_ids: list, legs_positions_in_global: np.ndarray, joints_values: np.ndarray) -> np.ndarray:
    """Calculate spider's pose in global origin. If more than three legs are given, it calculates spider's pose from each
    combination of these three legs. Finally pose is determined as mean value of all calculations.

    Args:
        legs_ids (list): Ids of legs to use for calculating spider's pose. Should not use less than three legs.
        legs_positions_in_global (np.ndarray): nx3 array of x, y, z positions of used legs in global origin, where n should be the same as length of legs_ids.
        joints_values (np.ndarray): Array of angles in joints in radians.

    Returns:
        np.ndarray: 1x6 array of xyzrpy pose.
    """
    legs_positions_in_global = np.array(legs_positions_in_global)
    legs_ids = list(legs_ids)
    legs_combinations = itt.combinations(legs_ids, 3)
    poses = np.zeros(6, len(legs_combinations))
    for combination_index, legs_subset in enumerate(legs_combinations):
        legs_subset = np.array(legs_subset)
        subset_indexes = [legs_ids.index(leg) for leg in legs_subset]
        # Skip calculations if all three selected legs are on the same line.
        if not (np.diff(legs_positions_in_global[subset_indexes][:, 0]).any() and np.diff(legs_positions_in_global[subset_indexes][:, 1]).any()):
            continue
        joints_values_in_legs_subset = joints_values[legs_subset]
        legs_poses = np.zeros([3, 4, 4])
        for idx, leg in enumerate(legs_subset):
            legs_poses[idx] = spider_base_to_leg_tip_forward_kinematics(leg, joints_values_in_legs_subset[idx])
        poses[combination_index] = platform_forward_kinematics(legs_subset, legs_positions_in_global[(subset_indexes)], legs_poses)
    pose = np.mean(np.array(poses), axis = 0)

    return pose

def get_goal_pin_in_local(leg_id: int, attached_legs_ids: np.ndarray, legs_positions_in_global: np.ndarray, joints_values: np.ndarray, rpy: list) -> np.ndarray:
    """Calculate goal pin position in leg-local origin.

    Args:
        leg_id (int): Leg id.
        attached_legs_ids (np.ndarray): List of attached legs.
        legs_positions_in_global (np.ndarray): 5x3 array of global legs positions.
        joints_values (np.ndarray): 5x3 array of joints values.
        rpy (list): 1x3 array of spider's rpy rotation values.

    Returns:
        np.ndarray: 1x3 array of goal pin's x, y, z position in leg-local origin.
    """
    pose = get_spider_pose(attached_legs_ids, legs_positions_in_global[attached_legs_ids], joints_values)
    pose = tf.xyzrpy_to_matrix(pose)
    rotation = tf.xyzrpy_to_matrix(rpy, True)
    pose[:3, :3] = rotation
    goal_pin_in_spider = np.dot(np.linalg.inv(pose), np.append(legs_positions_in_global[leg_id], 1))
    goal_pin_in_local = np.dot(np.linalg.inv(spider.T_BASES[leg_id]), goal_pin_in_spider)[:3]

    return goal_pin_in_local
#endregion

#region inverse kinematics
@numba.jit(nopython = True, cache = False)
def leg_inverse_kinematics(leg_position_in_local: np.ndarray, legs_dimensions: np.ndarray = spider.LEGS_DIMENSIONS) -> tuple[float, float, float]:
    """Calculate inverse kinematics for leg, using geometry.

    Args:
        leg_position_in_local (np.ndarray): Desired position of end effector in leg-base origin.
        legs_dimensions (np.ndarray, optional): Dimensions of legs' segments.

    Returns:
        tuple: Angles in radians in first, second and third joint.
    """
    l_1, l_2, l_3 = legs_dimensions

    # Angle in first joint.
    q_1 = math.atan2(leg_position_in_local[1], leg_position_in_local[0])

    # Position of second joint in leg-base origin.
    second_joint_position = np.array([
        l_1 * math.cos(q_1),
        l_1 * math.sin(q_1),
        0], dtype = np.float32)

    # Vector from second joint to end effector.
    second_joint_to_end_vector = leg_position_in_local - second_joint_position

    # Distance between second joint and end effector.
    r = np.linalg.norm(second_joint_to_end_vector)
    # Angle in third joint, note Pi - acos(x) = acos(-x).
    q_3 =  math.acos(np.round((r**2 - l_2**2 - l_3**2) / (2 * l_2 * l_3), 4))

    # Angle in second joint.
    alpha = abs(math.atan2(l_3 * math.sin(q_3), l_2 + l_3 * math.cos(q_3)))
    xy = np.linalg.norm(second_joint_to_end_vector[0:2])
    gamma = math.atan2(second_joint_to_end_vector[2], xy)
    q_2 = alpha + gamma
    
    return q_1, q_2, q_3
#endregion

#region jacobians
@numba.jit(nopython = True, cache = False)
def leg_jacobi(joints_values_in_leg: np.ndarray, legs_dimensions: np.ndarray = spider.LEGS_DIMENSIONS) -> np.ndarray:
    """Calculate Jacobian matrix for given leg.

    Args:
        leg_id (np.ndarray): Leg id.
        joints_values_in_leg (np.ndarray): 1x3 array of joints values in leg, in radians.

    Returns:
        np.ndarray: 3x3 Jacobian matrix.
    """
    q_1, q_2, q_3 = joints_values_in_leg
    l_1 = legs_dimensions[0]
    l_2 = legs_dimensions[1]
    l_3 = legs_dimensions[2]
    
    return np.array([
        [-(l_1 + l_2 * math.cos(q_2) + l_3 * math.cos(q_2 + q_3)) * math.sin(q_1) * 1.0, -math.cos(q_1) * (l_2 * math.sin(q_2) + l_3 * math.sin(q_2 + q_3)) * 1.0, -l_3 * math.cos(q_1) * math.sin(q_2 + q_3) * 1.0],
        [math.cos(q_1) * (l_1 + l_2 * math.cos(q_2) + l_3 * math.cos(q_2 + q_3)) * 1.0, -math.sin(q_1) * (l_2 * math.sin(q_2) + l_3 * math.sin(q_2 + q_3)) * 1.0, -l_3 * math.sin(q_1) * math.sin(q_2 + q_3) * 1.0],
        [0.0, l_2 * math.cos(q_2) + l_3 * math.cos(q_2 + q_3) * 1.0, l_3 * math.cos(q_2 + q_3) * 1.0]
        ], dtype = np.float32)

@numba.jit(nopython = True, cache = False)
def spider_base_to_leg_tip_jacobi(
    leg_id: int, 
    joints_values_in_leg: np.ndarray, 
    angle: float = spider.ANGLE_BETWEEN_LEGS, 
    legs_dimensions: np.ndarray = spider.LEGS_DIMENSIONS) -> np.ndarray:
    """Calculate Jacobian matrix for spider's origin - leg-tip relation.

    Args:
        leg_id (int): Leg id.
        joints_values_in_leg (np.ndarray): 1x3 array of joints values in leg, in radians.
        angle (float, optional): Angle between legs.
        legs_dimensions (np.ndarray, optional): Dimensions of legs' segments.
    Returns:
        np.ndarray: 3x3 Jacobian matrix.
    """
    q_b = leg_id * angle + math.pi / 2
    q_1, q_2, q_3 = joints_values_in_leg
    l_1 = legs_dimensions[0]
    l_2 = legs_dimensions[1]
    l_3 = legs_dimensions[2]

    return np.array([
        [-(l_1 + l_2 * math.cos(q_2) + l_3 * math.cos(q_2 + q_3)) * math.sin(q_1 + q_b), -math.cos(q_1 + q_b) * (l_2 * math.sin(q_2) + l_3 * math.sin(q_2 + q_3)), -l_3 * math.cos(q_1 + q_b) * math.sin(q_2 + q_3)],
        [(l_1 + l_2 * math.cos(q_2) + l_3 * math.cos(q_2 + q_3)) * math.cos(q_1 + q_b), -(l_2 * math.sin(q_2) + l_3 * math.sin(q_2 + q_3)) * math.sin(q_1 + q_b), -l_3 * math.sin(q_2 + q_3) * math.sin(q_1 + q_b)],
        [0, l_2 * math.cos(q_2) + l_3 * math.cos(q_2 + q_3), l_3 * math.cos(q_2 + q_3)]
    ], dtype = np.float32)

@numba.jit(nopython = True, cache = False)
def get_joints_velocities(
    joints_values: np.ndarray,
    legs_velocities: np.ndarray,
    number_of_legs: int = spider.NUMBER_OF_LEGS,
    number_of_motors_in_leg: int = spider.NUMBER_OF_MOTORS_IN_LEG) -> np.ndarray:
    """Calculated needed joints velocities from given end-effectors' velocites, for all legs.

    Args:
        joints_values (np.ndarray): 5x3 array of angles in joints.
        legs_velocities (np.ndarray): 5x3 array of x, y, z velocities of all legs.
        number_of_legs (int, optional): Number of legs. Defaults to spider.NUMBER_OF_LEGS.
        number_of_motors_in_leg (int, optional): Number of motors in leg. Defaults to spider.NUMBER_OF_MOTORS_IN_LEG.

    Returns:
        np.ndarray: 5x3 array of joints velocities.
    """
    joints_velocities = np.zeros((number_of_legs, number_of_motors_in_leg), dtype = np.float32)

    for leg, leg_velocity in enumerate(legs_velocities):
        J_inv = np.ascontiguousarray(mathTools.damped_pseudoinverse(leg_jacobi(joints_values[leg])))
        leg_velocity = np.ascontiguousarray(leg_velocity)
        joints_velocities[leg] = np.dot(J_inv, leg_velocity)
    
    return joints_velocities

@numba.jit(nopython = True, cache = False)
def get_xd_xdd_from_offsets(
    force_mode_legs_ids: np.ndarray,
    offsets_in_spider: np.ndarray,
    velocities_in_spider: np.ndarray,
    spider_to_leg_transformations: np.ndarray = spider.T_BASES) -> tuple[np.ndarray, np.ndarray]:
    """Rotate position offets and velocities, calculated from force controller into leg-local origins.

    Args:
        force_mode_legs_ids (np.ndarray): List of legs' ids, that are used in force controll.
        offsets_in_spider (np.ndarray): Calculated offsets from force-position P controller.
        velocities_in_spider (np.ndarray): Calculated velocities from force-position P controller.
        spider_to_leg_transformations (np.ndarray): Transformations from spider's to legs' origins. Defaults to spider.T_BASE.

    Returns:
        Tuple: Two nx3 numpy.ndarrays with (x, y, z) offsets and velocities for each leg used in force mode, where n is number of used legs.
    """
    offsets_in_locals = np.zeros((len(force_mode_legs_ids), 3), dtype = np.float32)
    velocities_in_locals = np.zeros((len(force_mode_legs_ids), 3), dtype = np.float32)

    for i, leg in enumerate(force_mode_legs_ids):
        # Rotate offset and velocity vector in spider's origin into leg-local origin.
        spider_to_leg_rotation = np.transpose(spider_to_leg_transformations[leg][:3, :3])
        offsets_in_locals[i] = np.dot(spider_to_leg_rotation, offsets_in_spider[leg])
        velocities_in_locals[i] = np.dot(spider_to_leg_rotation, velocities_in_spider[leg])

    return offsets_in_locals, velocities_in_locals
#endregion
