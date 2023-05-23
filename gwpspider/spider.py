"""Module with constants describing spider's geometry.
"""
import numpy as np
import math

#region helper private methods
def _get_legs_bases_in_spider() -> np.ndarray:
    """Calculate positions of legs-anchors in spider's origin.

    Returns:
        np.ndarray: 5x2 array of x, y positions for each anchor.
    """
    # Angles between anchors and spiders x axis.
    leg_angles = _get_leg_angles_x_axis()
    # Positions of leg bases on spiders platform in spiders origin.
    leg_bases_in_spider = [[BODY_RADIUS * math.cos(angle), BODY_RADIUS * math.sin(angle)] for angle in leg_angles]

    # Reverse to match actual spiders legs order.
    leg_bases_in_spider_reversed = np.flip(leg_bases_in_spider[1:], 0)
    leg_bases_in_spider_reversed = np.insert(leg_bases_in_spider_reversed, 0, leg_bases_in_spider[0], 0)

    return np.array(leg_bases_in_spider_reversed)

def _get_ideal_leg_vectors() -> np.ndarray:
    """Calculate directions of ideal leg vectors in spider's origin. Ideal leg vector has a radial direction, looking from center of a spider's body.

    Returns:
        np.ndarray: 5x2 array of x, y directions of ideal vectors.
    """
    leg_angles = _get_leg_angles_x_axis()

    ideal_leg_vectors = [np.array([leg_base[0] + math.cos(leg_angles[idx]), leg_base[1] + math.sin(leg_angles[idx])]) - leg_base for idx, leg_base in enumerate(LEG_BASES)]

    # Reverse to match actual spiders legs order.
    ideal_leg_vectors_reversed = np.flip(ideal_leg_vectors[1:], 0)
    ideal_leg_vectors_reversed = np.insert(ideal_leg_vectors_reversed, 0, ideal_leg_vectors[0], 0)

    return np.array(ideal_leg_vectors_reversed)

def _get_leg_angles_x_axis() -> np.ndarray:
    """Calculate angles between vectors between anchor and spider's origin and spider's x axis.

    Returns:
        np.ndarray: 1x5 array of angles in radians.
    """
    leg_angles = [np.radians(90) - leg * ANGLE_BETWEEN_LEGS for leg in range(NUMBER_OF_LEGS)]
    return np.array(leg_angles)

def _get_spider_to_legs_bases_transformations() -> np.ndarray:
    """Calculate transformation matrices for transformation from spider's base to anchor.

    Returns:
        np.ndarray: 5x4x4 array of 4x4 transformation matrices for each leg-anchor.
    """
    # Constant rotation offset, because anchors x axis is pointed in radial direction.
    constant_rotation = math.pi / 2

    T = []
    for i in range(NUMBER_OF_LEGS):
        rotation_angle = i * ANGLE_BETWEEN_LEGS + constant_rotation
        T.append(np.array([
            [math.cos(rotation_angle), -math.sin(rotation_angle), 0.0, LEG_BASES[i][0]],
            [math.sin(rotation_angle), math.cos(rotation_angle), 0.0, LEG_BASES[i][1]],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype = np.float32))

    return np.array(T, dtype = np.float32)
#endregion

NUMBER_OF_LEGS = 5
NUMBER_OF_MOTORS_IN_LEG = 3
# Radius of spiders platform, in meters.
BODY_RADIUS = 0.15
SPIDER_WALKING_HEIGHT = 0.3
# Leg id used for (re)filling water tank.
REFILLING_LEG_ID = 2
REFILLING_Y_WALL_POSITION = 0.6
# Leg limit to avoid singularity.
LEG_LENGTH_MAX_LIMIT = 0.58
LEG_LENGTH_MIN_LIMIT = 0.32
# Angles between legs, looking from spiders origin.
ANGLE_BETWEEN_LEGS = np.radians(360.0 / NUMBER_OF_LEGS)
#region constant lists
# Positions of leg anchors on spiders platform, given in spiders origin - matching the actual legs order on spider.
LEG_BASES = _get_legs_bases_in_spider()
# Unit vectors pointing in radial directions (looking from center of body).
IDEAL_LEG_VECTORS = _get_ideal_leg_vectors()
# Spiders constrains - min and max leg length from second joint to the end of leg and max angle of the first joint (+/- from the ideal leg vector direction).
CONSTRAINS = [0.15, 0.45, np.radians(60)]
# Array of transformation matrices for transformations from spider base to anchors in base origin.
T_BASES = _get_spider_to_legs_bases_transformations()
# Spiders legs, given as lengths of all three links in one leg.
LEGS_DIMENSIONS = np.array([0.064, 0.3, 0.276], dtype = np.float32)
SEGMENTS_MASSES = np.array([
    [0.057, 0.62, 0.27],
    [0.057, 0.62, 0.29],
    [0.057, 0.62, 0.27],
    [0.057, 0.62, 0.27],
    [0.057, 0.62, 0.27]], dtype = np.float32)
# Vectors from start of the segment to COG.
VECTORS_TO_COG_SEGMENT = np.array([
    [0.032, 0.19, 0.158],
    [0.032, 0.19, 0.158],
    [0.032, 0.19, 0.158],
    [0.032, 0.19, 0.158],
    [0.032, 0.19, 0.158]], dtype = np.float32)
# Spider's watering position regarding to plant position.
WATERING_XY_OFFSET_ABS = [0.45, 0.15]
WATERING_LEGS_IDS = [1, 4]
REFILLING_LEG_OFFSET = np.array([-0.2, -0.65, -0.2], dtype = np.float32)
LEGS_IDS = np.array([0, 1, 2, 3, 4], dtype = np.int8)
#endregion