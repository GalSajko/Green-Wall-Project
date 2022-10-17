"""Module with constants describing spider's geometry.
"""
import numpy as np
import math

#region helper private methods
def _getLegAnchorsInSpiderOrigin():
    """Calculate positions of legs-anchors in spider's origin.

    Returns:
        numpy.ndarray: 5x2 array of x, y positions for each anchor.
    """
    # Angles between anchors and spiders x axis.
    legAngles = _getLegAnglesXAxis()
    # Positions of leg anchors on spiders platform in spiders origin.
    legAnchors = [[BODY_RADIUS * math.cos(angle), BODY_RADIUS * math.sin(angle)] for angle in legAngles]

    # Reverse to match actual spiders legs order.
    legAnchorsReversed = np.flip(legAnchors[1:], 0)
    legAnchorsReversed = np.insert(legAnchorsReversed, 0, legAnchors[0], 0)

    return np.array(legAnchorsReversed)

def _getIdealLegVectors():
    """Calculate directions of ideal leg vectors in spider's origin. Ideal leg vector has a radial direction, looking from center of a spider's body.

    Returns:
        numpy.ndarray: 5x2 array of x, y directions of ideal vectors.
    """
    legAngles = _getLegAnglesXAxis()

    idealLegVectors = [np.array([legAnchor[0] + math.cos(legAngles[idx]), legAnchor[1] + math.sin(legAngles[idx])]) - legAnchor
        for idx, legAnchor in enumerate(LEG_ANCHORS)]

    # Reverse to match actual spiders legs order.
    idealLegVectorsReversed = np.flip(idealLegVectors[1:], 0)
    idealLegVectorsReversed = np.insert(idealLegVectorsReversed, 0, idealLegVectors[0], 0)

    return np.array(idealLegVectorsReversed)

def _getLegAnglesXAxis():
    """Calculate angles between vectors between anchor and spider's origin and spider's x axis.

    Returns:
        numpy.ndarray: 1x5 array of angles in radians.
    """
    legAngles = [np.radians(90) - leg * ANGLE_BETWEEN_LEGS for leg in range(NUMBER_OF_LEGS)]
    return np.array(legAngles)

def _getTransformMatricesToAnchors():
    """Calculate transformation matrices for transformation from spider's base to anchor.

    Returns:
        numpy.ndarray: 5x4x4 array of 4x4 transformation matrices for each leg-anchor.
    """
    # Constant rotation offset, because anchors x axis is pointed in radial direction.
    constantRotation = math.pi / 2

    T = []
    for i in range(NUMBER_OF_LEGS):
        rotationAngle = i * ANGLE_BETWEEN_LEGS + constantRotation
        T.append(np.array([
            [math.cos(rotationAngle), -math.sin(rotationAngle), 0, LEG_ANCHORS[i][0]],
            [math.sin(rotationAngle), math.cos(rotationAngle), 0, LEG_ANCHORS[i][1]],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]))

    return np.array(T)
#endregion

#region constants
# Number of spiders legs.
NUMBER_OF_LEGS = 5
# Number of motors in leg.
NUMBER_OF_MOTORS_IN_LEG = 3
# Radius of spiders platform, in meters.
BODY_RADIUS = 0.13145
LEGS_IDS = np.array([0, 1, 2, 3, 4], dtype = np.int8)
# Spiders legs, given as lengths of all three links in one leg.
LEGS_DIMENSIONS = np.array([[0.064, 0.3, 0.276]] * NUMBER_OF_LEGS, dtype = np.float32)
SEGMENTS_MASSES = np.array([
    [0.05, 0.475, 0.16],
    [0.05, 0.545, 0.16],
    [0.05, 0.545, 0.16],
    [0.05, 0.475, 0.16],
    [0.05, 0.545, 0.16]], dtype = np.float32)
# Vectors from start of the segment to COG.
VECTORS_TO_COG_SEGMENT = np.array([
    [0.032, 0.15, 0.14],
    [0.032, 0.14, 0.14],
    [0.032, 0.14, 0.14],
    [0.032, 0.15, 0.14],
    [0.032, 0.14, 0.14]], dtype = np.float32)
# Leg limit to avoid singularity.
LEG_LENGTH_LIMIT = 0.6
# Angles between legs, looking from spiders origin.
ANGLE_BETWEEN_LEGS = np.radians(360.0 / NUMBER_OF_LEGS)
# Positions of leg anchors on spiders platform, given in spiders origin - matching the actual legs order on spider.
LEG_ANCHORS = _getLegAnchorsInSpiderOrigin()
# Unit vectors pointing in radial directions (looking from center of body).
IDEAL_LEG_VECTORS = _getIdealLegVectors()
# Spiders constrains - min and max leg length from second joint to the end of leg and max angle of the first joint (+/- from the ideal leg vector direction).
CONSTRAINS = [0.25, 0.55, np.radians(40)]
# Array of transformation matrices for transformations from spider base to anchors in base origin.
T_ANCHORS = _getTransformMatricesToAnchors()
#endregion