""" Module for mapping angles between leg model and motors. """

import math
import numpy as np
import time

def mapModelAnglesRadiansToMotorsAnglesRadians(modelAngles):
    """Map leg-model angles to motors angles, both in radians. Each leg has three motors, 
    so this function converts three leg-model angles to three motors angles.

    Args:
        modelAngles (list): 1x3 array of leg-model angles in radians.

    Returns:
        tuple: Motors angles for each motor in leg in radians, matching given leg-model angles.
    """
    q1, q2, q3 = modelAngles
    q1 = q1 + math.pi
    q2 = math.pi - q2

    return q1, q2, q3

def mapMotorsAnglesRadiansToModelAnglesRadians(motorsAngles):
    """Map motors angles to leg-model angles, both in radians, for all three motos in leg.

    Args:
        motorsAngles (list): 1x3 array of motors angles in radians

    Returns:
        tuple: Leg-model angles in radians, matching given motors' angles.
    """
    q1, q2, q3 = motorsAngles

    q1 = q1 - math.pi
    q2 = -(q2 - math.pi)

    return q1, q2, q3

def mapModelAnglesRadiansToPositionEncoderValues(modelAngles):
    """Map leg-model angles in radians to motors' encoders values.

    Args:
        modelAngles (list): 1x3 array of leg-model angles in radians.

    Returns:
        numpy.ndarray: 1x3 array of motors' encoders values, matching given leg-model angles.
    """
    modelAngles = np.degrees(modelAngles)
    encoderBits = 12
    modelAngles = np.array(modelAngles)
    encoderValues = np.array(modelAngles * math.pow(2, encoderBits) / 360.0)

    return encoderValues

def mapPositionEncoderValuesToMotorsAnglesDegrees(encodersValues):
    """Map motors' encoder values to motors' angles in degrees.

    Args:
        encodersValues (list): 1xn array of encoder values to convert, where n is number of given values.

    Returns:
        numpy.ndarray: 1xn array of motors' angles in degrees, where n is number of input encoders' values.
    """
    encoderBits = 12
    encodersValues = np.array(encodersValues)

    return encodersValues * 360.0 / math.pow(2, encoderBits)

def mapPositionEncoderValuesToModelAnglesRadians(encodersValues):
    """Map encoders' values of each motor in leg to model angles in radians.

    Args:
        encodersValues (list): 1x3 array of encoders' values of each motor in leg.

    Returns:
        numpy.ndarray: 1x3 array of leg-model angles in radians.
    """
    encodersValues = np.array(encodersValues)
    k = np.array([math.pi / 2048, -math.pi / 2048, math.pi / 2048])
    n = np.array([-math.pi, math.pi, -math.pi])

    return np.array(k * encodersValues + n)

def mapCurrentEncoderValuesToMotorsCurrentsAmpers(encoderValues):
    """Map encoded current values of each motor in leg to currents in motors in Ampers.

    Args:
        encoderValues (list): 1x3 array of encoded current values of each motor in leg.

    Returns:
        numpy.ndarray: 1x3 array of currents in motors in Ampers.
    """

    encoderValues = np.array(encoderValues, dtype = np.float32)
    mappedValues = np.array([(encoderValue - 65535) if encoderValue > 0x7fff else encoderValue for encoderValue in encoderValues], dtype = np.float32)

    return mappedValues * 0.00269

def mapModelVelocitiesToVelocityEncoderValues(modelVelocities):
    """Map velocities of each joint in leg-model to velocity encoders values.

    Args:
        modelVelocities (list): 1x3 array of velocities in each joint of leg-model.

    Returns:
        numpy.ndarray: 1x3 array of encoded motors' velocities, matching given leg-model velocities.
    """
    encoderVelocityLimit = 75
    jointVelocitiyRpmLimit = 17.18

    modelVelocities = np.array(modelVelocities)
    # Rad/s to rad/min.
    jointVelocitiesRpm = (60 / (2*math.pi)) * modelVelocities

    # Convert to encoder values.
    encoderValues = jointVelocitiesRpm * (encoderVelocityLimit / jointVelocitiyRpmLimit)
    encoderValues[1] = -encoderValues[1]

    return encoderValues

def mapRgValuesToOffsetsForOffloading(usedLegs, rgValues, offsetDirection = np.array([0, -1, 0])):
    """Map rg values of used legs to offsets for the purpuse of offloading one of the legs.

    Args:
        usedLegs (list): 1x4 array of ids of used legs.
        rgValues (list): 1x5 array of rg values.

    Returns:
        numpy.ndarray: 1x4 array of offsets in meters.
    """
    maxOffset = 0.02
    usedRgValues = np.array(rgValues)[usedLegs]
    weights = usedRgValues / np.max(usedRgValues)
    offsets = [maxOffset * weight * offsetDirection for weight in weights]
    
    return offsets

