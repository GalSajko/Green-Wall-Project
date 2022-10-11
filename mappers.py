""" Module for mapping angles between leg model and motors. """

import math
import numpy as np

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

def bno055MapYawDegrees(sensorValue):
    """Map sensor's yaw output to spider's convention.
    Sensor's convention - 0 to 360 in clockwise direction,
    Spider's convetion - 0 to 180, -180 to 0 in counter-clockwise direction.

    Args:
        sensorValue (float): Sensor's yaw value in degrees.

    Returns:
        float: Spider's yaw value in degrees.
    """
    if sensorValue < 0.0:
        sensorValue = 0.0
    elif sensorValue > 360.0:
        sensorValue = 360.0

    if 0.0 <= sensorValue < 180.0:
        return sensorValue * (-1)
    if 180.0 <= sensorValue <= 360.0:
        return (-1) * sensorValue + 360.0

def bno055MapPitchDegrees(sensorValue):
    """Map sensor's pitch output to spider's convention.
    Sensor's convention - 180 to -180 in clockwise direction,
    Spider's convention - 180 to -180 in counter-clockwise direction.

    Args:
        sensorValue (float): Sensor's pitch output in degrees.

    Returns:
        float: Spider's pitch value in degrees.
    """
    return sensorValue * (-1)

def mapBno055ToSpiderDegrees(sensorYrp, isVertical = False):
    """Map sensor's orientation to spider's, all in degrees. Note that axis are rotated to match wall (vertical) orientation.

    Args:
        sensorYrp (list): Sensor's yaw, roll and pitch values in degrees.
        isVertical (bool): If True, spider is in vertical orientation and sensor's axis are remapped, otherwise it's in horizontal orientation. Defaults to False.

    Returns:
        numpy.ndarray: 1x3 array of spider's roll, pitch and yaw in radians.
    """
    sensorYaw, sensorRoll, sensorPitch = sensorYrp
    spiderYaw = bno055MapYawDegrees(sensorYaw)
    spiderPitch = bno055MapPitchDegrees(sensorPitch)
    
    if isVertical:
        return np.array([np.radians(spiderYaw), np.radians(spiderPitch), np.radians(sensorRoll)])
    return np.array([np.radians(sensorRoll), np.radians(spiderPitch), np.radians(spiderYaw)])

def mapGravityVectorToSpiderOrigin(gravity, isVertical = False):
    """Map sensor's gravity vector into spider's origin.

    Args:
        gravity (list): 1x3 sensor's gravity vector.
        isVertical (bool): If True, spider is in vertical orientation and sensor's axis are remapped, otherwise it's in horizontal orientation. Defaults to False.

    Returns:
        numpy.ndarray: 1x3 gravity vector in spider's origin.
    """
    gravity = np.array(gravity) * (-1)
    if isVertical:
        return np.array([gravity[0], gravity[2], -gravity[1]])
    return gravity
