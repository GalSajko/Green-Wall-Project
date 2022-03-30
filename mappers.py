""" Module for mapping angles between DH model and motor values. """

import math
import numpy as np
import calculations

def mapJointRadiansToMotorRadians(jointValues):
    """Map angles from Inverse kinematics in radians into motor angles in radians.

    :param q1: First joint angle.
    :param q2: Second joint angle.
    :param q3: Third joint angle.
    :return: Motors angles in radians that match input angles.
    """
    q1, q2, q3 = jointValues
    q1 = calculations.GeometryTools().wrapToPi(q1 + math.pi)
    q2 = calculations.GeometryTools().wrapToPi(math.pi - q2)
    q3 = calculations.GeometryTools().wrapToPi(1.5 * math.pi + q3)

    return q1, q2, q3

def mapMotorRadiansToJointRadians(motorValues):
    """Map angles from motors in degrees to joint angles in degrees.

    :param motorValues: Motor values in degrees.
    :return: Joint values in degrees.
    """

    q1, q2, q3 = motorValues
    

    q1 = q1 - math.pi
    q2 = -(q2 - math.pi)
    q3 = q3 - 1.5 * math.pi

    return q1, q2, q3

def mapMotorRadiansToEncoder(jointValues):
    """Map degrees in motors to encoder values.

    :param jointValues: Angles in motors in degrees. 
    :return: Encoder values to match input angles.
    """
    jointValues = np.degrees(jointValues)
    encoderBits = 12
    # Convert joints values to encoder values.
    jointValues = np.array(jointValues)
    encoderValues = np.array(jointValues * 2**encoderBits / 360.0)

    return encoderValues

def mapEncoderToDegrees(encoderValues):
    """Map encoder values to motor degrees.

    :param encoderValues: Encoder values for each motor in leg.
    :return: Motors degrees for each motor in leg.
    """
    encoderBits = 12
    # Convert encoders values in degrees.
    encoderValues = np.array(encoderValues)
    jointValues = []

    jointValues.append(encoderValues[0] * 360.0 / 2**encoderBits)
    jointValues.append(encoderValues[1] * 360.0 / 2**encoderBits)
    jointValues.append(encoderValues[2] * 360.0 / 2**encoderBits)

    return np.array(jointValues)