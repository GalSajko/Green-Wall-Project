"""Math tools needed in other modules.
"""
import math
import numpy as np

import config

def calculateSignedAngleBetweenTwoVectors(firstVector, secondVector):
    """Calculate signed angle between two vectors.

    Args:
        firstVector (list): First vector.
        secondVector (list): Second vector.

    Returns:
        float: Signed angle in radians.
    """
    dotProduct = np.dot(firstVector, secondVector)
    productOfNorms = np.linalg.norm(firstVector) * np.linalg.norm(secondVector)
    angle = math.acos(np.round(dotProduct / productOfNorms, 4))
    crossProduct = np.cross(firstVector, secondVector)
    # 2d vector.
    if len(firstVector) <= 2 and crossProduct < 0:
        angle = -angle
    # 3d vector.
    elif (crossProduct < 0).any() < 0:
        angle = -angle

    return angle

def wrapToPi(angle):
    """Wrap angle to Pi.

    Args:
        angle (float): Angle.

    Returns:
        float: Wrapped angle.
    """
    if angle < -math.pi:
        angle += math.pi * 2
    elif angle > math.pi:
        angle -= math.pi * 2

    return angle

def runningAverage(buffer, counter, newValue):
    """Calculate running average of values in buffer.

    Args:
        buffer (list): Values, used to calculate average.
        counter (int): Index at which new values are writen in buffer, if it is not already full.
        newValue (list): New values, which will be written in the buffer.

    Returns:
        tuple: Average of the buffer (element wise), shifted buffer and updated counter.
    """
    if counter < len(buffer):
        buffer[counter] = newValue
        counter += 1
        average = np.mean(buffer[:counter], axis = 0)

        return average, buffer, counter
    
    buffer = np.roll(buffer, -1, axis = 0)
    buffer[-1] = newValue
    average = np.mean(buffer, axis = 0)

    return average, buffer, counter

def dampedPseudoInverse(J):
    """Calculate damped Moore-Penrose pseudo inverse.

    Args:
        J (list): 3x3 matrix whose pseudo inverse will be calculated.

    Returns:
        numpy.ndarray: 3x3 damped pseudo inverse of J.
    """
    Jtrans = np.transpose(J)
    JJtrans = np.dot(J, Jtrans)
    alpha = np.eye(len(JJtrans)) * config.FORCE_DAMPING
    dampedFactor = np.linalg.inv(JJtrans + alpha)

    return np.dot(Jtrans, dampedFactor)

def weightedPseudoInverse(J, A):
    Jtrans = np.transpose(J)
    Ainv = np.linalg.inv(A)
    Jw = np.dot(np.dot(Ainv, Jtrans), np.linalg.inv(np.dot(J, np.dot(Ainv, Jtrans))))
    return Jw


def centerOfPolygon(vertices):
    """Calculate x and y of geometric center of polygon from given vertices.

    Args:
        points (list): nx3 array of vertices.

    Returns:
        numpy.ndarray: 1x2 array of x and y coordinate of polygon's geometric center.
    """
    return np.array([np.mean(vertices[:,0]), np.mean(vertices[:,1])])