"""Module with wall measurements and methods for creating pin-grid.
"""
import math
import numpy as np

def createGrid(threeDim):
    """Calculate pins positions. Pins are placed in squared pattern.

    Args:
        threeDim (bool): If True, calculate pins positions in 3d space, otherwise in 2d space.

    Returns:
        numpy.ndarray: nx3 or nx2 array of pins positions, where n is number of pins and second number depends on wheter positions are calculated in 3d or 2d space.
    """
    numberOfPinsInX = math.ceil(WALL_SIZE[0] / WALL_RASTER[0])
    numberOfPinsInY = math.ceil(WALL_SIZE[1] / WALL_RASTER[1])

    xGrid = [x * WALL_RASTER[0] for x in range(int(numberOfPinsInX))]
    yGrid = [y * WALL_RASTER[1] for y in range(int(numberOfPinsInY))]

    pins = []
    for x in xGrid:
        for y in yGrid:
            if not threeDim:
                pins.append([x ,y])
            else:
                pins.append([x, y, PIN_HEIGHT])

    return np.array(pins)

# Wall size given in meters - (x, y).
WALL_SIZE = [1.25, 1.3]
# Pin raster - distances between pins in (x, y).
WALL_RASTER = [0.2, 0.25]
PIN_HEIGHT = 0.0