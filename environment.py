"""
Green wall environment module. Contains all objects, which are present in environment, such as
Spider and Wall.
"""

from multiprocessing.sharedctypes import Value
import numpy as np
import math

class Spider:
    """ Spider class. Class is a singleton class, meaning it can only be instantiated once."""

    # Singleton class instance tracker.
    instance = None

    def __init__(self):
        # Number of spiders legs.
        self.NUMBER_OF_LEGS = 5
        # Radius of spiders platform, in meters.
        self.BODY_RADIUS = 0.1215
        # Spiders legs, given as lengths of all three links in one leg. Note that second link is in L shape -
        # first value is a length in vertical direction, second value is a horizontal offset.
        self.LEGS = [[0.065, (0.3, 0.025), 0.278],
                     [0.065, (0.3, 0.025), 0.275],
                     [0.065, (0.3, 0.025), 0.277],
                     [0.065, (0.3, 0.025), 0.2785],
                     [0.065, (0.3, 0.020), 0.276]]
        self.SECOND_JOINTS_OFFSETS = [math.tan(leg[1][1] / leg[1][0]) for leg in self.LEGS]
        # Angles between legs, looking from spiders origin.
        self.ANGLE_BETWEEN_LEGS = np.radians(360 / self.NUMBER_OF_LEGS)
        # Positions of leg anchors on spiders platform, given in spiders origin - matching the actual legs order on spider.
        self.LEG_ANCHORS = self.getLegAnchorsInSpiderOrigin()
        # Unit vectors pointing in radial directions (looking from center of body).
        self.IDEAL_LEG_VECTORS = self.getIdealLegVectors()
        # Spiders constrains - min and max leg length from second joint to the end of leg and max angle of the first joint (+/- from the ideal leg vector direction).
        self.CONSTRAINS = [0.25, 0.55, np.radians(40)]
        # Array of transformation matrices for transformations from spider base to anchors in base origin.
        self.T_ANCHORS = self.getTransformMatricesToAnchors()
        # Spider's walking height.
        self.WALKING_HEIGHT = 0.25
        # Spider's height when laying on pins.
        self.LYING_HEIGHT = 0.057
    
    def __new__(cls, *args, **kwargs):
        if not isinstance(cls.instance, cls):
            cls.instance = object.__new__(cls)
        return cls.instance

    #region Helper functions
    def getLegAnchorsInSpiderOrigin(self):
        """ Helper function for setting positions of legs anchors in spiders origin.

        :return: Numpy array of anchors positions. 
        """
        # Angles between anchors and spiders x axis.
        legAngles = self.getLegAnglesXAxis()
        # Positions of leg anchors on spiders platform in spiders origin.
        legAnchors = [[self.BODY_RADIUS * math.cos(angle), self.BODY_RADIUS * math.sin(angle)] for angle in legAngles]

        # Reverse to match actual spiders legs order.
        legAnchorsReversed = np.flip(legAnchors[1:], 0)
        legAnchorsReversed = np.insert(legAnchorsReversed, 0, legAnchors[0], 0)

        return np.array(legAnchorsReversed)
    
    def getIdealLegVectors(self):
        """ Helper functions for setting ideal leg vectors in spiders origin. 
        Ideal leg vector has a radial direction, looking from spiders origin (center of a body).

        :return: Numpy array of ideal leg unit vectors.
        """
        legAngles = self.getLegAnglesXAxis()

        idealLegVectors = [np.array([legAnchor[0] + math.cos(legAngles[idx]), legAnchor[1] + math.sin(legAngles[idx])]) - legAnchor
            for idx, legAnchor in enumerate(self.LEG_ANCHORS)]

        # Reverse to match actual spiders legs order.
        idealLegVectorsReversed = np.flip(idealLegVectors[1:], 0)
        idealLegVectorsReversed = np.insert(idealLegVectorsReversed, 0, idealLegVectors[0], 0)

        return np.array(idealLegVectorsReversed)

    def getLegAnglesXAxis(self):
        """ Helper function for calculating angles between leg anchors and spiders x axis. 

        :return: Numpy array of angles in radians. 
        """
        legAngles = [np.radians(90) - leg * self.ANGLE_BETWEEN_LEGS for leg in range(self.NUMBER_OF_LEGS)]
        return np.array(legAngles)

    def getTransformMatricesToAnchors(self):
        """Calculate transformation matrices for transformation between base and anchors origin, in base origin.

        :return: Array of transformation matrices
        """
        # Constant rotation offset, because anchors x axis is pointed in radial direction.
        constantRoation = math.pi / 2

        T = []
        for i in range(self.NUMBER_OF_LEGS):
            rotationAngle = i * self.ANGLE_BETWEEN_LEGS + constantRoation
            T.append(np.array([
                [math.cos(rotationAngle), -math.sin(rotationAngle), 0, self.LEG_ANCHORS[i][0]],
                [math.sin(rotationAngle), math.cos(rotationAngle), 0, self.LEG_ANCHORS[i][1]],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]))
        
        return np.array(T)
    #endregion

class Wall:
    """ Wall class. Class is a singleton class, meaning it can only be instantiated once.
    
    :param: gridPattern: Squared or rhombus pattern of pins grid.
    """
    
    # Singleton class instance tracker.
    instance = None

    def __init__(self, gridPattern):
        # Wall size given in meters - (x, y).
        self.WALL_SIZE = [1.25, 1.3]
        # Pin raster - distances between pins in (x, y).
        self.WALL_RASTER = [0.2, 0.25]
        self.PIN_HEIGHT = 0.02
        if (gridPattern != 'squared' and gridPattern != 'rhombus'):
            raise ValueError("Invalid value of gridPatter parameter!")
        self.gridPattern = gridPattern
    
    def __new__(cls, *args, **kwargs):
            if not isinstance(cls.instance, cls):
                cls.instance = object.__new__(cls)
            return cls.instance

    def createRhombusGrid(self, threeDim):
        """ Calculate pins positions. Pins are placed in rhombus pattern. 

        :return: Numpy array of pins positions in wall origin - (0, 0) is in the bottom-left corner of the wall.
        """
        # Distance from edge of the wall and first row/column of pins.
        edgeOffset = 0.0

        # Calculate number of pins in X and Y directions of the wall (x - horizontal, y - vertical).
        numberOfPinsInX = math.ceil(self.WALL_SIZE[0] / (self.WALL_RASTER[0] * math.sqrt(2)))
        numberOfPinsInY = math.ceil(self.WALL_SIZE[1] / (self.WALL_RASTER[1] * math.sqrt(2)))

        # Create two grids and than add them together.
        xFirstGrid = np.linspace(edgeOffset, self.WALL_SIZE[0] + edgeOffset, numberOfPinsInX)
        yFirstGrid = np.linspace(edgeOffset, self.WALL_SIZE[1] + edgeOffset, numberOfPinsInY)
        xSecondGrid = xFirstGrid + self.WALL_RASTER[0] * math.sqrt(2) / 2
        ySecondGrid = yFirstGrid + self.WALL_RASTER[1] * math.sqrt(2) / 2

        pins = []
        for x in xFirstGrid:
            for y in yFirstGrid:
                if (edgeOffset <= x < self.WALL_SIZE[0]) and (edgeOffset <= y < self.WALL_SIZE[1]):
                    if not threeDim:  
                        pins.append([x ,y])  
                    else:
                        pins.append([x, y, self.PIN_HEIGHT])

        for x in xSecondGrid:
            for y in ySecondGrid:
                if (edgeOffset <= x < self.WALL_SIZE[0]) and (edgeOffset <= y < self.WALL_SIZE[1]):
                    if not threeDim:  
                        pins.append([x ,y])  
                    else:
                        pins.append([x, y, self.PIN_HEIGHT])

        return np.array(pins)
    
    def createSquaredGrid(self, threeDim):
        """Calculate pins positions. Pins are placed in square pattern.

        :return: Numpy array of pins positions in wall origin - (0, 0) is in the left-bottom corner of the wall.
        """
        numberOfPinsInX = math.ceil(self.WALL_SIZE[0] / self.WALL_RASTER[0])
        numberOfPinsInY = math.ceil(self.WALL_SIZE[1] / self.WALL_RASTER[1])

        xGrid = [x * self.WALL_RASTER[0] for x in range(int(numberOfPinsInX))]
        yGrid = [y * self.WALL_RASTER[1] for y in range(int(numberOfPinsInY))]

        pins = []
        for x in xGrid:
            for y in yGrid:
                if not threeDim:  
                    pins.append([x ,y])  
                else:
                    pins.append([x, y, self.PIN_HEIGHT])
            
        return np.array(pins)
    
    def createGrid(self, threeDim = False):
        """Create grid of pins.

        :param gridPattern: Grid pattern.
        :param threeDim: Wheter or not pins positions are given in 3d space (including z value), defaults to False
        :raises ValueError: If gridPattern parameter value is not 'squared' or 'rhombus'.
        :return: Array of pins.
        """
        if self.gridPattern == 'squared':
            return self.createSquaredGrid(threeDim)
        elif self.gridPattern == 'rhombus':
            return self.createRhombusGrid(threeDim)
