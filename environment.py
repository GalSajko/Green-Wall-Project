"""
Green wall environment module. Contains all objects, which are present in environment, such as
Spider and Wall.
"""
import numpy as np
import math

class Spider:
    """ Spider class.
    """
    def __init__(self):
        # Number of spiders legs.
        self.NUMBER_OF_LEGS = 5
        # Number of motors in leg.
        self.NUMBER_OF_MOTORS_IN_LEG = 3
        # Radius of spiders platform, in meters.
        self.BODY_RADIUS = 0.13145
        self.LEGS_IDS = np.array([0, 1, 2, 3, 4], dtype = np.int8)
        # Spiders legs, given as lengths of all three links in one leg.
        self.LEGS_DIMENSIONS = np.array([[0.064, 0.3, 0.276]] * self.NUMBER_OF_LEGS, dtype = np.float32)
        self.SEGMENTS_MASSES = np.array([
            [0.05, 0.475, 0.16],
            [0.05, 0.545, 0.16],
            [0.05, 0.545, 0.16],
            [0.05, 0.475, 0.16],
            [0.05, 0.545, 0.16]], dtype = np.float32)
        # Vectors from start of the segment to COG.
        self.VECTORS_TO_COG_SEGMENT = np.array([
            [0.032, 0.15, 0.14],
            [0.032, 0.14, 0.14],
            [0.032, 0.14, 0.14],
            [0.032, 0.15, 0.14],
            [0.032, 0.14, 0.14]], dtype = np.float32)
        # Leg limit to avoid singularity.
        self.LEG_LENGTH_LIMIT = 0.6
        # Angles between legs, looking from spiders origin.
        self.ANGLE_BETWEEN_LEGS = np.radians(360.0 / self.NUMBER_OF_LEGS)
        # Positions of leg anchors on spiders platform, given in spiders origin - matching the actual legs order on spider.
        self.LEG_ANCHORS = self.__getLegAnchorsInSpiderOrigin()
        # Unit vectors pointing in radial directions (looking from center of body).
        self.IDEAL_LEG_VECTORS = self.__getIdealLegVectors()
        # Spiders constrains - min and max leg length from second joint to the end of leg and max angle of the first joint (+/- from the ideal leg vector direction).
        self.CONSTRAINS = [0.25, 0.55, np.radians(40)]
        # Array of transformation matrices for transformations from spider base to anchors in base origin.
        self.T_ANCHORS = self.__getTransformMatricesToAnchors()

    def __getLegAnchorsInSpiderOrigin(self):
        """Calculate positions of legs-anchors in spider's origin.

        Returns:
            numpy.ndarray: 5x2 array of x, y positions for each anchor.
        """
        # Angles between anchors and spiders x axis.
        legAngles = self.__getLegAnglesXAxis()
        # Positions of leg anchors on spiders platform in spiders origin.
        legAnchors = [[self.BODY_RADIUS * math.cos(angle), self.BODY_RADIUS * math.sin(angle)] for angle in legAngles]

        # Reverse to match actual spiders legs order.
        legAnchorsReversed = np.flip(legAnchors[1:], 0)
        legAnchorsReversed = np.insert(legAnchorsReversed, 0, legAnchors[0], 0)

        return np.array(legAnchorsReversed)

    def __getIdealLegVectors(self):
        """Calculate directions of ideal leg vectors in spider's origin. Ideal leg vector has a radial direction, looking from center of a spider's body.

        Returns:
            numpy.ndarray: 5x2 array of x, y directions of ideal vectors.
        """
        legAngles = self.__getLegAnglesXAxis()

        idealLegVectors = [np.array([legAnchor[0] + math.cos(legAngles[idx]), legAnchor[1] + math.sin(legAngles[idx])]) - legAnchor
            for idx, legAnchor in enumerate(self.LEG_ANCHORS)]

        # Reverse to match actual spiders legs order.
        idealLegVectorsReversed = np.flip(idealLegVectors[1:], 0)
        idealLegVectorsReversed = np.insert(idealLegVectorsReversed, 0, idealLegVectors[0], 0)

        return np.array(idealLegVectorsReversed)

    def __getLegAnglesXAxis(self):
        """Calculate angles between vectors between anchor and spider's origin and spider's x axis.

        Returns:
            numpy.ndarray: 1x5 array of angles in radians.
        """
        legAngles = [np.radians(90) - leg * self.ANGLE_BETWEEN_LEGS for leg in range(self.NUMBER_OF_LEGS)]
        return np.array(legAngles)

    def __getTransformMatricesToAnchors(self):
        """Calculate transformation matrices for transformation from spider's base to anchor.

        Returns:
            numpy.ndarray: 5x4x4 array of 4x4 transformation matrices for each leg-anchor.
        """
        # Constant rotation offset, because anchors x axis is pointed in radial direction.
        constantRotation = math.pi / 2

        T = []
        for i in range(self.NUMBER_OF_LEGS):
            rotationAngle = i * self.ANGLE_BETWEEN_LEGS + constantRotation
            T.append(np.array([
                [math.cos(rotationAngle), -math.sin(rotationAngle), 0, self.LEG_ANCHORS[i][0]],
                [math.sin(rotationAngle), math.cos(rotationAngle), 0, self.LEG_ANCHORS[i][1]],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]))

        return np.array(T)

class Wall:
    """Wall class.
    """
    def __init__(self, gridPattern):
        # Wall size given in meters - (x, y).
        self.WALL_SIZE = [1.25, 1.3]
        # self.WALL_SIZE = [6.0, 4.0]
        # Pin raster - distances between pins in (x, y).
        self.WALL_RASTER = [0.2, 0.25]
        self.PIN_HEIGHT = 0.0
        if gridPattern not in ('squared', 'rhombus'):
            raise ValueError("Invalid value of gridPatter parameter!")
        self.gridPattern = gridPattern

    #region public methods
    def createGrid(self, threeDim = False):
        """Wrapper function for calculating pins positions on the wall. Desired shape of grid is given at class initialization.

        Args:
            threeDim (bool, optional): If True, calculate pins positions in 3d space, otherwise in 2d space. Defaults to False.

        Returns:
            numpy.ndarray: nx3 or nx2 array of pins positions, where n is number of pins and second number depends on wheter positions are calculated in 3d or 2d space.
        """
        if self.gridPattern == 'squared':
            return self.__createSquaredGrid(threeDim)

        return self.__createRhombusGrid(threeDim)
    #endregion

    #region private methods
    def __createRhombusGrid(self, threeDim):
        """Calculate pins positions. Pins are placed in rhombus pattern.

        Args:
            threeDim (bool): If True, calculate pins positions in 3d space, otherwise in 2d space.

        Returns:
            numpy.ndarray: nx3 or nx2 array of pins positions, where n is number of pins and second number depends wheter positions are calculated in 3d or 2d space.
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

    def __createSquaredGrid(self, threeDim):
        """Calculate pins positions. Pins are placed in squared pattern.

        Args:
            threeDim (bool): If True, calculate pins positions in 3d space, otherwise in 2d space.

        Returns:
            numpy.ndarray: nx3 or nx2 array of pins positions, where n is number of pins and second number depends on wheter positions are calculated in 3d or 2d space.
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
    #endregion