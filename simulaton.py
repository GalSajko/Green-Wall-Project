""" Module for simulating Green Wall environment and Spiders movement. """

from multiprocessing.sharedctypes import Value
import matplotlib.pyplot as plt
import numpy as np

import environment
import calculations

class Plotter:
    """Class for ploting a 2d representation of a wall and spiders movement.
    """
    def __init__(self, gridPattern):
        self.wall = environment.Wall(gridPattern)
        self.spider = environment.Spider()
        self.matrixCalculator = calculations.MatrixCalculator()
        # Figure for plotting.
        self.figure = plt.figure()
        self.board = plt.axes(xlim = (0, self.wall.WALL_SIZE[0]), ylim = (0, self.wall.WALL_SIZE[1]))
        self.board.set_aspect('equal')

    def plotWallGrid(self, plotPins = False):
        """Plot 2-d representation of wall with pins.

        Args:
            plotPins (bool, optional): Show graph if True. Defaults to False.
        """
        pins = np.transpose(self.wall.createGrid())
        self.board.plot(pins[0], pins[1], 'g.')
        if plotPins:
            plt.show()

    def plotSpidersPath(self, path, plotPath = False):
        """Plot 2-d representation of spider's path.

        Args:
            path (list): Array of spider's body path. Only x and y values are used for plotting.
            plotPath (bool, optional): Show plotting figure if True. Defaults to False.
        """
        path = np.transpose(path)
        self.board.plot(path[0], path[1], 'rx')
        if plotPath:
            self.plotWallGrid()
            plt.show()

    def plotSpiderMovement(self, path, legPositions, allPotentialPins = None):
        """2-d animation of spider's movement between two points on the wall.

        Args:
            path (list): nxm array of spider's body path, where n is number of steps. Only x and y values are used for plotting.
            legPositions (list): nx5x3 array of global legs positions on each step of the path.
            allPotentialPins(list): Array of all potential pins for each leg on each step. If value is not None, potential pins will also be visualized
            with different colors and sizes for each leg. Defaults to None.
        """
        # First plot an environment (wall with pins) and path.
        self.plotWallGrid()
        self.plotSpidersPath(path)

        # Loop through each step on path.
        for idx, pose in enumerate(path):
            # Plot spiders body on each step.
            spiderBody = plt.Circle((pose[0], pose[1]), self.spider.BODY_RADIUS, color = "blue")
            self.board.add_patch(spiderBody) 

            if allPotentialPins is not None:
                potentialCircles = []
                for i, potentials in enumerate(allPotentialPins[idx]):
                    if i == 0:
                        c = 'yellow'
                        r = 0.1
                    elif i == 1:
                        c = 'blue'
                        r = 0.08
                    elif i == 2:
                        c = 'green'
                        r = 0.06
                    elif i == 3:
                        c = 'magenta'
                        r = 0.04
                    elif i == 4:
                        c = 'red'
                        r = 0.04
                    for pin in potentials:
                        potentialCircle = plt.Circle((pin[0], pin[1]), r, color = c)
                        self.board.add_patch(potentialCircle)
                        potentialCircles.append(potentialCircle)

            # Plot all legs and their tips on each step.
            legs = []
            legTips = []    
            for i in range(len(legPositions[idx])):
                if len(pose) > 2:
                    T_GA = self.matrixCalculator.xyzRpyToMatrix(pose)
                    anchorPosition = np.dot(T_GA, self.spider.T_ANCHORS[i])[:,3][:3]
                    xVals = [anchorPosition[0], legPositions[idx][i][0]]
                    yVals = [anchorPosition[1], legPositions[idx][i][1]]
                else:
                    xVals = [pose[0] + self.spider.LEG_ANCHORS[i][0], legPositions[idx][i][0]]
                    yVals = [pose[1] + self.spider.LEG_ANCHORS[i][1], legPositions[idx][i][1]]
                legs.append(self.board.plot(xVals, yVals, 'g')[0])
                # Mark 1st leg with red tip and 2nd leg with yellow (to check legs orientation)
                color = "blue"
                if i == 0:
                    color = "red"
                    firstAnchor = plt.Circle((anchorPosition[0], anchorPosition[1]), 0.03, color = "red")
                    self.board.add_patch(firstAnchor)
                elif i == 1:
                    color = "yellow"
                legTip = plt.Circle((legPositions[idx][i]), 0.02, color = color)
                self.board.add_patch(legTip)
                legTips.append(legTip)

            plt.draw()
            plt.pause(0.5)

            # Remove all drawn components from board, unless spider is at the end of the path.
            if (pose - path[-1]).any():
                if allPotentialPins is not None:
                    for potentialCircle in potentialCircles:
                        potentialCircle.remove()
                spiderBody.remove()
                firstAnchor.remove()
                for i in range(len(legs)):
                    legTips[i].remove()
                    legs[i].remove()
        
        plt.show()
