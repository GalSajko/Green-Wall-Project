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
    
    def plotWallGrid(self, plotPins = False):
        """Plot wall with pins.

        :param pattern: Wall grid pattern, defaults to 'square'.
        :param plotPins: Show figure if True, defaults to False
        """
        pins = np.transpose(self.wall.createGrid())
        self.board.plot(pins[0], pins[1], 'g.')
        if plotPins:
            plt.show()

    def plotSpidersPath(self, path, plotPath = False):
        """Plot path of the spider on the wall.

        :param path: (x, y) points on spiders path.
        :param plotPath: Show figure if True, defaults to False
        """
        path = np.transpose(path)
        self.board.plot(path[0], path[1], 'rx')
        if plotPath:
            self.plotWallGrid()
            plt.show()
    
    def plotSpiderMovement(self, path, legPositions):
        """Complete drawing function for showing a spiders movement on the wall.

        :param path: (x, y) points on the path.
        :param legPositions: Array of (x, y) positions of legs for each point on the path.
        """
        # First plot an environment (wall with pins) and path.
        self.plotWallGrid()
        self.plotSpidersPath(path)

        # Loop through each step on path.
        for idx, pose in enumerate(path):
            # Plot spiders body on each step.
            spiderBody = plt.Circle((pose[0], pose[1]), self.spider.BODY_RADIUS, color = "blue")
            self.board.add_patch(spiderBody)    
            # Plot all legs and their tips on each step.
            legs = []
            legTips = []    
            for i in range(len(legPositions[idx])):
                if len(pose) > 2:
                    T_GA = self.matrixCalculator.xyzRpyToMatrix(pose, True)
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
                spiderBody.remove()
                firstAnchor.remove()
                for i in range(len(legs)):
                    legTips[i].remove()
                    legs[i].remove()
        
        plt.show()
