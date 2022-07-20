"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
import calculations
import environment as env
import planning
import simulaton

if __name__ == "__main__":    
    wall = env.Wall('squared')
    pins = wall.createGrid(True)
    pathPlanner = planning.PathPlanner(0.05, 0.1)
    plotter = simulaton.Plotter('squared')
    start = [0.5, 0.5, 0.2, 0.0]
    goal = [0.5, 0.5, 0.2, 0.0]
    path = pathPlanner.calculateSpiderBodyPath(start, goal)
    pins, rValues = pathPlanner.calculateIdealLegsPositionsFF(path)
    print(rValues)
    plotter.plotSpiderMovement(path, pins)





    