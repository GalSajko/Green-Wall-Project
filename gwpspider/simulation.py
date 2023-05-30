""" Module for simulating Green Wall environment and Spiders movement. """
import matplotlib.pyplot as plt
import numpy as np
from gwpconfig import wall

import spider  
from calculations import transformations as tf

class Plotter:
    """Class for ploting a 2d representation of a wall and spiders movement.
    """
    def __init__(self):
        self.figure = plt.figure()
        self.board = plt.axes(xlim = (0, wall.WALL_SIZE[0]), ylim = (0, wall.WALL_SIZE[1]))
        self.board.set_aspect('equal')

    def plot_wall_grid(self, plotPins = False):
        """Plot 2-d representation of wall with pins.

        Args:
            plotPins (bool, optional): Show graph if True. Defaults to False.
        """
        pins = np.transpose(wall.createGrid(False))
        self.board.plot(pins[0], pins[1], 'g.')
        if plotPins:
            plt.show()

    def plot_spider_path(self, path, do_plot_path = False):
        """Plot 2-d representation of spider's path.

        Args:
            path (list): Array of spider's body path. Only x and y values are used for plotting.
            do_plot_path (bool, optional): Show plotting figure if True. Defaults to False.
        """
        path = np.transpose(path)
        self.board.plot(path[0], path[1], 'rx')
        if do_plot_path:
            self.plot_wall_grid()
            plt.show()

    def plotSpiderMovement(self, path, legs_positions, all_potential_pins = None):
        """2-d animation of spider's movement between two points on the wall.

        Args:
            path (list): nxm array of spider's body path, where n is number of steps. Only x and y values are used for plotting.
            legs_positions (list): nx5x3 array of global legs positions on each step of the path.
            all_potential_pins(list): Array of all potential pins for each leg on each step. If value is not None, potential pins will also be visualized
            with different colors and sizes for each leg. Defaults to None.
        """
        # First plot an environment (wall with pins) and path.
        self.plot_wall_grid()
        self.plot_spider_path(path)

        # Loop through each step on path.
        for idx, pose in enumerate(path):
            # Plot spiders body on each step.
            spider_body = plt.Circle((pose[0], pose[1]), spider.BODY_RADIUS, color = "blue")
            self.board.add_patch(spider_body) 

            if all_potential_pins is not None:
                potential_pins_circles = []
                for i, potentials in enumerate(all_potential_pins[idx]):
                    if i == 0:
                        pins_color = 'red'
                        pins_radius = 0.1
                    elif i == 1:
                        pins_color = 'yellow'
                        pins_radius = 0.08
                    elif i == 2:
                        pins_color = 'green'
                        pins_radius = 0.06
                    elif i == 3:
                        pins_color = 'magenta'
                        pins_radius = 0.04
                    elif i == 4:
                        pins_color = 'blue'
                        pins_radius = 0.04
                    for pin in potentials:
                        potential_pins_circle = plt.Circle((pin[0], pin[1]), pins_radius, color = pins_color)
                        self.board.add_patch(potential_pins_circle)
                        potential_pins_circles.append(potential_pins_circle)

            # Plot all legs and their tips on each step.
            legs = []
            leg_tips_circles = []    
            for i in range(len(legs_positions[idx])):
                if len(pose) > 2:
                    T_GA = tf.xyzrpy_to_matrix(pose)
                    leg_base_position = np.dot(T_GA, spider.T_BASES[i])[:,3][:3]
                    x_values = [leg_base_position[0], legs_positions[idx][i][0]]
                    y_values = [leg_base_position[1], legs_positions[idx][i][1]]
                else:
                    x_values = [pose[0] + spider.LEG_BASES[i][0], legs_positions[idx][i][0]]
                    y_values = [pose[1] + spider.LEG_BASES[i][1], legs_positions[idx][i][1]]
                legs.append(self.board.plot(x_values, y_values, 'g')[0])
                # Mark 1st leg with red tip and 2nd leg with yellow (to check legs orientation)
                leg_tip_color = "orange"
                if i == 0:
                    leg_tip_color = "red"
                    first_leg_base_circle = plt.Circle((leg_base_position[0], leg_base_position[1]), 0.03, color = leg_tip_color)
                    self.board.add_patch(first_leg_base_circle)
                elif i == 1:
                    leg_tip_color = "yellow"
                leg_tip_circle = plt.Circle((legs_positions[idx][i]), 0.02, color = leg_tip_color)
                self.board.add_patch(leg_tip_circle)
                leg_tips_circles.append(leg_tip_circle)

            plt.draw()
            plt.pause(0.2)

            # Remove all drawn components from board, unless spider is at the end of the path.
            if (pose - path[-1]).any():
                if all_potential_pins is not None:
                    for potential_pins_circle in potential_pins_circles:
                        potential_pins_circle.remove()
                spider_body.remove()
                first_leg_base_circle.remove()
                for i in range(len(legs)):
                    leg_tips_circles[i].remove()
                    legs[i].remove()
        
        plt.show()
    
    def plot_spider_movement_in_steps(self, path, pins_instructions):
        """2-d animation of spider's movement on the wall, with animated steps of each leg.

        Args:
            path (list): nxm array of spider's body poses along the way. Only x and y values are used for animation.
            pins_instructions (list): nx5x3 array of legs positions on each step of the path.
        """
        import string
        import random

        self.plot_wall_grid()
        self.plot_spider_path(path)

        legs = list(range(spider.NUMBER_OF_LEGS))

        def plot_legs(step, pose, in_loop_pause, do_delete_legs):
            for i in range(len(pins_instructions[step])):
                leg_idx = int(pins_instructions[step][i][0])
                leg_position = pins_instructions[step][i][1:]

                if do_delete_legs:
                    legs[leg_idx].remove()
                if len(pose) > 2:
                    T_GA = tf.xyzrpy_to_matrix(pose)
                    leg_base_position = np.dot(T_GA, spider.T_BASES[leg_idx])[:,3][:3]
                    x_values = [leg_base_position[0], leg_position[0]]
                    y_values = [leg_base_position[1], leg_position[1]]
                else:
                    x_values = [pose[0] + spider.LEG_BASES[leg_idx][0], leg_position[0]]
                    y_values = [pose[1] + spider.LEG_BASES[leg_idx][1], leg_position[1]]
                legs[leg_idx] = self.board.plot(x_values, y_values, 'g')[0]
                if in_loop_pause:
                    plt.draw()
                    plt.savefig('slike/' + ''.join(random.choice(string.ascii_lowercase) for i in range(10)) + '.jpg', dpi=500)
                    if (leg_position - pins_instructions[step - 1][i][1:]).any():
                        plt.pause(0.5)

            if not in_loop_pause:
                plt.draw()
                plt.savefig('slike/' + ''.join(random.choice(string.ascii_lowercase) for i in range(10)) + '.jpg', dpi=500)
                plt.pause(0.75)
            

        for step, pose in enumerate(path):
            spider_body = plt.Circle((pose[0], pose[1]), spider.BODY_RADIUS, color = 'blue')
            self.board.add_patch(spider_body)
            
            if step == 0:
                plot_legs(step, pose, False, False)
            else:
                plot_legs(step - 1, pose, False, True)
                plot_legs(step, pose, True, True)

            if (pose - path[-1]).any():
                spider_body.remove()
  
        plt.show()
        