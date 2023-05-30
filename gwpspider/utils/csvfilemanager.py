import csv
import numpy as np
from os import path

import spider
import config
from calculations import transformations as tf

class CsvFileManager():
    def __init__(self):
        self.FILENAME = 'data/leg_movement_data.csv'
        self.HEADERS = [
            'xA_before_0_x', 'xA_before_0_y', 'xA_before_0_z', 
            'xA_before_1_x', 'xA_before_1_y', 'xA_before_1_z',
            'xA_before_2_x', 'xA_before_2_y', 'xA_before_2_z',
            'xA_before_3_x', 'xA_before_3_y', 'xA_before_3_z',
            'xA_before_4_x', 'xA_before_4_y', 'xA_before_4_z',
            'roll', 'pitch', 'yaw',
            'leg_to_move_id',
            'xC_x', 'xC_y', 'xC_z',
            'number_of_tries',
            'xA_after_0_x', 'xA_after_0_y', 'xA_after_0_z', 
            'xA_after_1_x', 'xA_after_1_y', 'xA_after_1_z',
            'xA_after_2_x', 'xA_after_2_y', 'xA_after_2_z',
            'xA_after_3_x', 'xA_after_3_y', 'xA_after_3_z',
            'xA_after_4_x', 'xA_after_4_y', 'xA_after_4_z',
        ]

        if not path.exists(self.FILENAME):
            self.__write_header()
    
    def write_row(self, x_a_before, rpy, moving_leg_id, x_c, number_of_tries, x_a_after):
        """Write row into csv file.
        """
        row_data = self.__prepare_data(x_a_before, rpy, moving_leg_id, x_c, number_of_tries, x_a_after)
        with open(self.FILENAME, 'a', encoding = 'utf-8', newline = '') as file:
            writer = csv.writer(file)
            if len(row_data) == len(self.HEADERS):
                writer.writerow(row_data)
            else:
                return
    
    def __prepare_data(self, x_a_before: np.ndarray, rpy: list, moving_leg_id: int, x_c: np.ndarray, number_of_tries: int, x_a_after: np.ndarray) -> list:
        """Organize data in a list that can be written in csv file as a row.

        Args:
            x_a_before (np.ndarray): Positions of legs before movement.
            rpy (list): Roll, pitch and yaw of a spider's body.
            moving_leg_id (int): Id of a moving leg.
            x_c (np.ndarray): Commands given to a moving leg.
            number_of_tries (int): Number of tries in which leg successfuly grabed a pin.
            x_a_after (np.ndarray): Positions of legs after movement.

        Returns:
            list: List of data.
        """
        x_a_transformed_before, x_a_transformed_after, x_c_transformed = self.__transform_data(x_a_before, x_a_after, x_c, moving_leg_id)
        data_row = [x_a_transformed_before, rpy, moving_leg_id, x_c_transformed, number_of_tries, x_a_transformed_after]
        row = []
        for data in data_row:
            if isinstance(data, list):
                row += data
            else:
                try:
                    row += list(np.round(data.flatten(), 4))
                except TypeError:
                    row += [int(data)]
                except AttributeError:
                    row += [int(data)]

        return row

    def __transform_data(self, x_a_before: np.ndarray, x_a_after: np.ndarray, x_c: np.ndarray, moving_leg_id: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Rotate positions given in local origins into spider's origin.

        Args:
            x_a_before (np.ndarray): Positions of legs before movement.
            x_a_after (np.ndarray): Positions of legs after movement.
            x_c (np.ndarray): Commands given to a moving leg.
            moving_leg_id (int): Id of a moving leg.

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray]: Transformed positions.
        """
        x_a_transformed_before = np.zeros((5, 3))
        x_a_transformed_after = np.zeros((5, 3))

        for i in range(spider.NUMBER_OF_LEGS):
            x_a_transformed_before[i, :] = tf.transform_vector(x_a_before[i, :], config.LEG_ORIGIN, config.SPIDER_ORIGIN, moving_leg_id)
            x_a_transformed_after[i, :] = tf.transform_vector(x_a_after[i, :], config.LEG_ORIGIN, config.SPIDER_ORIGIN, moving_leg_id)

        x_c_transformed = tf.transform_vector(x_c[i, :], config.LEG_ORIGIN, config.SPIDER_ORIGIN, moving_leg_id)

        return x_a_transformed_before, x_a_transformed_after, x_c_transformed

    def __write_header(self):
        """Write header in csv file.
        """
        with open(self.FILENAME, 'x', encoding = 'utf-8', newline = '') as file:
            writer = csv.writer(file)
            writer.writerow(self.HEADERS)