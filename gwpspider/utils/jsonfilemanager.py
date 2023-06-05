import json
import numpy as np
from gwpconfig import wall
import os

import config

class JsonFileManager():
    def __init__(self):
        self.FILENAME = os.path.abspath('gwpspider/data/spider_state_dict')

        self.pins = wall.create_grid(True)
        self.state_dict = {
            'pose' : [],
            'pins' : []
        } 

    def update_whole_dict(self, pose: list, current_pins: list, legs_moving_order: list):
        """Update pose and pins in dictionary and write it in JSON file.

        Args:
            pose (list): Spider's pose.
            current_pins (list): 5x3 array of pins positions.
            legs_moving_order (list): Legs moving order.
        """
        sorted_legs_ids = np.argsort(legs_moving_order)
        sorted_pins = current_pins[sorted_legs_ids]

        pins_ids = []
        for used_pin in sorted_pins:
            for pin_id, pin in enumerate(self.pins):
                if pin.tolist() == used_pin.tolist():
                    pins_ids.append(pin_id)

        self.state_dict['pose'] = pose.tolist()
        self.state_dict['pins'] = pins_ids

        self.__write_json()
    
    def update_pins(self, leg_id: int, pin: list):
        """Update only one pin position in dictionary and write it in JSON file.

        Args:
            leg_id (int): Leg id.
            pin (list): 1x3 array of pin position.
        """
        pin_id = np.flatnonzero((self.pins == pin).all(1))[0]
        self.state_dict['pins'][leg_id] = int(pin_id)

        self.__write_json()
    
    def read_spider_state(self):
        """Read pose and pins from JSON file and save them into dictionary.

        Returns:
            tuple: Spider's pose, used pins indexes and used pins positions.
        """
        with open(self.FILENAME, 'r', encoding = 'utf-8') as file:
            self.state_dict = json.load(file)
        
        pose = np.array(self.state_dict[config.STATE_DICT_POSE_KEY])
        pins_ids = np.array(self.state_dict[config.STATE_DICT_PINS_KEY])
        pins_positions = self.pins[pins_ids]

        return pose, pins_ids, pins_positions

    def __write_json(self):
        with open(self.FILENAME, 'w', encoding = 'utf-8') as file:
            json.dump(self.state_dict, file)
