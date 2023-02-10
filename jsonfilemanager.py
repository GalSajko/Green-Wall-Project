import json
import numpy as np

import config
from environment import wall

class JsonFileManager():
    def __init__(self):
        self.FILENAME = 'spider_state_dict'

        self.pins = wall.createGrid(True)
        self.stateDict = {
            'pose' : [],
            'pins' : []
        }     

    def updateWholeDict(self, pose, currentPins, legsOrder):
        """Update pose and pins in dictionary and write it in JSON file.

        Args:
            pose (list): Spider's pose.
            currentPins (list): 5x3 array of pins positions.
            legsOrder (list): Legs moving order.
        """
        sortedLegsIndexes = np.argsort(legsOrder)
        sortedPins = currentPins[sortedLegsIndexes]

        pinsIndexes = []
        for usedPin in sortedPins:
            for pinIndex, pin in enumerate(self.pins):
                if pin.tolist() == usedPin.tolist():
                    pinsIndexes.append(pinIndex)

        self.stateDict['pose'] = pose.tolist()
        self.stateDict['pins'] = pinsIndexes

        self.__writeJson()
    
    def updatePins(self, legId, pin):
        """Update only one pin position in dictionary and write it in JSON file.

        Args:
            legId (int): Leg id.
            pin (list): 1x3 array of pin position.
        """
        pinIndex = np.flatnonzero((self.pins == pin).all(1))[0]
        self.stateDict['pins'][legId] = int(pinIndex)

        self.__writeJson()
    
    def readPosePins(self):
        """Read pose and pins from JSON file and save them into dictionary.

        Returns:
            _type_: _description_
        """
        with open(self.FILENAME, 'r', encoding = 'utf-8') as file:
            self.stateDict = json.load(file)
        
        pose = np.array(self.stateDict[config.STATE_DICT_POSE_KEY])
        pinsIndexes = np.array(self.stateDict[config.STATE_DICT_PINS_KEY])
        pinsPositions = self.pins[pinsIndexes]

        return pose, pinsIndexes, pinsPositions

    def __writeJson(self):
        with open(self.FILENAME, 'w', encoding = 'utf-8') as file:
            json.dump(self.stateDict, file)
