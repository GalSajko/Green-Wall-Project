import csv
import numpy as np
import pandas as pd

FILENAME = 'leg_movement_data.csv'

headers = [
    'xA_before_0_x', 'xA_before_0_y', 'xA_before_0_z', 
    'xA_before_1_x', 'xA_before_1_y', 'xA_before_1_z',
    'xA_before_2_x', 'xA_before_2_y', 'xA_before_2_z',
    'xA_before_3_x', 'xA_before_3_y', 'xA_before_3_z',
    'xA_before_4_x', 'xA_before_4_y', 'xA_before_4_z',
    'roll_bno', 'pitch_bno', 'yaw_bno',
    'roll_calculated', 'pitch_calculated', 'yaw_calculated',
    'leg_to_move_id',
    'xC_x', 'xC_y', 'xC_z',
    'number_of_tries',
    'xA_after_0_x', 'xA_after_0_y', 'xA_after_0_z', 
    'xA_after_1_x', 'xA_after_1_y', 'xA_after_1_z',
    'xA_after_2_x', 'xA_after_2_y', 'xA_after_2_z',
    'xA_after_3_x', 'xA_after_3_y', 'xA_after_3_z',
    'xA_after_4_x', 'xA_after_4_y', 'xA_after_4_z',
]

if __name__ == '__main__':
    # with open('leg_movement_data.csv', 'w') as file:
    #     writer = csv.writer(file)
    #     writer.writerow(headers)
    data = pd.read_csv(FILENAME)
    
    print(data['number_of_tries'])
    print(data['leg_to_move_id'])
    print(f"XA BEFORE: {data['xA_before_4_x'][1]}, {data['xA_before_4_y'][1]}, {data['xA_before_4_z'][1]}")
    print(f"XC: {data['xC_x'][1]}, {data['xC_y'][1]}, {data['xC_z'][1]}")
    print(f"XA AFTER: {data['xA_after_4_x'][1]}, {data['xA_after_4_y'][1]}, {data['xA_after_4_z'][1]}")


