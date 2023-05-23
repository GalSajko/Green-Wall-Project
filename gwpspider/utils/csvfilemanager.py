import csv

class CsvFileManager():
    def __init__(self):
        self.FILENAME = 'leg_movement_data.csv'
        self.HEADERS = [
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
    
    def write_row(self, row_data: list):
        """Write row into csv file.

        Args:
            row_data (list): Row of data.
        """
        with open(self.FILENAME, 'a') as file:
            writer = csv.writer(file)
            if len(row_data) == len(self.HEADERS):
                writer.writerow(row_data)
            else:
                return
