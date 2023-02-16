"""File for storing global variables.
"""
CONTROLLER_FREQUENCY = 75.0
FORCE_DAMPING = 0.01
K_P_FORCE = 0.03
# K_P = 25.0
# K_D = 1.8
# K_ACC = 0.18
K_P = 10.0
K_D = 0.7
K_ACC = 0.13

SPIDER_ORIGIN = 'spider'
LEG_ORIGIN = 'local'
GLOBAL_ORIGIN = 'global'

BEZIER_TRAJECTORY = 'bezier'
MINJERK_TRAJECTORY = 'minJerk'

FORCE_DISTRIBUTION_DURATION = 1

FORCE_MODE = 'forceMode'
IMPEDANCE_MODE = 'impedanceMode'

WORKING_STATE = 'working'
RESTING_STATE = 'resting'
TRANSITION_STATE = 'transition'

WORKING_THREAD_NAME = 'working_thread'
TRANSITION_THREAD_NAME = 'transition_thread'
RESTING_THREAD_NAME = 'resting_thread'
DXL_WIRITNG_THREAD_NAME = 'dxl_writing_thread'
DXL_READING_THREAD_NAME = 'dxl_reading_thread'
SAFETY_THREAD_NAME = 'safety_thread'
CONVERTING_THREAD_NAME = 'converting_thread'
CONTROL_THREAD_NAME = 'control_thread'
UPDATE_DICT_THREAD_NAME = 'update_dict_thread'
UPDATE_DATA_THREAD_NAME = 'update_data_thread'
STATE_DICT_POSE_KEY = 'pose'
STATE_DICT_PINS_KEY = 'pins'
