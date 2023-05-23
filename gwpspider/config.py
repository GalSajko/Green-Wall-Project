"""File for storing global variables.
"""
CONTROLLER_FREQUENCY = 70.0
FORCE_DAMPING = 0.01
K_P_FORCE = 0.03
K_P = 25.0
K_D = 1.8
K_ACC = 0.18
F_D_PUSHING = 2.5

SPIDER_ORIGIN = 'spider'
LEG_ORIGIN = 'local'
GLOBAL_ORIGIN = 'global'

BEZIER_TRAJECTORY = 'bezier'
MINJERK_TRAJECTORY = 'minJerk'

FORCE_DISTRIBUTION_DURATION = 1.0

WORKING_STATE = 'working'
RESTING_STATE = 'resting'
TRANSITION_STATE = 'transition'

WORKING_THREAD_NAME = 'working_thread'
TRANSITION_THREAD_NAME = 'transition_thread'
RESTING_THREAD_NAME = 'resting_thread'
SAFETY_THREAD_NAME = 'safety_thread'
CONTROL_THREAD_NAME = 'control_thread'
UPDATE_DATA_THREAD_NAME = 'update_data_thread'
SEND_MESSAGE_DATA_THREAD_NAME = 'send_message_thread'
STATE_DICT_POSE_KEY = 'pose'
STATE_DICT_PINS_KEY = 'pins'

NUMBER_OF_WATERING_BEFORE_REFILL = 6
WATERING_TIME = 12.0
REFILL_TIME = 28.0

PROGRAM_KILL_KEY = 'k'

GRIPPER_BNO_ARDUINO_READING_PERIOD = 0.01

ENABLE_LEGS_COMMAND = 'enable'
DISABLE_LEGS_COMMAND = 'disable'


#TODO: Move those constants below to Wall repo (they will be included in shared package).

REQUEST_SENSOR_POSITION_ADDR = 'http://192.168.1.20:5000/zalij'
POST_SPIDER_POSITION_ADDR = 'http://192.168.1.20:5000/spiderPos'
POST_STATE_MESSAGE_ADDR = 'http://192.168.1.20:5000/message'

MICROSWITCH_ERROR = 'E01'
VOLTAGE_DROP_ERROR = 'E02'
STARTUP_SINGULARITY_ERROR = 'E03'
BREAKS_ACTIVATED_ERROR = 'E04'
ALL_LEGS_NOT_ATTACHED_ERROR = 'E05'
AUTO_CORRECITON_ERROR = 'E06'
GRIPPER_ERROR = 'E07'

ERROR_IN_MOTOR_WARNING = 'W01'
LEG_CLOSE_TO_SINGULARITY_WARNING = 'W02'
LEG_MISSED_PIN_WARNING = 'W03'
TEMPERATURES_IN_MOTORS_TOO_HIGH_WARNING = 'W04'

WORKING_PHASE_STARTED_MESSAGE = 'M01'
WORKING_PHASE_FINISHED_MESSAGE = 'M02'
TRANSITIONING_TO_RESTING_PHASE_MESSAGE = 'M03'
RESTING_PHASE_STARTED_MESSAGE = 'M04'
RESTING_PHASE_FINISHED_MESSAGE = 'M05'
AUTO_CORRECTION_SUCCESSFUL_MESSAGE = 'M06'
REFILLING_STARTED_MESSAGE = 'M07'
REFILLING_FINISHED_MESSAGE = 'M08'
WATERING_STARTED_MESSAGE = 'M09'
WATERING_FINISHED_MESSAGE = 'M10'
LEG_IN_MANUAL_CORRECTION_MODE_MESSAGE = 'M11'
REBOOTING_MOTORS_MESSAGE = 'M12'

MANUALLY_MOVE_LEG_ON_PIN_INSTRUCTION = 'I01'

#TODO: Add messages for enable-disable action.
STATUS_CODES_DICT = {
    MICROSWITCH_ERROR : 'Microswitch stucked.',
    #TODO: Not used yet.
    VOLTAGE_DROP_ERROR : 'Voltage droped below *.',
    STARTUP_SINGULARITY_ERROR : 'Possible singularity on startup.',
    #TODO: Not used yet.
    BREAKS_ACTIVATED_ERROR : 'Breaks are still activated.',
    ALL_LEGS_NOT_ATTACHED_ERROR : 'One or more legs are not attached. Cannot move selected leg.',
    AUTO_CORRECITON_ERROR : 'Auto-correction was not succesful. Entering manual-correction mode.',
    GRIPPER_ERROR : 'Gripper did not open correctly.',

    ERROR_IN_MOTOR_WARNING : 'Hardware error in motor.',
    #TODO: Not used yet.
    LEG_CLOSE_TO_SINGULARITY_WARNING : 'Leg is close to singularity.',
    LEG_MISSED_PIN_WARNING : 'Leg missed the pin. Entering auto-correction mode.',
    #TODO: Should that also be used during working?
    TEMPERATURES_IN_MOTORS_TOO_HIGH_WARNING : 'Temperature in one or more motors is above allowed value.',

    WORKING_PHASE_STARTED_MESSAGE : 'Working phase has started.',
    WORKING_PHASE_FINISHED_MESSAGE : 'Working phase has finished.',
    TRANSITIONING_TO_RESTING_PHASE_MESSAGE : 'Transitioning into resting phase.',
    RESTING_PHASE_STARTED_MESSAGE : 'Resting phase has started.',
    RESTING_PHASE_FINISHED_MESSAGE : 'Resting phase has finished.',
    AUTO_CORRECTION_SUCCESSFUL_MESSAGE : 'Auto-correction was successful.',
    REFILLING_STARTED_MESSAGE : 'Refilling the water tank has started.',
    REFILLING_FINISHED_MESSAGE : 'Refilling the water tank has finished.',
    WATERING_STARTED_MESSAGE : 'Watering has started.',
    WATERING_FINISHED_MESSAGE : 'Watering has finished.',
    LEG_IN_MANUAL_CORRECTION_MODE_MESSAGE : 'Leg is in manual-correction mode.',
    REBOOTING_MOTORS_MESSAGE : 'Rebooting motors in error.',
}

def create_instruction_message(*instruction_args) -> str:
    """Create instruction message, based on given parameters. First parameter should always be the instruction code. Following parameters
    are specific for each instruction:
    - MANUALLY_MOVE_LEG_ON_PIN_INSTRUCTION: 2nd parameter is leg id, 3rd is pin id.

    Returns:
        string: Instruction message.
    """
    return ','.join(map(str, list(instruction_args)))

 