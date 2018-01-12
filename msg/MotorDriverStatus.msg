#action state
uint8 STANDBY = 0
uint8 RUNNING = 1
uint8 PAUSING = 2

#system error
uint8 NO_ERROR = 0
uint8 MOTOR_DRIVER_ERROR = 1
uint8 CONTROL_PARAM_ERROR = 2
uint8 MOTOR_SPEED_BEYOND = 4
uint8 UNKNOWN_COMMAND = 8
uint8 COMMAND_BUFFER_EMPTY = 16
uint8 COMMAND_BUFFER_FULL = 32
uint8 RESERVED_1 = 64
uint8 RESERVED_2 = 128

#motor error
uint8 MOTOR_ERROR = 0
uint8 MOTOR_NORMAL =1 

uint8 action_state
uint8 system_error
uint8 motor_error
uint8 actionbuffer_rest_action
uint8 actionbuffer_current_address
uint8 commandbuffer_rest_command
uint8 commandbuffer_current_address
