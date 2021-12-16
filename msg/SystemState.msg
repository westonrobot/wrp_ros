# system error code
uint32 NONE = 0
#   // warning
uint32 BATTERY_LOW_WARN = 1
uint32 MOTOR_OVERHEAT_WARN = 2
uint32 DRIVER_OVERHEAT_WARN = 4
uint32 WARN_BIT_MASK = 4095
#   // fault
uint32 BATTERY_LOW_FAULT = 65536
uint32 MOTOR_OVERHEAT_FAULT = 131072
uint32 DRIVER_OVERHEAT_FAULT = 262144
uint32 MOTOR_COMM_FAULT = 524288
uint32 FAULT_BIT_MASK = 268369920

# system oper state
uint8 OPERATIONAL = 0
uint8  INITIALIZATION = 1
uint8  MAINTENANCE = 2
uint8  SOFTWARE_UPDATE = 3
uint8  ESTOP_ACTIVATED = 4
uint8  HARDWARE_FAULT = 5

# system control state
uint8  UNINITIALIZED = 0
uint8  STANDBY = 1
uint8  RC_HALT_TRIGGERED = 2
uint8  RC_MANUAL_CONTROL = 3
uint8  CAN_COMMAND_CONTROL = 4

bool rc_connected
uint32  error_code
int32  operational_state
int32  control_state
