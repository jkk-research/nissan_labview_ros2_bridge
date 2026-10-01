# UDP packet ID 4 - Vehicle motion state and control feedback
# Source: RT/AUT time + FPGA_2_Host.CAR_CAN_Msgs (0x284, 0x002, 0x358)

std_msgs/Header header

# Timing
float64 rt_time
float64 aut_time

# CAR 0x284
float64 car_284_timestamp
float64 car_284_vehicle_speed_from_abs

# CAR 0x002 - sign already flipped on the cRIO (*-1) for the Autoware convention
float64 car_002_timestamp
float64 car_002_steering_angle

# Pedal states (currently constant 0.5 during development)
float64 accel_pedal_state
float64 brake_pedal_state

# Control state feedback (LabVIEW enum, valid range 1..5)
uint8 control_state

# CAR 0x358
float64 car_358_timestamp
uint8 car_358_turn_signal_status

uint8 TURN_SIGNAL_OFF     = 0      # 0x0 BothOff
uint8 TURN_SIGNAL_LEFT    = 4      # 0x4 Left Signal ON
uint8 TURN_SIGNAL_RIGHT   = 8      # 0x8 Right Signal ON
uint8 TURN_SIGNAL_HAZARD  = 12     # 0xC Both Signals ON (Hazards)
