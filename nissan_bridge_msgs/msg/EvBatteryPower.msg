# UDP packet ID 1 - Li-ion battery power limits and electrical state
# Source: FPGA_2_Host.EV_CAN_Msgs (CAN 0x1DB, 0x1DC)

std_msgs/Header header          # ROS receive time

# CAN 0x1DB
float64 ev_1db_timestamp
float64 ev_1db_lb_voltage
float64 ev_1db_lb_current
float64 ev_1db_lb_usable_soc

# CAN 0x1DC
float64 ev_1dc_timestamp
float64 ev_1dc_lb_charge_power_limit
float64 ev_1dc_lb_discharge_power_limit
