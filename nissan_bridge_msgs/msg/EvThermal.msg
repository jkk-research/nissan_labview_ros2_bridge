# UDP packet ID 2 - Temperatures, SOC and battery health
# Source: FPGA_2_Host.EV_CAN_Msgs (0x54A, 0x54C, 0x54F, 0x55A, 0x55B, 0x5BC)
#         FPGA_2_Host.CAR_CAN_Msgs (0x510, 0x5B3)

std_msgs/Header header

# EV 0x54A
float64 ev_54a_timestamp
float64 ev_54a_ambient_temp_ac

# EV 0x54C
float64 ev_54c_timestamp
float64 ev_54c_ac_evaporator_temperature

# EV 0x54F
float64 ev_54f_timestamp
float64 ev_54f_interior_intake_temp

# EV 0x55A
float64 ev_55a_timestamp
float64 ev_55a_motor_temperature
float64 ev_55a_igbt_temperature
float64 ev_55a_igbt_driver_board_temperature
float64 ev_55a_inverter_com_board_temp

# EV 0x55B
float64 ev_55b_timestamp
float64 ev_55b_lb_soc

# EV 0x5BC
float64 ev_5bc_timestamp
float64 ev_5bc_lb_capacity_deterioration_rate
float64 ev_5bc_lb_remain_capacity_gids
float64 ev_5bc_lb_temperature_segment_for_dash

# CAR 0x510
float64 car_510_timestamp
float64 car_510_outside_ambient_temperature

# CAR 0x5B3
float64 car_5b3_timestamp
float64 car_5b3_battery_state_of_health
float64 car_5b3_battery_gids
float64 car_5b3_battery_pack_temperature
