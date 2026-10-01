# UDP packet ID 3 - Battery capacity and historical statistics
# Source: FPGA_2_Host.EV_CAN_Msgs (0x59E, 0x5C0)

std_msgs/Header header

# EV 0x59E
float64 ev_59e_timestamp
float64 ev_59e_lb_full_capacity_for_qc

# EV 0x5C0 - LB_HistData
float64 ev_5c0_timestamp
float64 ev_5c0_degr_int_res_coeff_min
float64 ev_5c0_degr_int_res_coeff_max
float64 ev_5c0_degr_int_res_coeff_avg
float64 ev_5c0_cell_voltage_min
float64 ev_5c0_cell_voltage_max
float64 ev_5c0_cell_voltage_avg
float64 ev_5c0_temperature_min
float64 ev_5c0_temperature_max
float64 ev_5c0_temperature_avg
float64 ev_5c0_temp_wakeup_phase_min
float64 ev_5c0_temp_wakeup_phase_max
float64 ev_5c0_temp_wakeup_phase_avg
