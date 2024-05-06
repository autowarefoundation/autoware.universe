# Vedder VESC open source motor controller state (telemetry)

# fault codes
int32 FAULT_CODE_NONE=0
int32 FAULT_CODE_OVER_VOLTAGE=1
int32 FAULT_CODE_UNDER_VOLTAGE=2
int32 FAULT_CODE_DRV8302=3
int32 FAULT_CODE_ABS_OVER_CURRENT=4
int32 FAULT_CODE_OVER_TEMP_FET=5
int32 FAULT_CODE_OVER_TEMP_MOTOR=6

# follow the bledc firwmare: commands.c
float64 temp_fet             # fet temperature 
float64 temp_motor           # motor temperature
float64 current_motor        # motor current (ampere) avg_motor_current
float64 current_input        # input current (ampere) avg_input_current          
float64 avg_id
float64 avg_iq
float64 duty_cycle           # duty cycle (0 to 1) duty_cycle_now
float64 speed                # motor electrical speed (revolutions per minute) rpm

float64 voltage_input        # input voltage (volt)
float64 charge_drawn         # electric charge drawn from input (ampere-hours)
float64 charge_regen         # electric charge regenerated to input (ampere-hour) amp_hours_charged    
float64 energy_drawn         # energy drawn from input (watt-hour)
float64 energy_regen         # energy regenerated to input (watt_hours_charged)
int32   displacement         # net tachometer (counts) tachometer
int32   distance_traveled    # total tachnometer (counts) tachometer_abs
int32   fault_code

float64 pid_pos_now
int32 controller_id


float64 ntc_temp_mos1
float64 ntc_temp_mos2
float64 ntc_temp_mos3
float64 avg_vd
float64 avg_vq

#float64 temperature_pcb      # temperature of printed circuit board (degrees Celsius)
