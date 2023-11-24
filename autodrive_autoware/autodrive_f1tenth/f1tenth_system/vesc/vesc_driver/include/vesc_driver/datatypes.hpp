// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


/*
  Some parts of this code, Copyright 2016 - 2019 Benjamin Vedder    benjamin@vedder.se

  This file is part of the VESC firmware.

  The VESC firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  The VESC firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  */

#ifndef VESC_DRIVER__DATATYPES_HPP_
#define VESC_DRIVER__DATATYPES_HPP_

#include <stdint.h>
#include <stdbool.h>

#include <cstdint>

namespace vesc_driver
{

typedef struct
{
  bool isVesc;
}
VSerialInfo_t;

typedef enum
{
  CFG_T_UNDEFINED = 0,
  CFG_T_DOUBLE,
  CFG_T_INT,
  CFG_T_QSTRING,
  CFG_T_ENUM,
  CFG_T_BOOL
}
CFG_T;

typedef enum
{
  VESC_TX_UNDEFINED = 0,
  VESC_TX_UINT8,
  VESC_TX_INT8,
  VESC_TX_UINT16,
  VESC_TX_INT16,
  VESC_TX_UINT32,
  VESC_TX_INT32,
  VESC_TX_DOUBLE16,
  VESC_TX_DOUBLE32,
  VESC_TX_DOUBLE32_AUTO
} VESC_TX_T;

// #include "ch.h"
typedef uint32_t systime_t;    // defined in ch.h

// Data types
typedef enum
{
  MC_STATE_OFF = 0,
  MC_STATE_DETECTING,
  MC_STATE_RUNNING,
  MC_STATE_FULL_BRAKE,
} mc_state;

typedef enum
{
  PWM_MODE_NONSYNCHRONOUS_HISW = 0,    // This mode is not recommended
  PWM_MODE_SYNCHRONOUS,                // The recommended and most tested mode
  PWM_MODE_BIPOLAR                     // Some glitches occasionally, can kill MOSFETs
} mc_pwm_mode;

typedef enum
{
  COMM_MODE_INTEGRATE = 0,
  COMM_MODE_DELAY
} mc_comm_mode;

typedef enum
{
  SENSOR_MODE_SENSORLESS = 0,
  SENSOR_MODE_SENSORED,
  SENSOR_MODE_HYBRID
} mc_sensor_mode;

typedef enum
{
  FOC_SENSOR_MODE_SENSORLESS = 0,
  FOC_SENSOR_MODE_ENCODER,
  FOC_SENSOR_MODE_HALL,
  FOC_SENSOR_MODE_HFI
} mc_foc_sensor_mode;

// Auxiliary output mode
typedef enum
{
  OUT_AUX_MODE_OFF = 0,
  OUT_AUX_MODE_ON_AFTER_2S,
  OUT_AUX_MODE_ON_AFTER_5S,
  OUT_AUX_MODE_ON_AFTER_10S,
  OUT_AUX_MODE_UNUSED
} out_aux_mode;

// Temperature sensor type
typedef enum
{
  TEMP_SENSOR_NTC_10K_25C = 0,
  TEMP_SENSOR_PTC_1K_100C,
  TEMP_SENSOR_KTY83_122,
} temp_sensor_type;

// General purpose drive output mode
typedef enum
{
  GPD_OUTPUT_MODE_NONE = 0,
  GPD_OUTPUT_MODE_MODULATION,
  GPD_OUTPUT_MODE_VOLTAGE,
  GPD_OUTPUT_MODE_CURRENT
} gpd_output_mode;

typedef enum
{
  MOTOR_TYPE_BLDC = 0,
  MOTOR_TYPE_DC,
  MOTOR_TYPE_FOC,
  MOTOR_TYPE_GPD
} mc_motor_type;

// FOC current controller decoupling mode.
typedef enum
{
  FOC_CC_DECOUPLING_DISABLED = 0,
  FOC_CC_DECOUPLING_CROSS,
  FOC_CC_DECOUPLING_BEMF,
  FOC_CC_DECOUPLING_CROSS_BEMF
} mc_foc_cc_decoupling_mode;

typedef enum
{
  FOC_OBSERVER_ORTEGA_ORIGINAL = 0,
  FOC_OBSERVER_ORTEGA_ITERATIVE
} mc_foc_observer_type;

typedef enum
{
  FAULT_CODE_NONE = 0,
  FAULT_CODE_OVER_VOLTAGE,
  FAULT_CODE_UNDER_VOLTAGE,
  FAULT_CODE_DRV,
  FAULT_CODE_ABS_OVER_CURRENT,
  FAULT_CODE_OVER_TEMP_FET,
  FAULT_CODE_OVER_TEMP_MOTOR,
  FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,
  FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,
  FAULT_CODE_MCU_UNDER_VOLTAGE,
  FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,
  FAULT_CODE_ENCODER_SPI,
  FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,
  FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,
  FAULT_CODE_FLASH_CORRUPTION,
  FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,
  FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,
  FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,
  FAULT_CODE_UNBALANCED_CURRENTS,
  FAULT_CODE_BRK,
  FAULT_CODE_RESOLVER_LOT,
  FAULT_CODE_RESOLVER_DOS,
  FAULT_CODE_RESOLVER_LOS
} mc_fault_code;

typedef enum
{
  CONTROL_MODE_DUTY = 0,
  CONTROL_MODE_SPEED,
  CONTROL_MODE_CURRENT,
  CONTROL_MODE_CURRENT_BRAKE,
  CONTROL_MODE_POS,
  CONTROL_MODE_HANDBRAKE,
  CONTROL_MODE_OPENLOOP,
  CONTROL_MODE_OPENLOOP_PHASE,
  CONTROL_MODE_OPENLOOP_DUTY,
  CONTROL_MODE_OPENLOOP_DUTY_PHASE,
  CONTROL_MODE_NONE
} mc_control_mode;

typedef enum
{
  DISP_POS_MODE_NONE = 0,
  DISP_POS_MODE_INDUCTANCE,
  DISP_POS_MODE_OBSERVER,
  DISP_POS_MODE_ENCODER,
  DISP_POS_MODE_PID_POS,
  DISP_POS_MODE_PID_POS_ERROR,
  DISP_POS_MODE_ENCODER_OBSERVER_ERROR
} disp_pos_mode;

typedef enum
{
  SENSOR_PORT_MODE_HALL = 0,
  SENSOR_PORT_MODE_ABI,
  SENSOR_PORT_MODE_AS5047_SPI,
  SENSOR_PORT_MODE_AD2S1205,
  SENSOR_PORT_MODE_SINCOS,
  SENSOR_PORT_MODE_TS5700N8501,
  SENSOR_PORT_MODE_TS5700N8501_MULTITURN
} sensor_port_mode;

typedef struct
{
  float cycle_int_limit;
  float cycle_int_limit_running;
  float cycle_int_limit_max;
  float comm_time_sum;
  float comm_time_sum_min_rpm;
  int32_t comms;
  float time_at_comm;
} mc_rpm_dep_struct;

typedef enum
{
  DRV8301_OC_LIMIT = 0,
  DRV8301_OC_LATCH_SHUTDOWN,
  DRV8301_OC_REPORT_ONLY,
  DRV8301_OC_DISABLED
} drv8301_oc_mode;

typedef enum
{
  DEBUG_SAMPLING_OFF = 0,
  DEBUG_SAMPLING_NOW,
  DEBUG_SAMPLING_START,
  DEBUG_SAMPLING_TRIGGER_START,
  DEBUG_SAMPLING_TRIGGER_FAULT,
  DEBUG_SAMPLING_TRIGGER_START_NOSEND,
  DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND,
  DEBUG_SAMPLING_SEND_LAST_SAMPLES
} debug_sampling_mode;

typedef enum
{
  CAN_BAUD_125K = 0,
  CAN_BAUD_250K,
  CAN_BAUD_500K,
  CAN_BAUD_1M,
  CAN_BAUD_10K,
  CAN_BAUD_20K,
  CAN_BAUD_50K,
  CAN_BAUD_75K
} CAN_BAUD;

typedef enum
{
  BATTERY_TYPE_LIION_3_0__4_2,
  BATTERY_TYPE_LIIRON_2_6__3_6,
  BATTERY_TYPE_LEAD_ACID
} BATTERY_TYPE;

typedef enum
{
  HFI_SAMPLES_8 = 0,
  HFI_SAMPLES_16,
  HFI_SAMPLES_32
} FOC_HFI_SAMPLES;

typedef struct
{
  // Switching and drive
  mc_pwm_mode pwm_mode;
  mc_comm_mode comm_mode;
  mc_motor_type motor_type;
  mc_sensor_mode sensor_mode;
  // Limits
  float l_current_max;
  float l_current_min;
  float l_in_current_max;
  float l_in_current_min;
  float l_abs_current_max;
  float l_min_erpm;
  float l_max_erpm;
  float l_erpm_start;
  float l_max_erpm_fbrake;
  float l_max_erpm_fbrake_cc;
  float l_min_vin;
  float l_max_vin;
  float l_battery_cut_start;
  float l_battery_cut_end;
  bool l_slow_abs_current;
  float l_temp_fet_start;
  float l_temp_fet_end;
  float l_temp_motor_start;
  float l_temp_motor_end;
  float l_temp_accel_dec;
  float l_min_duty;
  float l_max_duty;
  float l_watt_max;
  float l_watt_min;
  float l_current_max_scale;
  float l_current_min_scale;
  float l_duty_start;
  // Overridden limits (Computed during runtime)
  float lo_current_max;
  float lo_current_min;
  float lo_in_current_max;
  float lo_in_current_min;
  float lo_current_motor_max_now;
  float lo_current_motor_min_now;
  // Sensorless (bldc)
  float sl_min_erpm;
  float sl_min_erpm_cycle_int_limit;
  float sl_max_fullbreak_current_dir_change;
  float sl_cycle_int_limit;
  float sl_phase_advance_at_br;
  float sl_cycle_int_rpm_br;
  float sl_bemf_coupling_k;
  // Hall sensor
  int8_t hall_table[8];
  float hall_sl_erpm;
  // FOC
  float foc_current_kp;
  float foc_current_ki;
  float foc_f_sw;
  float foc_dt_us;
  float foc_encoder_offset;
  bool foc_encoder_inverted;
  float foc_encoder_ratio;
  float foc_encoder_sin_offset;
  float foc_encoder_sin_gain;
  float foc_encoder_cos_offset;
  float foc_encoder_cos_gain;
  float foc_encoder_sincos_filter_constant;
  float foc_motor_l;
  float foc_motor_r;
  float foc_motor_flux_linkage;
  float foc_observer_gain;
  float foc_observer_gain_slow;
  float foc_pll_kp;
  float foc_pll_ki;
  float foc_duty_dowmramp_kp;
  float foc_duty_dowmramp_ki;
  float foc_openloop_rpm;
  float foc_sl_openloop_hyst;
  float foc_sl_openloop_time;
  float foc_sl_d_current_duty;
  float foc_sl_d_current_factor;
  mc_foc_sensor_mode foc_sensor_mode;
  uint8_t foc_hall_table[8];
  float foc_sl_erpm;
  bool foc_sample_v0_v7;
  bool foc_sample_high_current;
  float foc_sat_comp;
  bool foc_temp_comp;
  float foc_temp_comp_base_temp;
  float foc_current_filter_const;
  mc_foc_cc_decoupling_mode foc_cc_decoupling;
  mc_foc_observer_type foc_observer_type;
  float foc_hfi_voltage_start;
  float foc_hfi_voltage_run;
  float foc_hfi_voltage_max;
  float foc_sl_erpm_hfi;
  uint16_t foc_hfi_start_samples;
  float foc_hfi_obs_ovr_sec;
  FOC_HFI_SAMPLES foc_hfi_samples;
  // GPDrive
  int gpd_buffer_notify_left;
  int gpd_buffer_interpol;
  float gpd_current_filter_const;
  float gpd_current_kp;
  float gpd_current_ki;
  // Speed PID
  float s_pid_kp;
  float s_pid_ki;
  float s_pid_kd;
  float s_pid_kd_filter;
  float s_pid_min_erpm;
  bool s_pid_allow_braking;
  // Pos PID
  float p_pid_kp;
  float p_pid_ki;
  float p_pid_kd;
  float p_pid_kd_filter;
  float p_pid_ang_div;
  // Current controller
  float cc_startup_boost_duty;
  float cc_min_current;
  float cc_gain;
  float cc_ramp_step_max;
  // Misc
  int32_t m_fault_stop_time_ms;
  float m_duty_ramp_step;
  float m_current_backoff_gain;
  uint32_t m_encoder_counts;
  sensor_port_mode m_sensor_port_mode;
  bool m_invert_direction;
  drv8301_oc_mode m_drv8301_oc_mode;
  int m_drv8301_oc_adj;
  float m_bldc_f_sw_min;
  float m_bldc_f_sw_max;
  float m_dc_f_sw;
  float m_ntc_motor_beta;
  out_aux_mode m_out_aux_mode;
  temp_sensor_type m_motor_temp_sens_type;
  float m_ptc_motor_coeff;
  // Setup info
  uint8_t si_motor_poles;
  float si_gear_ratio;
  float si_wheel_diameter;
  BATTERY_TYPE si_battery_type;
  int si_battery_cells;
  float si_battery_ah;
} mc_configuration;

// Applications to use
typedef enum
{
  APP_NONE = 0,
  APP_PPM,
  APP_ADC,
  APP_UART,
  APP_PPM_UART,
  APP_ADC_UART,
  APP_NUNCHUK,
  APP_NRF,
  APP_CUSTOM,
  APP_BALANCE
} app_use;

// Throttle curve mode
typedef enum
{
  THR_EXP_EXPO = 0,
  THR_EXP_NATURAL,
  THR_EXP_POLY
} thr_exp_mode;

// PPM control types
typedef enum
{
  PPM_CTRL_TYPE_NONE = 0,
  PPM_CTRL_TYPE_CURRENT,
  PPM_CTRL_TYPE_CURRENT_NOREV,
  PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE,
  PPM_CTRL_TYPE_DUTY,
  PPM_CTRL_TYPE_DUTY_NOREV,
  PPM_CTRL_TYPE_PID,
  PPM_CTRL_TYPE_PID_NOREV,
  PPM_CTRL_TYPE_CURRENT_BRAKE_REV_HYST,
  PPM_CTRL_TYPE_CURRENT_SMART_REV
} ppm_control_type;

typedef struct
{
  ppm_control_type ctrl_type;
  float pid_max_erpm;
  float hyst;
  float pulse_start;
  float pulse_end;
  float pulse_center;
  bool median_filter;
  bool safe_start;
  float throttle_exp;
  float throttle_exp_brake;
  thr_exp_mode throttle_exp_mode;
  float ramp_time_pos;
  float ramp_time_neg;
  bool multi_esc;
  bool tc;
  float tc_max_diff;
  float max_erpm_for_dir;
  float smart_rev_max_duty;
  float smart_rev_ramp_time;
} ppm_config;

// ADC control types
typedef enum
{
  ADC_CTRL_TYPE_NONE = 0,
  ADC_CTRL_TYPE_CURRENT,
  ADC_CTRL_TYPE_CURRENT_REV_CENTER,
  ADC_CTRL_TYPE_CURRENT_REV_BUTTON,
  ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC,
  ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER,
  ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER,
  ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON,
  ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC,
  ADC_CTRL_TYPE_DUTY,
  ADC_CTRL_TYPE_DUTY_REV_CENTER,
  ADC_CTRL_TYPE_DUTY_REV_BUTTON,
  ADC_CTRL_TYPE_PID,
  ADC_CTRL_TYPE_PID_REV_CENTER,
  ADC_CTRL_TYPE_PID_REV_BUTTON
} adc_control_type;

typedef struct
{
  adc_control_type ctrl_type;
  float hyst;
  float voltage_start;
  float voltage_end;
  float voltage_center;
  float voltage2_start;
  float voltage2_end;
  bool use_filter;
  bool safe_start;
  bool cc_button_inverted;
  bool rev_button_inverted;
  bool voltage_inverted;
  bool voltage2_inverted;
  float throttle_exp;
  float throttle_exp_brake;
  thr_exp_mode throttle_exp_mode;
  float ramp_time_pos;
  float ramp_time_neg;
  bool multi_esc;
  bool tc;
  float tc_max_diff;
  uint32_t update_rate_hz;
} adc_config;

// Nunchuk control types
typedef enum
{
  CHUK_CTRL_TYPE_NONE = 0,
  CHUK_CTRL_TYPE_CURRENT,
  CHUK_CTRL_TYPE_CURRENT_NOREV
} chuk_control_type;

typedef struct
{
  chuk_control_type ctrl_type;
  float hyst;
  float ramp_time_pos;
  float ramp_time_neg;
  float stick_erpm_per_s_in_cc;
  float throttle_exp;
  float throttle_exp_brake;
  thr_exp_mode throttle_exp_mode;
  bool multi_esc;
  bool tc;
  float tc_max_diff;
  bool use_smart_rev;
  float smart_rev_max_duty;
  float smart_rev_ramp_time;
} chuk_config;

// NRF Datatypes
typedef enum
{
  NRF_SPEED_250K = 0,
  NRF_SPEED_1M,
  NRF_SPEED_2M
} NRF_SPEED;

typedef enum
{
  NRF_POWER_M18DBM = 0,
  NRF_POWER_M12DBM,
  NRF_POWER_M6DBM,
  NRF_POWER_0DBM,
  NRF_POWER_OFF
} NRF_POWER;

typedef enum
{
  NRF_AW_3 = 0,
  NRF_AW_4,
  NRF_AW_5
} NRF_AW;

typedef enum
{
  NRF_CRC_DISABLED = 0,
  NRF_CRC_1B,
  NRF_CRC_2B
} NRF_CRC;

typedef enum
{
  NRF_RETR_DELAY_250US = 0,
  NRF_RETR_DELAY_500US,
  NRF_RETR_DELAY_750US,
  NRF_RETR_DELAY_1000US,
  NRF_RETR_DELAY_1250US,
  NRF_RETR_DELAY_1500US,
  NRF_RETR_DELAY_1750US,
  NRF_RETR_DELAY_2000US,
  NRF_RETR_DELAY_2250US,
  NRF_RETR_DELAY_2500US,
  NRF_RETR_DELAY_2750US,
  NRF_RETR_DELAY_3000US,
  NRF_RETR_DELAY_3250US,
  NRF_RETR_DELAY_3500US,
  NRF_RETR_DELAY_3750US,
  NRF_RETR_DELAY_4000US
} NRF_RETR_DELAY;

typedef struct
{
  NRF_SPEED speed;
  NRF_POWER power;
  NRF_CRC crc_type;
  NRF_RETR_DELAY retry_delay;
  unsigned char retries;
  unsigned char channel;
  unsigned char address[3];
  bool send_crc_ack;
} nrf_config;

typedef struct
{
  float kp;
  float ki;
  float kd;
  uint16_t hertz;
  float pitch_fault;
  float roll_fault;
  float adc1;
  float adc2;
  float overspeed_duty;
  float tiltback_duty;
  float tiltback_angle;
  float tiltback_speed;
  float tiltback_high_voltage;
  float tiltback_low_voltage;
  float startup_pitch_tolerance;
  float startup_roll_tolerance;
  float startup_speed;
  float deadzone;
  float current_boost;
  bool multi_esc;
  float yaw_kp;
  float yaw_ki;
  float yaw_kd;
  float roll_steer_kp;
  float brake_current;
  uint16_t overspeed_delay;
  uint16_t fault_delay;
  float tiltback_constant;
  float roll_steer_erpm_kp;
  float yaw_current_clamp;
  uint16_t adc_half_fault_erpm;
  float setpoint_pitch_filter;
  float setpoint_target_filter;
  float setpoint_clamp;
} balance_config;

// CAN status modes
typedef enum
{
  CAN_STATUS_DISABLED = 0,
  CAN_STATUS_1,
  CAN_STATUS_1_2,
  CAN_STATUS_1_2_3,
  CAN_STATUS_1_2_3_4,
  CAN_STATUS_1_2_3_4_5
} CAN_STATUS_MODE;

typedef enum
{
  SHUTDOWN_MODE_ALWAYS_OFF = 0,
  SHUTDOWN_MODE_ALWAYS_ON,
  SHUTDOWN_MODE_TOGGLE_BUTTON_ONLY,
  SHUTDOWN_MODE_OFF_AFTER_10S,
  SHUTDOWN_MODE_OFF_AFTER_1M,
  SHUTDOWN_MODE_OFF_AFTER_5M,
  SHUTDOWN_MODE_OFF_AFTER_10M,
  SHUTDOWN_MODE_OFF_AFTER_30M,
  SHUTDOWN_MODE_OFF_AFTER_1H,
  SHUTDOWN_MODE_OFF_AFTER_5H,
} SHUTDOWN_MODE;

typedef enum
{
  IMU_TYPE_OFF = 0,
  IMU_TYPE_INTERNAL,
  IMU_TYPE_EXTERNAL_MPU9X50,
  IMU_TYPE_EXTERNAL_ICM20948,
  IMU_TYPE_EXTERNAL_BMI160
} IMU_TYPE;

typedef enum
{
  AHRS_MODE_MADGWICK = 0,
  AHRS_MODE_MAHONY
} AHRS_MODE;

typedef struct
{
  IMU_TYPE type;
  AHRS_MODE mode;
  int sample_rate_hz;
  float accel_confidence_decay;
  float mahony_kp;
  float mahony_ki;
  float madgwick_beta;
  float rot_roll;
  float rot_pitch;
  float rot_yaw;
  float accel_offsets[3];
  float gyro_offsets[3];
  float gyro_offset_comp_fact[3];
  float gyro_offset_comp_clamp;
} imu_config;

typedef enum
{
  CAN_MODE_VESC = 0,
  CAN_MODE_UAVCAN,
  CAN_MODE_COMM_BRIDGE
} CAN_MODE;

typedef struct
{
  // Settings
  uint8_t controller_id;
  uint32_t timeout_msec;
  float timeout_brake_current;
  CAN_STATUS_MODE send_can_status;
  uint32_t send_can_status_rate_hz;
  CAN_BAUD can_baud_rate;
  bool pairing_done;
  bool permanent_uart_enabled;
  SHUTDOWN_MODE shutdown_mode;

  // CAN modes
  CAN_MODE can_mode;
  uint8_t uavcan_esc_index;

  // Application to use
  app_use app_to_use;

  // PPM application settings
  ppm_config app_ppm_conf;

  // ADC application settings
  adc_config app_adc_conf;

  // UART application settings
  uint32_t app_uart_baudrate;

  // Nunchuk application settings
  chuk_config app_chuk_conf;

  // NRF application settings
  nrf_config app_nrf_conf;

  // Balance application settings
  balance_config app_balance_conf;

  // IMU Settings
  imu_config imu_conf;
} app_configuration;

// Communication commands
typedef enum
{
  COMM_FW_VERSION = 0,
  COMM_JUMP_TO_BOOTLOADER,
  COMM_ERASE_NEW_APP,
  COMM_WRITE_NEW_APP_DATA,
  COMM_GET_VALUES,
  COMM_SET_DUTY,
  COMM_SET_CURRENT,
  COMM_SET_CURRENT_BRAKE,
  COMM_SET_RPM,
  COMM_SET_POS,
  COMM_SET_HANDBRAKE,
  COMM_SET_DETECT,
  COMM_SET_SERVO_POS,
  COMM_SET_MCCONF,
  COMM_GET_MCCONF,
  COMM_GET_MCCONF_DEFAULT,
  COMM_SET_APPCONF,
  COMM_GET_APPCONF,
  COMM_GET_APPCONF_DEFAULT,
  COMM_SAMPLE_PRINT,
  COMM_TERMINAL_CMD,
  COMM_PRINT,
  COMM_ROTOR_POSITION,
  COMM_EXPERIMENT_SAMPLE,
  COMM_DETECT_MOTOR_PARAM,
  COMM_DETECT_MOTOR_R_L,
  COMM_DETECT_MOTOR_FLUX_LINKAGE,
  COMM_DETECT_ENCODER,
  COMM_DETECT_HALL_FOC,
  COMM_REBOOT,
  COMM_ALIVE,
  COMM_GET_DECODED_PPM,
  COMM_GET_DECODED_ADC,
  COMM_GET_DECODED_CHUK,
  COMM_FORWARD_CAN,
  COMM_SET_CHUCK_DATA,
  COMM_CUSTOM_APP_DATA,
  COMM_NRF_START_PAIRING,
  COMM_GPD_SET_FSW,
  COMM_GPD_BUFFER_NOTIFY,
  COMM_GPD_BUFFER_SIZE_LEFT,
  COMM_GPD_FILL_BUFFER,
  COMM_GPD_OUTPUT_SAMPLE,
  COMM_GPD_SET_MODE,
  COMM_GPD_FILL_BUFFER_INT8,
  COMM_GPD_FILL_BUFFER_INT16,
  COMM_GPD_SET_BUFFER_INT_SCALE,
  COMM_GET_VALUES_SETUP,
  COMM_SET_MCCONF_TEMP,
  COMM_SET_MCCONF_TEMP_SETUP,
  COMM_GET_VALUES_SELECTIVE,
  COMM_GET_VALUES_SETUP_SELECTIVE,
  COMM_EXT_NRF_PRESENT,
  COMM_EXT_NRF_ESB_SET_CH_ADDR,
  COMM_EXT_NRF_ESB_SEND_DATA,
  COMM_EXT_NRF_ESB_RX_DATA,
  COMM_EXT_NRF_SET_ENABLED,
  COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,
  COMM_DETECT_APPLY_ALL_FOC,
  COMM_JUMP_TO_BOOTLOADER_ALL_CAN,
  COMM_ERASE_NEW_APP_ALL_CAN,
  COMM_WRITE_NEW_APP_DATA_ALL_CAN,
  COMM_PING_CAN,
  COMM_APP_DISABLE_OUTPUT,
  COMM_TERMINAL_CMD_SYNC,
  COMM_GET_IMU_DATA,
  COMM_BM_CONNECT,
  COMM_BM_ERASE_FLASH_ALL,
  COMM_BM_WRITE_FLASH,
  COMM_BM_REBOOT,
  COMM_BM_DISCONNECT,
  COMM_BM_MAP_PINS_DEFAULT,
  COMM_BM_MAP_PINS_NRF5X,
  COMM_ERASE_BOOTLOADER,
  COMM_ERASE_BOOTLOADER_ALL_CAN,
  COMM_PLOT_INIT,
  COMM_PLOT_DATA,
  COMM_PLOT_ADD_GRAPH,
  COMM_PLOT_SET_GRAPH,
  COMM_GET_DECODED_BALANCE,
  COMM_BM_MEM_READ,
  COMM_WRITE_NEW_APP_DATA_LZO,
  COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO,
  COMM_BM_WRITE_FLASH_LZO,
  COMM_SET_CURRENT_REL,
  COMM_CAN_FWD_FRAME,
  COMM_SET_BATTERY_CUT,
  COMM_SET_BLE_NAME,
  COMM_SET_BLE_PIN,
  COMM_SET_CAN_MODE,
  COMM_GET_IMU_CALIBRATION
} COMM_PACKET_ID;

// CAN commands
typedef enum
{
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT,
  CAN_PACKET_SET_CURRENT_BRAKE,
  CAN_PACKET_SET_RPM,
  CAN_PACKET_SET_POS,
  CAN_PACKET_FILL_RX_BUFFER,
  CAN_PACKET_FILL_RX_BUFFER_LONG,
  CAN_PACKET_PROCESS_RX_BUFFER,
  CAN_PACKET_PROCESS_SHORT_BUFFER,
  CAN_PACKET_STATUS,
  CAN_PACKET_SET_CURRENT_REL,
  CAN_PACKET_SET_CURRENT_BRAKE_REL,
  CAN_PACKET_SET_CURRENT_HANDBRAKE,
  CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
  CAN_PACKET_STATUS_2,
  CAN_PACKET_STATUS_3,
  CAN_PACKET_STATUS_4,
  CAN_PACKET_PING,
  CAN_PACKET_PONG,
  CAN_PACKET_DETECT_APPLY_ALL_FOC,
  CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
  CAN_PACKET_CONF_CURRENT_LIMITS,
  CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
  CAN_PACKET_CONF_CURRENT_LIMITS_IN,
  CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
  CAN_PACKET_CONF_FOC_ERPMS,
  CAN_PACKET_CONF_STORE_FOC_ERPMS,
  CAN_PACKET_STATUS_5,
  CAN_PACKET_POLL_TS5700N8501_STATUS,
  CAN_PACKET_CONF_BATTERY_CUT,
  CAN_PACKET_CONF_STORE_BATTERY_CUT,
  CAN_PACKET_SHUTDOWN
} CAN_PACKET_ID;

// Logged fault data
typedef struct
{
  uint8_t motor;
  mc_fault_code fault;
  float current;
  float current_filtered;
  float voltage;
  float gate_driver_voltage;
  float duty;
  float rpm;
  int tacho;
  int cycles_running;
  int tim_val_samp;
  int tim_current_samp;
  int tim_top;
  int comm_step;
  float temperature;
  int drv8301_faults;
} fault_data;

// External LED state
typedef enum
{
  LED_EXT_OFF = 0,
  LED_EXT_NORMAL,
  LED_EXT_BRAKE,
  LED_EXT_TURN_LEFT,
  LED_EXT_TURN_RIGHT,
  LED_EXT_BRAKE_TURN_LEFT,
  LED_EXT_BRAKE_TURN_RIGHT,
  LED_EXT_BATT
} LED_EXT_STATE;

typedef struct
{
  int js_x;
  int js_y;
  int acc_x;
  int acc_y;
  int acc_z;
  bool bt_c;
  bool bt_z;
  bool rev_has_state;
  bool is_rev;
} chuck_data;

typedef struct
{
  int id;
  systime_t rx_time;
  float rpm;
  float current;
  float duty;
} can_status_msg;

typedef struct
{
  int id;
  systime_t rx_time;
  float amp_hours;
  float amp_hours_charged;
} can_status_msg_2;

typedef struct
{
  int id;
  systime_t rx_time;
  float watt_hours;
  float watt_hours_charged;
} can_status_msg_3;

typedef struct
{
  int id;
  systime_t rx_time;
  float temp_fet;
  float temp_motor;
  float current_in;
  float pid_pos_now;
} can_status_msg_4;

typedef struct
{
  int id;
  systime_t rx_time;
  float v_in;
  int32_t tacho_value;
} can_status_msg_5;

typedef struct
{
  uint8_t js_x;
  uint8_t js_y;
  bool bt_c;
  bool bt_z;
  bool bt_push;
  bool rev_has_state;
  bool is_rev;
  float vbat;
} mote_state;

typedef enum
{
  MOTE_PACKET_BATT_LEVEL = 0,
  MOTE_PACKET_BUTTONS,
  MOTE_PACKET_ALIVE,
  MOTE_PACKET_FILL_RX_BUFFER,
  MOTE_PACKET_FILL_RX_BUFFER_LONG,
  MOTE_PACKET_PROCESS_RX_BUFFER,
  MOTE_PACKET_PROCESS_SHORT_BUFFER,
  MOTE_PACKET_PAIRING_INFO
} MOTE_PACKET;

typedef struct
{
  float v_in;
  float temp_mos;
  float temp_mos_1;
  float temp_mos_2;
  float temp_mos_3;
  float temp_motor;
  float current_motor;
  float current_in;
  float id;
  float iq;
  float rpm;
  float duty_now;
  float amp_hours;
  float amp_hours_charged;
  float watt_hours;
  float watt_hours_charged;
  int tachometer;
  int tachometer_abs;
  float position;
  mc_fault_code fault_code;
  int vesc_id;
  float vd;
  float vq;
} mc_values;

typedef enum
{
  NRF_PAIR_STARTED = 0,
  NRF_PAIR_OK,
  NRF_PAIR_FAIL
} NRF_PAIR_RES;

// Orientation data
typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
  float integralFBx;
  float integralFBy;
  float integralFBz;
  float accMagP;
  int initialUpdateDone;
} ATTITUDE_INFO;

// Custom EEPROM variables
typedef union {
  uint32_t as_u32;
  int32_t as_i32;
  float as_float;
} eeprom_var;

#define EEPROM_VARS_HW            64
#define EEPROM_VARS_CUSTOM        64

typedef struct
{
  float ah_tot;
  float ah_charge_tot;
  float wh_tot;
  float wh_charge_tot;
  float current_tot;
  float current_in_tot;
  uint8_t num_vescs;
} setup_values;
}  // namespace vesc_driver
#endif  // VESC_DRIVER__DATATYPES_HPP_
