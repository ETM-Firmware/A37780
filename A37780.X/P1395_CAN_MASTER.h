#ifndef __P1395_CAN_MASTER_H
#define __P1395_CAN_MASTER_H

#include "P1395_CAN_CORE.h"

//#define __IGNORE_ION_PUMP_MODULE
//#define __IGNORE_AFC_MODULE
//#define __IGNORE_GUN_DRIVER_MODULE
//#define __IGNORE_COOLING_INTERFACE_MODULE
//#define __IGNORE_HEATER_MAGNET_MODULE
//#define __IGNORE_HV_LAMBDA_MODULE
//#define __IGNORE_PULSE_CURRENT_MODULE
//#define __IGNORE_PULSE_SYNC_MODULE
#define __IGNORE_TCU





typedef struct {
  unsigned unused_0:1;
  unsigned ion_pump_board:1;
  unsigned magnetron_current_board:1;
  unsigned pulse_sync_board:1;
  unsigned hv_lambda_board:1;
  unsigned afc_board:1;
  unsigned cooling_interface_board:1;
  unsigned heater_magnet_board:1;
  unsigned gun_driver_board:1;
  unsigned unused_9:1;
  unsigned unused_10:1;
  unsigned unused_11:1;
  unsigned unused_12:1;
  unsigned unused_13:1;
  unsigned ethernet_board:1;
  unsigned unused_15:1;
} P1395BoardBits;

extern P1395BoardBits board_status_received;
extern P1395BoardBits board_com_ok;


typedef struct {
  unsigned high_energy_pulse:1;
  unsigned arc_this_pulse:1;
  unsigned tbd_2:1;
  unsigned tbd_3:1;

  unsigned tbd_4:1;
  unsigned tbd_5:1;
  unsigned tbd_6:1;
  unsigned tbd_7:1;

  unsigned tbd_8:1;
  unsigned tbd_9:1;
  unsigned tbd_A:1;
  unsigned tbd_B:1;

  unsigned tbd_C:1;
  unsigned tbd_D:1;
  unsigned tbd_E:1;
  unsigned tbd_F:1;
} HighSpeedLogStatusBits;

typedef struct {
  unsigned int pulse_count;
  HighSpeedLogStatusBits status_bits; //This will contain high_low_energy?, arc_this_pulse?, what else???
  
  unsigned int x_ray_on_seconds_lsw;  // This is the lsw of x_ray_on_seconds, when the ECB recieved the "next pulse level" command
  unsigned int x_ray_on_milliseconds; // This is a representation of the milliseconds, when the ECB recieved the "next pulse level" command

  unsigned int hvlambda_vmon_at_eoc_period;  //unsigned int hvlambda_readback_high_energy_lambda_program_voltage;
  unsigned int hvlambda_vmon_at_trigger; //unsigned int hvlambda_readback_low_energy_lambda_program_voltage;
  unsigned int hvlambda_vpeak_at_eoc_period; //unsigned int hvlambda_readback_peak_lambda_voltage;

  unsigned int afc_readback_current_position;
  unsigned int afc_readback_target_position;
  unsigned int afc_readback_a_input;
  unsigned int afc_readback_b_input;
  unsigned int afc_readback_filtered_error_reading;

  unsigned int ionpump_readback_high_energy_target_current_reading;
  unsigned int ionpump_readback_low_energy_target_current_reading;

  unsigned int magmon_internal_adc_reading;
  unsigned int magmon_external_adc_reading;

  unsigned int psync_trigger_width_and_filtered_trigger_width;
  unsigned int psync_grid_width_and_delay;  //unsigned int psync_readback_high_energy_grid_width_and_delay;
  unsigned int psync_period; //unsigned int psync_readback_low_energy_grid_width_and_delay;
} ETMCanHighSpeedData;
// 19 words


extern ETMCanBoardData local_data_ecb;
extern ETMCanBoardData mirror_hv_lambda;
extern ETMCanBoardData mirror_ion_pump;
extern ETMCanBoardData mirror_afc;
extern ETMCanBoardData mirror_cooling;
extern ETMCanBoardData mirror_htr_mag;
extern ETMCanBoardData mirror_gun_drv;
extern ETMCanBoardData mirror_pulse_mon;
extern ETMCanBoardData mirror_pulse_sync;




#define _CONTROL_NOT_READY            local_data_ecb.status.control_notice_bits.control_not_ready
#define _CONTROL_NOT_CONFIGURED       local_data_ecb.status.control_notice_bits.control_not_configured
#define _CONTROL_SELF_CHECK_ERROR     local_data_ecb.status.control_notice_bits.control_self_check_error


#define _NOTICE_0                     local_data_ecb.status.control_notice_bits.notice_0
#define _NOTICE_1                     local_data_ecb.status.control_notice_bits.notice_1
#define _NOTICE_2                     local_data_ecb.status.control_notice_bits.notice_2
#define _NOTICE_3                     local_data_ecb.status.control_notice_bits.notice_3
#define _NOTICE_4                     local_data_ecb.status.control_notice_bits.notice_4
#define _NOTICE_5                     local_data_ecb.status.control_notice_bits.notice_5
#define _NOTICE_6                     local_data_ecb.status.control_notice_bits.notice_6
#define _NOTICE_7                     local_data_ecb.status.control_notice_bits.notice_7

#define _FAULT_0                      local_data_ecb.status.fault_bits.fault_0
#define _FAULT_1                      local_data_ecb.status.fault_bits.fault_1
#define _FAULT_2                      local_data_ecb.status.fault_bits.fault_2
#define _FAULT_3                      local_data_ecb.status.fault_bits.fault_3
#define _FAULT_4                      local_data_ecb.status.fault_bits.fault_4
#define _FAULT_5                      local_data_ecb.status.fault_bits.fault_5
#define _FAULT_6                      local_data_ecb.status.fault_bits.fault_6
#define _FAULT_7                      local_data_ecb.status.fault_bits.fault_7
#define _FAULT_8                      local_data_ecb.status.fault_bits.fault_8
#define _FAULT_9                      local_data_ecb.status.fault_bits.fault_9
#define _FAULT_A                      local_data_ecb.status.fault_bits.fault_A
#define _FAULT_B                      local_data_ecb.status.fault_bits.fault_B
#define _FAULT_C                      local_data_ecb.status.fault_bits.fault_C
#define _FAULT_D                      local_data_ecb.status.fault_bits.fault_D
#define _FAULT_E                      local_data_ecb.status.fault_bits.fault_E
#define _FAULT_F                      local_data_ecb.status.fault_bits.fault_F

#define _LOGGED_0                     local_data_ecb.status.warning_bits.warning_0
#define _LOGGED_1                     local_data_ecb.status.warning_bits.warning_1
#define _LOGGED_2                     local_data_ecb.status.warning_bits.warning_2
#define _LOGGED_3                     local_data_ecb.status.warning_bits.warning_3
#define _LOGGED_4                     local_data_ecb.status.warning_bits.warning_4
#define _LOGGED_5                     local_data_ecb.status.warning_bits.warning_5
#define _LOGGED_6                     local_data_ecb.status.warning_bits.warning_6
#define _LOGGED_7                     local_data_ecb.status.warning_bits.warning_7
#define _LOGGED_8                     local_data_ecb.status.warning_bits.warning_8
#define _LOGGED_9                     local_data_ecb.status.warning_bits.warning_9
#define _LOGGED_A                     local_data_ecb.status.warning_bits.warning_A
#define _LOGGED_B                     local_data_ecb.status.warning_bits.warning_B
#define _LOGGED_C                     local_data_ecb.status.warning_bits.warning_C
#define _LOGGED_D                     local_data_ecb.status.warning_bits.warning_D
#define _LOGGED_E                     local_data_ecb.status.warning_bits.warning_E
#define _LOGGED_F                     local_data_ecb.status.warning_bits.warning_F

#define _NOT_LOGGED_0                 local_data_ecb.status.not_logged_bits.not_logged_0
#define _NOT_LOGGED_1                 local_data_ecb.status.not_logged_bits.not_logged_1
#define _NOT_LOGGED_2                 local_data_ecb.status.not_logged_bits.not_logged_2
#define _NOT_LOGGED_3                 local_data_ecb.status.not_logged_bits.not_logged_3
#define _NOT_LOGGED_4                 local_data_ecb.status.not_logged_bits.not_logged_4
#define _NOT_LOGGED_5                 local_data_ecb.status.not_logged_bits.not_logged_5
#define _NOT_LOGGED_6                 local_data_ecb.status.not_logged_bits.not_logged_6
#define _NOT_LOGGED_7                 local_data_ecb.status.not_logged_bits.not_logged_7
#define _NOT_LOGGED_8                 local_data_ecb.status.not_logged_bits.not_logged_8
#define _NOT_LOGGED_9                 local_data_ecb.status.not_logged_bits.not_logged_9
#define _NOT_LOGGED_A                 local_data_ecb.status.not_logged_bits.not_logged_A
#define _NOT_LOGGED_B                 local_data_ecb.status.not_logged_bits.not_logged_B
#define _NOT_LOGGED_C                 local_data_ecb.status.not_logged_bits.not_logged_C
#define _NOT_LOGGED_D                 local_data_ecb.status.not_logged_bits.not_logged_D
#define _NOT_LOGGED_E                 local_data_ecb.status.not_logged_bits.not_logged_E
#define _NOT_LOGGED_F                 local_data_ecb.status.not_logged_bits.not_logged_F

#define _CONTROL_REGISTER             *(unsigned int*)&local_data_ecb.status.control_notice_bits
#define _FAULT_REGISTER               *(unsigned int*)&local_data_ecb.status.fault_bits
#define _WARNING_REGISTER             *(unsigned int*)&local_data_ecb.status.warning_bits
#define _NOT_LOGGED_REGISTER          *(unsigned int*)&local_data_ecb.status.not_logged_bits



// Board Configuration data - 0x06
#define config_agile_number_high_word      local_data_ecb.config_data[3]
#define config_agile_number_low_word       local_data_ecb.config_data[2]
#define config_agile_dash                  local_data_ecb.config_data[1]
#define config_agile_rev_ascii             local_data_ecb.config_data[0]

// Board Configuration data - 0x07
#define config_serial_number               local_data_ecb.config_data[7]
#define config_firmware_agile_rev          local_data_ecb.config_data[6]
#define config_firmware_branch             local_data_ecb.config_data[5]
#define config_firmware_branch_rev         local_data_ecb.config_data[4]






#define mirror_ecb_control_state                        local_data_ecb.board_data[0]
#define mirror_ecb_avg_output_power_watts               local_data_ecb.board_data[1]
#define mirror_ecb_thyratron_warmup_remaining           local_data_ecb.board_data[2]
#define mirror_ecb_magnetron_warmup_remaining           local_data_ecb.board_data[3]
#define mirror_ecb_gun_driver_warmup_remaining          local_data_ecb.board_data[4]
//#define mirror_board_com_fault                          local_data_ecb.board_data[5]
#define mirror_ecb_time_now_seconds // 6,7long          
#define mirror_ecb_system_power_seconds  // 8,9 long
#define mirror_ecb_system_hv_on_seconds  // 10,11 long
#define mirror_ecb_system_xray_on_seconds // 12,13 long










extern ETMCanBoardDebuggingData debug_data_ecb;
extern ETMCanBoardDebuggingData debug_data_slave_mirror;


// This "defines" data that may need to be displayed on the GUI
#define local_hvps_set_point_dose_0                     mirror_hv_lambda.local_data[0]
#define local_hvps_set_point_dose_1                     mirror_hv_lambda.local_data[1]

#define local_afc_home_position_dose_0                  mirror_afc.local_data[0]
#define local_afc_home_position_dose_1                  mirror_afc.local_data[3]
#define local_afc_aft_control_voltage_dose_all          mirror_afc.local_data[1]


#define local_magnetron_heater_current_dose_all         mirror_htr_mag.local_data[0]
#define local_heater_current_scaled_set_point           mirror_htr_mag.local_data[1]
#define local_magnet_current_set_point_dose_0           mirror_htr_mag.local_data[2]
#define local_magnet_current_set_point_dose_1           mirror_htr_mag.local_data[3]

#define local_gun_drv_top_v_dose_0                      mirror_gun_drv.local_data[0]
#define local_gun_drv_top_v_dose_1                      mirror_gun_drv.local_data[1]
#define local_gun_drv_heater_v_dose_all                 mirror_gun_drv.local_data[2]
#define local_gun_drv_cathode_v_dose_0                  mirror_gun_drv.local_data[3]
#define local_gun_drv_cathode_v_dose_1                  mirror_gun_drv.local_data[4]  // This is loaded but unused by anything at this point in time


/*
#define psync_grid_start_high_intensity_3               *(unsigned char*)&mirror_pulse_sync.local_data[0]
#define psync_grid_start_high_intensity_2               *((unsigned char*)&mirror_pulse_sync.local_data[0] + 1)
#define psync_grid_start_high_intensity_1               *(unsigned char*)&mirror_pulse_sync.local_data[1]
#define psync_grid_start_high_intensity_0               *((unsigned char*)&mirror_pulse_sync.local_data[1] + 1)
#define psync_dose_delay_high                           *(unsigned char*)&mirror_pulse_sync.local_data[2]
#define psync_pfn_delay_high                            *((unsigned char*)&mirror_pulse_sync.local_data[2] + 1)

#define psync_grid_stop_high_intensity_3                *(unsigned char*)&mirror_pulse_sync.local_data[4]
#define psync_grid_stop_high_intensity_2                *((unsigned char*)&mirror_pulse_sync.local_data[4] + 1)
#define psync_grid_stop_high_intensity_1                *(unsigned char*)&mirror_pulse_sync.local_data[5]
#define psync_grid_stop_high_intensity_0                *((unsigned char*)&mirror_pulse_sync.local_data[5] + 1)
#define psync_mag_delay_high                            *(unsigned char*)&mirror_pulse_sync.local_data[6]
#define psync_afc_delay_high                            *((unsigned char*)&mirror_pulse_sync.local_data[6] + 1)

#define psync_grid_start_low_intensity_3                *(unsigned char*)&mirror_pulse_sync.local_data[8]
#define psync_grid_start_low_intensity_2                *((unsigned char*)&mirror_pulse_sync.local_data[8] + 1)
#define psync_grid_start_low_intensity_1                *(unsigned char*)&mirror_pulse_sync.local_data[9]
#define psync_grid_start_low_intensity_0                *((unsigned char*)&mirror_pulse_sync.local_data[9] + 1)
#define psync_dose_delay_low                            *(unsigned char*)&mirror_pulse_sync.local_data[10]
#define psync_pfn_delay_low                             *((unsigned char*)&mirror_pulse_sync.local_data[10] + 1)

#define psync_grid_stop_low_intensity_3                 *(unsigned char*)&mirror_pulse_sync.local_data[12]
#define psync_grid_stop_low_intensity_2                 *((unsigned char*)&mirror_pulse_sync.local_data[12] + 1)
#define psync_grid_stop_low_intensity_1                 *(unsigned char*)&mirror_pulse_sync.local_data[13]
#define psync_grid_stop_low_intensity_0                 *((unsigned char*)&mirror_pulse_sync.local_data[13] + 1)
#define psync_mag_delay_low                             *(unsigned char*)&mirror_pulse_sync.local_data[14]
#define psync_afc_delay_low                             *((unsigned char*)&mirror_pulse_sync.local_data[14] + 1)
*/


#define local_pulse_sync_gun_trig_start_max_dose_0     mirror_pulse_sync.local_data[0]
#define local_pulse_sync_gun_trig_stop_max_dose_0      mirror_pulse_sync.local_data[1]
#define local_pulse_sync_gun_trig_start_max_dose_1     mirror_pulse_sync.local_data[2]
#define local_pulse_sync_gun_trig_stop_max_dose_1      mirror_pulse_sync.local_data[3]

#define local_pulse_sync_afc_trig_dose_0               mirror_pulse_sync.local_data[4]
#define local_pulse_sync_afc_trig_dose_1               mirror_pulse_sync.local_data[5]

#define local_pulse_sync_hvps_trig_start_dose_all      mirror_pulse_sync.local_data[8]
#define local_pulse_sync_pfn_trig_dose_all             mirror_pulse_sync.local_data[9]
#define local_pulse_sync_pulse_mon_trig_start_dose_all mirror_pulse_sync.local_data[10]


#define _HV_LAMBDA_NOT_READY               mirror_hv_lambda.status.control_notice_bits.control_not_ready
#define _HV_LAMBDA_NOT_CONFIGURED          mirror_hv_lambda.status.control_notice_bits.control_not_configured
#define _HV_LAMBDA_FAULT_REGISTER          *(unsigned int*)&mirror_hv_lambda.status.fault_bits

#define _ION_PUMP_NOT_READY                mirror_ion_pump.status.control_notice_bits.control_not_ready
#define _ION_PUMP_NOT_CONFIGURED           mirror_ion_pump.status.control_notice_bits.control_not_configured
#define _ION_PUMP_FAULT_REGISTER           *(unsigned int*)&mirror_ion_pump.status.fault_bits
#define _ION_PUMP_OVER_CURRENT_ACTIVE      mirror_ion_pump.status.warning_bits.warning_0

#define _AFC_NOT_READY                     mirror_afc.status.control_notice_bits.control_not_ready
#define _AFC_NOT_CONFIGURED                mirror_afc.status.control_notice_bits.control_not_configured
#define _AFC_FAULT_REGISTER                *(unsigned int*)&mirror_afc.status.fault_bits

#define _COOLING_NOT_READY                 mirror_cooling.status.control_notice_bits.control_not_ready
#define _COOLING_NOT_CONFIGURED            mirror_cooling.status.control_notice_bits.control_not_configured
#define _COOLING_FLOW_OK                   mirror_cooling.status.warning_bits.warning_6
#define _COOLING_FAULT_REGISTER            *(unsigned int*)&mirror_cooling.status.fault_bits

#define _HEATER_MAGNET_NOT_READY           mirror_htr_mag.status.control_notice_bits.control_not_ready
#define _HEATER_MAGNET_NOT_CONFIGURED      mirror_htr_mag.status.control_notice_bits.control_not_configured
#define _HEATER_MAGNET_HEATER_OK           mirror_htr_mag.status.warning_bits.warning_1
#define _HEATER_MAGNET_FAULT_REGISTER      *(unsigned int*)&mirror_htr_mag.status.fault_bits

#define _GUN_DRIVER_NOT_READY              mirror_gun_drv.status.control_notice_bits.control_not_ready
#define _GUN_DRIVER_NOT_CONFIGURED         mirror_gun_drv.status.control_notice_bits.control_not_configured
#define _GUN_DRIVER_HEATER_RAMP_COMPLETE   mirror_gun_drv.status.warning_bits.warning_5
#define _GUN_DRIVER_FAULT_REGISTER         *(unsigned int*)&mirror_gun_drv.status.fault_bits

#define _PULSE_MON_NOT_READY               mirror_pulse_mon.status.control_notice_bits.control_not_ready
#define _PULSE_MON_NOT_CONFIGURED          mirror_pulse_mon.status.control_notice_bits.control_not_configured
#define _PULSE_MON_FAULT_REGISTER          *(unsigned int*)&mirror_pulse_mon.status.fault_bits
#define _PULSE_MON_FALSE_TRIGGER           mirror_pulse_mon.status.fault_bits.fault_4

#define _PULSE_SYNC_NOT_READY              mirror_pulse_sync.status.control_notice_bits.control_not_ready
#define _PULSE_SYNC_NOT_CONFIGURED         mirror_pulse_sync.status.control_notice_bits.control_not_configured
#define _PULSE_SYNC_CUSTOMER_HV_OFF        mirror_pulse_sync.status.warning_bits.warning_0
#define _PULSE_SYNC_CUSTOMER_XRAY_OFF      mirror_pulse_sync.status.warning_bits.warning_1
#define _PULSE_SYNC_PERSONALITY_READY      mirror_pulse_sync.status.warning_bits.warning_4
#define _PULSE_SYNC_PERSONALITY_VALUE      ((*(unsigned int*)&mirror_pulse_sync.status.not_logged_bits) & 0x000F)
#define _PULSE_SYNC_FAULT_REGISTER          *(unsigned int*)&mirror_pulse_sync.status.fault_bits
#define _PULSE_SYNC_PFN_FAN_FAULT          mirror_pulse_sync.status.fault_bits.fault_3
#define _PULSE_SYNC_FAULT_X_RAY_MISMATCH   mirror_pulse_sync.status.fault_bits.fault_0


// PUBLIC Variables
#define HIGH_SPEED_DATA_BUFFER_SIZE   16
extern ETMCanHighSpeedData high_speed_data_buffer_a[HIGH_SPEED_DATA_BUFFER_SIZE]; // Used by TCP/IP Module
extern ETMCanHighSpeedData high_speed_data_buffer_b[HIGH_SPEED_DATA_BUFFER_SIZE]; // Used by TCP/IP Module
extern ETMCanHighSpeedData etm_can_high_speed_data_test;                          // Used by TCP/IP Module
extern unsigned int etm_can_active_debugging_board_id;                            // Used by TCP/IP Module



// Public Functions
void ETMCanMasterDoCan(void);

void ETMCanMasterInitialize(unsigned int requested_can_port, unsigned long fcy, unsigned int etm_can_address, unsigned long can_operation_led, unsigned int can_interrupt_priority);
/*
  This is called once when the processor starts up to initialize the can bus and all of the can variables
*/

void ETMCanMasterLoadConfiguration(unsigned long agile_id, unsigned int agile_dash, unsigned int agile_rev, unsigned int firmware_agile_rev, unsigned int firmware_branch, unsigned int firmware_branch_rev, unsigned int serial_number);
/*
  This is called once when the prcoessor starts up to load the board configuration into RAM so it can be sent over CAN to the ECB
*/

void ETMCanMasterSetSyncState(unsigned int state);

void SendCalibrationSetPointToSlave(unsigned int index, unsigned int data_1, unsigned int data_0);

void ReadCalibrationSetPointFromSlave(unsigned int index);

void SendSlaveLoadDefaultEEpromData(unsigned int board_id);

void SendSlaveReset(unsigned int board_id);

void SendToEventLog(unsigned int log_id);

void ETMCanMasterClearHighSpeedLogging(void);

// These defines are generated by spreadsheet to match the GUI
// https://docs.google.com/spreadsheets/d/1LzGyVvQnTHxvrr1z5vqS47o2sXn8c0amM__B0XKyfbk/edit#gid=0

#define LOG_ID_ENTERED_STATE_STARTUP                       0x0110
#define LOG_ID_ENTERED_STATE_SAFETY_SELF_TEST              0x0112
#define LOG_ID_ENTERED_STATE_WAITING_FOR_POWER_ON          0x0116
#define LOG_ID_ENTERED_STATE_WAITING_FOR_INITIALIZATION    0x0118
#define LOG_ID_ENTERED_STATE_WARMUP                        0x0120
#define LOG_ID_ENTERED_STATE_STANDBY                       0x0130
#define LOG_ID_ENTERED_STATE_DRIVE_UP                      0x0140
#define LOG_ID_ENTERED_STATE_READY                         0x0150
#define LOG_ID_ENTERED_STATE_XRAY_ON                       0x0160

#define LOG_ID_ENTERED_STATE_FAULT_WARMUP                  0x01A0
#define LOG_ID_ENTERED_STATE_FAULT_SYSTEM                  0x01A2
#define LOG_ID_ENTERED_STATE_FAULT_HOLD                    0x01A4
#define LOG_ID_ENTERED_STATE_FAULT_RESET                   0x01A6
#define LOG_ID_ENTERED_STATE_SAFE_POWER_DOWN               0x01A8


#define LOG_ID_NOT_CONNECTED_ION_PUMP_BOARD 0x0001
#define LOG_ID_CONNECTED_ION_PUMP_BOARD 0x0081
#define LOG_ID_NOT_CONNECTED_MAGNETRON_CURRENT_BOARD 0x0002
#define LOG_ID_CONNECTED_MAGNETRON_CURRENT_BOARD 0x0082
#define LOG_ID_NOT_CONNECTED_PULSE_SYNC_BOARD 0x0003
#define LOG_ID_CONNECTED_PULSE_SYNC_BOARD 0x0083
#define LOG_ID_NOT_CONNECTED_HV_LAMBDA_BOARD 0x0004
#define LOG_ID_CONNECTED_HV_LAMBDA_BOARD 0x0084
#define LOG_ID_NOT_CONNECTED_AFC_BOARD 0x0005
#define LOG_ID_CONNECTED_AFC_BOARD 0x0085
#define LOG_ID_NOT_CONNECTED_COOLING_BOARD 0x0006
#define LOG_ID_CONNECTED_COOLING_BOARD 0x0086
#define LOG_ID_NOT_CONNECTED_HEATER_MAGNET_BOARD 0x0007
#define LOG_ID_CONNECTED_HEATER_MAGNET_BOARD 0x0087
#define LOG_ID_NOT_CONNECTED_GUN_DRIVER_BOARD 0x0008
#define LOG_ID_CONNECTED_GUN_DRIVER_BOARD 0x0088

#define LOG_ID_PERSONALITY_RECEIVED 0x0200
#define LOG_ID_PERSONALITY_ERROR 0x0201
#define LOG_ID_ALL_MODULES_CONFIGURED 0x0202
#define LOG_ID_WARMUP_DONE 0x0203
#define LOG_ID_DRIVEUP_COMPLETE 0x0204
#define LOG_ID_DRIVE_UP_TIMEOUT 0x0205
#define LOG_ID_HV_OFF_FAULTS_CLEAR 0x0206




typedef struct {
  unsigned int  event_number; // this resets to zero at power up
  unsigned long event_time;   // this is the custom time format
  unsigned int  event_id;     // This tells what the event was

  // In the future we may add more data to the event;
} TYPE_EVENT;

extern unsigned int can_master_millisecond_counter;
#define mem_time_seconds_now                                 (*(unsigned long*)&local_data_ecb.log_data[1])


typedef struct {
  TYPE_EVENT event_data[128];
  unsigned int write_index;
  unsigned int gui_index;
  unsigned int eeprom_index;
} TYPE_EVENT_LOG;
 

extern TYPE_EVENT_LOG event_log;

/* 
   The ethernet control board keeps a record of standard data from all the slave boards
   This includes status, low level errors, configuration, and debug information
   This is a hack to allow the data on the master to be accessed the same way as it is on the slave boards
*/



extern ETMCanSyncMessage    etm_can_master_sync_message;

#define _SYNC_CONTROL_RESET_ENABLE            etm_can_master_sync_message.sync_0_control_word.sync_0_reset_enable
#define _SYNC_CONTROL_HIGH_SPEED_LOGGING      etm_can_master_sync_message.sync_0_control_word.sync_1_high_speed_logging_enabled
#define _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV   etm_can_master_sync_message.sync_0_control_word.sync_2_pulse_sync_disable_hv
#define _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY etm_can_master_sync_message.sync_0_control_word.sync_3_pulse_sync_disable_xray
#define _SYNC_CONTROL_COOLING_FAULT           etm_can_master_sync_message.sync_0_control_word.sync_4_cooling_fault
#define _SYNC_CONTROL_SYSTEM_HV_DISABLE       etm_can_master_sync_message.sync_0_control_word.sync_5_system_hv_disable
#define _SYNC_CONTROL_GUN_DRIVER_DISABLE_HTR  etm_can_master_sync_message.sync_0_control_word.sync_6_gun_driver_disable_heater
#define _SYNC_CONTROL_CLEAR_DEBUG_DATA        etm_can_master_sync_message.sync_0_control_word.sync_F_clear_debug_data


#define _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED   etm_can_master_sync_message.sync_0_control_word.sync_A_pulse_sync_warmup_led_on
#define _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED  etm_can_master_sync_message.sync_0_control_word.sync_B_pulse_sync_standby_led_on
#define _SYNC_CONTROL_PULSE_SYNC_READY_LED    etm_can_master_sync_message.sync_0_control_word.sync_C_pulse_sync_ready_led_on
#define _SYNC_CONTROL_PULSE_SYNC_FAULT_LED    etm_can_master_sync_message.sync_0_control_word.sync_D_pulse_sync_fault_led_on

#define _SYNC_CONTROL_WORD                    *(unsigned int*)&etm_can_master_sync_message.sync_0_control_word



extern unsigned int etm_can_master_next_pulse_level;  // This value will get updated when a next pulse level command is received
extern unsigned int etm_can_master_next_pulse_count;  // This value will get updated when a next pulse level command is received


unsigned int ETMCanMasterGetPulsePRF(void);


void ETMCanMasterSendMsg(unsigned int id, unsigned int word3, unsigned int word2, unsigned int word1, unsigned int word0);



#endif
