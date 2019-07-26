/*  
    What data is needed where.
    
    status_data - This is needed by the ECB module to pass on to the main loop code to detect communciation timeouts and shutdown the system.
                - Fault information should be available to the main loop code via function calls (for individual data or request to return the entire status message)
		- 
    log_data/device_data/debugging data - Only needs to be displayed on the GUI - 

    
*/






// -------------- DEBUG Data Logging ----------------- //





// ------------- Slave info Data Logging ---------- //











// ---------------- HIGH SPEED LOGGING ------------------------ //
/*
  There are two identical pulse-by-pulse data holding registers
  One is active for writing with data from the CAN bus
  The other is availble for transmission over the Ethernet connection
  
  (1) On ETMCanMasterSendSyncMessage, the row indicated by "pulse_count" is initialized
  (2) If row initialized >= 3 the other holding register is marked for transmit
  (3) When a can pulse-by-pulse data logging message is recieved, it is recorded in the row indicated by "pulse count" & 0xF
  (4) 
  
*/

#define HOLDING_REGUSTER_STAUTS_WRITING_DATA                0
#define HOLDING_REGISTER_STATUS_WAITING_FOR_TRANSMIT        1
#define HOLDING_REGISTER_STATUS_WAITING_TRANSMIT_COMPLETE   2


unsigned int pulse_by_pulse_holding_register_a_status;
unsigned int pulse_by_pulse_holding_register_b_status;

extern ETMCanHighSpeedData pulse_data_buffer_a[HIGH_SPEED_DATA_BUFFER_SIZE];
extern ETMCanHighSpeedData pulse_data_buffer_b[HIGH_SPEED_DATA_BUFFER_SIZE];








// ------------------- EVENT LOG LOGGING ---------------------- //







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

  unsigned int hvlambda_readback_high_energy_lambda_program_voltage;
  unsigned int hvlambda_readback_low_energy_lambda_program_voltage;
  unsigned int hvlambda_readback_peak_lambda_voltage;

  unsigned int afc_readback_current_position;
  unsigned int afc_readback_target_position;
  unsigned int afc_readback_a_input;
  unsigned int afc_readback_b_input;
  unsigned int afc_readback_filtered_error_reading;

  unsigned int ionpump_readback_high_energy_target_current_reading;
  unsigned int ionpump_readback_low_energy_target_current_reading;

  unsigned int magmon_readback_magnetron_high_energy_current;
  unsigned int magmon_readback_magnetron_low_energy_current;

  unsigned int psync_readback_trigger_width_and_filtered_trigger_width;
  unsigned int psync_readback_high_energy_grid_width_and_delay;
  unsigned int psync_readback_low_energy_grid_width_and_delay;
} ETMCanHighSpeedData;
// 19 words



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

#define _WARNING_0                    local_data_ecb.status.warning_bits.warning_0
#define _WARNING_1                    local_data_ecb.status.warning_bits.warning_1
#define _WARNING_2                    local_data_ecb.status.warning_bits.warning_2
#define _WARNING_3                    local_data_ecb.status.warning_bits.warning_3
#define _WARNING_4                    local_data_ecb.status.warning_bits.warning_4
#define _WARNING_5                    local_data_ecb.status.warning_bits.warning_5
#define _WARNING_6                    local_data_ecb.status.warning_bits.warning_6
#define _WARNING_7                    local_data_ecb.status.warning_bits.warning_7
#define _WARNING_8                    local_data_ecb.status.warning_bits.warning_8
#define _WARNING_9                    local_data_ecb.status.warning_bits.warning_9
#define _WARNING_A                    local_data_ecb.status.warning_bits.warning_A
#define _WARNING_B                    local_data_ecb.status.warning_bits.warning_B
#define _WARNING_C                    local_data_ecb.status.warning_bits.warning_C
#define _WARNING_D                    local_data_ecb.status.warning_bits.warning_D
#define _WARNING_E                    local_data_ecb.status.warning_bits.warning_E
#define _WARNING_F                    local_data_ecb.status.warning_bits.warning_F


#define _CONTROL_REGISTER             *(unsigned int*)&local_data_ecb.status.control_notice_bits
#define _FAULT_REGISTER               *(unsigned int*)&local_data_ecb.status.fault_bits
#define _WARNING_REGISTER             *(unsigned int*)&local_data_ecb.status.warning_bits




// Board Configuration data - 0x06
#define config_agile_number_high_word      local_data_ecb.config_data[0]
#define config_agile_number_low_word       local_data_ecb.config_data[1]
#define config_agile_dash                  local_data_ecb.config_data[2]
#define config_agile_rev_ascii             local_data_ecb.config_data[3]

// Board Configuration data - 0x07
#define config_serial_number               local_data_ecb.config_data[4]
#define config_firmware_agile_rev          local_data_ecb.config_data[5]
#define config_firmware_branch             local_data_ecb.config_data[6]
#define config_firmware_branch_rev         local_data_ecb.config_data[7]






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

ETMCanBoardDebuggingData debug_data_ecb;
ETMCanBoardDebuggingData debug_data_slave_mirror;

// This "defines" data that may need to be displayed on the GUI
#define local_hv_lambda_high_en_set_point               mirror_hv_lambda.local_data[0]
#define local_hv_lambda_low_en_set_point                mirror_hv_lambda.local_data[1]

#define local_afc_home_position                         mirror_afc.local_data[0]
#define local_afc_aft_control_voltage                   mirror_afc.local_data[1]

#define local_heater_current_full_set_point             mirror_htr_mag.local_data[0]
#define local_heater_current_scaled_set_point           mirror_htr_mag.local_data[1]
#define local_magnet_current_set_point                  mirror_htr_mag.local_data[2]

#define local_gun_drv_high_en_pulse_top_v               mirror_gun_drv.local_data[0]
#define local_gun_drv_low_en_pulse_top_v                mirror_gun_drv.local_data[1]
#define local_gun_drv_heater_v_set_point                mirror_gun_drv.local_data[2]
#define local_gun_drv_cathode_set_point                 mirror_gun_drv.local_data[3]

#define local_pulse_sync_timing_reg_0_word_0            mirror_pulse_sync.local_data[0]
#define local_pulse_sync_timing_reg_0_word_1            mirror_pulse_sync.local_data[1]
#define local_pulse_sync_timing_reg_0_word_2            mirror_pulse_sync.local_data[2]
#define local_pulse_sync_timing_reg_1_word_0            mirror_pulse_sync.local_data[3]
#define local_pulse_sync_timing_reg_1_word_1            mirror_pulse_sync.local_data[4]
#define local_pulse_sync_timing_reg_1_word_2            mirror_pulse_sync.local_data[5]
#define local_pulse_sync_timing_reg_2_word_0            mirror_pulse_sync.local_data[6]
#define local_pulse_sync_timing_reg_2_word_1            mirror_pulse_sync.local_data[7]
#define local_pulse_sync_timing_reg_2_word_2            mirror_pulse_sync.local_data[8]
#define local_pulse_sync_timing_reg_3_word_0            mirror_pulse_sync.local_data[9]
#define local_pulse_sync_timing_reg_3_word_1            mirror_pulse_sync.local_data[10]
#define local_pulse_sync_timing_reg_3_word_2            mirror_pulse_sync.local_data[11]

// PUBLIC Variables
#define HIGH_SPEED_DATA_BUFFER_SIZE   16


extern ETMCanHighSpeedData              etm_can_high_speed_data_test;



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
#define _SYNC_CONTROL_CLEAR_DEBUG_DATA        etm_can_master_sync_message.sync_0_control_word.sync_F_clear_debug_data

#define _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED   etm_can_master_sync_message.sync_0_control_word.sync_A_pulse_sync_warmup_led_on
#define _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED  etm_can_master_sync_message.sync_0_control_word.sync_B_pulse_sync_standby_led_on
#define _SYNC_CONTROL_PULSE_SYNC_READY_LED    etm_can_master_sync_message.sync_0_control_word.sync_C_pulse_sync_ready_led_on
#define _SYNC_CONTROL_PULSE_SYNC_FAULT_LED    etm_can_master_sync_message.sync_0_control_word.sync_D_pulse_sync_fault_led_on

#define _SYNC_CONTROL_WORD                    *(unsigned int*)&etm_can_master_sync_message.sync_0_control_word








  unsigned int buffer_a_ready_to_send;
  unsigned int buffer_a_sent;
  
  unsigned int buffer_b_ready_to_send;
  unsigned int buffer_b_sent;



// --------- Global Buffers --------------- //
ETMCanHighSpeedData         high_speed_data_buffer_a[HIGH_SPEED_DATA_BUFFER_SIZE];
ETMCanHighSpeedData         high_speed_data_buffer_b[HIGH_SPEED_DATA_BUFFER_SIZE];





void SendToEventLog(unsigned int log_id);
void SendToPulseLog(unsigned int log_id, unsigned int word3, unsigned int word2, unsigned word1, unsigned int word0);
void SendToScopeLog();





typedef struct {
  // From ECB
  unsigned char pulse_count;
  unsigned char status_bits;
  unsigned long tic_value;          // 32 bit counter value when trigger recieved
  unsigned int ecb_data_0;          // Requested Gun Trigger Width
  unsigned int ecb_data_1;          // Gun Trigger Width
  unsigned int ecb_data_2;          // 
  
  // 6 words total
  
  // From slave boards (HVPS for now)
  unsigned int board_a_data_0;      // HV Lambda Voltage Pre-Pulse
  unsigned int board_a_data_1;      // HV Lamdba Voltage at end of charge Period
  unsigned int board_a_data_2;      // HV Lambda Spare Data
  
  // (AFC for now)
  unsigned int  board_b_data_0;      // AFC Readback Current Position
  unsigned int  board_b_data_1;      // AFC Readback Reverse
  unsigned int  board_b_data_2;      // AFC Readback Forward

  // (Gun for now)
  unsigned int  board_c_data_0;      // AFC Readback Current Position
  unsigned int  board_c_data_1;      // AFC Readback Reverse
  unsigned int  board_d_data_2;      // AFC Readback Forward

  // Space for a couple of the Char data bytes from boards
  unsigned char board_char_data0;    // More from AFC
  unsigned char board_char_data1;    // More from Gun

  // 10 Words, 16 Total
  
  // Interlace board data - This is sent once every 3 pulses 
  unsigned int interlace_data_0;      // Magnetron Current Reading
  unsigned int interlace_data_1;      // Target Current Reading
  unsigned int interlace_data_2;      // Reference Detector data

  // 3 Words, 19 Total
  
} ETMCanHighSpeedData;



void SendToPulseLog(unsigned int log_id, unsigned int word3, unsigned int word2, unsigned word1, unsigned int word0) {
  unsigned int pulse_count;
  unsigned char data3;
  
  pulse_count = (word3 >> 8);
  data3 = word3;

}



void SendToEventLog(unsigned int log_id) {
  event_log.event_data[event_log.write_index].event_number = global_data_can_master.event_log_counter;
  event_log.event_data[event_log.write_index].event_time   = global_data_can_master.time_seconds_now;
  event_log.event_data[event_log.write_index].event_id     = log_id;
  event_log.write_index++;
  event_log.write_index &= 0x7F;
  global_data_can_master.event_log_counter++;
  // DPARKER need to check the EEPROM and TCP locations and advance them as nesseasry so that we don't pass them when advancing the write_index
}




void SendToScopeLog(unsigned int scope_id, unsigned int data3, unsigned int data2, unsigned int data1, unsigned int data0);


unsigned int scope_data_holding_a[40];
unsigned int scope_data_holding_b[40];

typedef struct {
  unsigned char data_format; // 
  unsigned char satus;
  data_a[40];
  data_b[40];
};

#define SCOPE_DATA_FORMAT_100NS_12_BIT               1
#define SCOPE_DATA_FORMAT_100US_12_BIT_FROM_HVPS     2
#define SCOPE_DATA_FORMAT_100US_12_BIT_FROM_PFN      3
#define SCOPE_DATA_FORMAT_1MS_10_BIT                 4
#define SCOPE_DATA_FORMAT_1MS_12_BIT                 5
#define SCOPE_DATA_FORMAT_1MS_8_BIT                  6
#define SCOPE_DATA_FORMAT_1MS_16_BIT                 7

  
  
  


void SendToScopeLog(unsigned int scope_id, unsigned int data3, unsigned int data2, unsigned int data1, unsigned int data0) {
  
}
