#ifndef __P1395_CAN_MASTER_H
#define __P1395_CAN_MASTER_H


#include "P1395_CAN_CORE.h"  // This is needed for status register deffenition
#include "P1395_CAN_CORE_PUBLIC.h"
#include "ETM_LINAC_MODBUS.h"




// Public Functions
void ETMCanMasterInitialize(unsigned int requested_can_port, unsigned long fcy,
			    unsigned long can_operation_led, unsigned int can_interrupt_priority,
			    unsigned int boards_to_ignore, unsigned int system_conf_setting);

#define SYSTEM_CONFIGURATION_A37500_000  0  // 6 MeV Medical Linac
#define SYSTEM_CONFIGURATION_A36375_240  1  // 2.5 MeV 240 Vac
#define SYSTEM_CONFIGURATION_A36375_400  2  // 2.5 MeV 400 Vac
#define SYSTEM_CONFIGURATION_A37854_000  3  // 6/4 MeV 


/*
  This is called once when the processor starts up to initialize the can bus and all of the can variables
*/

void ETMCanMasterDoCan(void);
/*
  This should be called once through the main execution loop
  This runs all the interrupt based can functions
  Recieving Messages, Sending messages, executing messages, logging data, ect.
*/

void ETMCanMasterSetScaledMagnetronHeaterCurrent(unsigned int scaled_current_setting);
/*
  This is used to set the scaled magentron heater current.  
  It is not stored in the dose_level registers because that data is just a mirror from the EEProm
*/

void ETMCanMasterSetActiveDebuggingBoardID(unsigned int save_debug_data_from_this_board_id);
/*
  This is used to selct which board's debugging information is stored
  This must come from the GUI as only the GUI knows which board it wants to display debugging information for
*/


void ETMCanMasterClearECBDebug(void);
/*
  Clears the debug info for the ECB
*/

void ETMCanMasterSendSlaveResetMCU(unsigned int board_id);
void ETMCanMasterSendSlaveLoadDefaultEEpromData(unsigned int board_id);
void ETMCanMasterSendSlaveRevAndSerialNumber(unsigned int board_id, unsigned int serial_number, unsigned int rev_char);
void ETMCanMasterSendSlaveCalibrationPair(unsigned int board_id, unsigned int cal_pair_select, unsigned int cal_offset, unsigned int cal_gain);
void ETMCanMasterSendSlaveRAMDebugLocations(unsigned int board_id, unsigned int address_A, unsigned int address_B, unsigned int address_C);
void ETMCanMasterSendSlaveEEPROMDebug(unsigned int board_id, unsigned int eeprom_register);
void ETMCanMasterSendDiscreteCMD(unsigned int discrete_cmd_id);
void ETMCanMasterSendSlaveClearPersistent(void);
void ETMCanMasterSendSlaveIgnoreMessage(unsigned int board_id, unsigned int unused_a, unsigned int unused_b, unsigned int ignore_bits);
void ETMCanMasterClearHighSpeedLogging(void);




void ETMCanMasterSelectScopeDataSourceGeneric(unsigned int scope_a_data_source, unsigned int scope_b_data_source);
void ETMCanMasterSelectScopeDataSourceHVVmon(unsigned int hv_vmon_source);


//void ETMCanMasterSendDefaultRequestRTSP();
//void ETMCanMasterSendDefaultConfirmRTSP();
//void ETMCanMasterSendDefaultIgnoreFaults(unsigned int board_id, unsigned int faults_to_ignore);




void ETMCanMasterSendSyncMessage(unsigned char dose_level, unsigned char pulse_count);
/*
  This is used after a trigger (or change in sync data) to send a sync message
  The can module will automatically send out a sync message every
  SYNC_MESSAGE_MAX_TRANSMISSION_PERIOD millisconds if this is not called
*/


void ETMCanMasterSyncSet(unsigned char sync_setting_select, unsigned char value);

#define SYNC_BIT_RESET_ENABLE      0x00
#define SYNC_BIT_ENABLE_PULSE_LOG  0x01 
#define SYNC_BIT_COOLING_FAULT     0x04
#define SYNC_BIT_HV_DISABLE        0x05
#define SYNC_BIT_GUN_HTR_DISABLE   0x06
#define SYNC_BIT_CLEAR_DEBUG_DATA  0x0F


unsigned int ETMCanMasterCheckResetActive(void);
/*
  Returns 0xFFFF if reset is active
  0 otherwise
*/


unsigned int ETMCanMasterCheckSlaveConfigured(unsigned int board_id);
/*
  Checks the connected and configured status of a slave board
  Returns 0xFFFF if the slave is connected and configured OR if the slave is being ignored
  Returns 0 otherwise
*/

unsigned int ETMCanMasterCheckAllBoardsConnected(void);
/*
  Checks the connections status of all boards (that not are not ignored)
  Returns 0xFFFF if all the boards are connected
  Returns 0 otherwise
*/

unsigned int ETMCanMasterCheckAllBoardsConfigured(void);
/*
  Checks the configuration AND connection status of all boards (that not are not ignored)
  Returns 0xFFFF if all the boards are configured and connected
  Returns 0 otherwise
*/

unsigned int ETMCanMasterCheckAllSlavesReady(void);
/*
  Returns 0xFFFF it all slave boards (except those ignored) are ready.
  Returns 0x0000 otherwise

  This should be called before every trigger is generated
*/


unsigned int ETMCanMasterCheckSlaveReady(unsigned int board_select);
/*
  If this board is being ignored, this function will awlays return 0xFFFF
  If this board is not connected, this function will return 0x0000
  If none of the above, will return 0xFFFF is not not_ready bit is clear, will return 0x0000 if the not_ready bit is set
*/

unsigned int ETMCanMasterCheckSlaveFault(void);
/*
  Returns 0xFFFF if any (non-ignored) slave has a fault.
  Returns 0x0000 otherwise

*/


unsigned int ETMCanMasterReturnSlaveStatusBit(unsigned int bit_select, unsigned int default_value);
/*
  This, funcion is used to check any bit in the status register of any board
  If, the bit selected is not valid for any reason, (bit_select not valid, board is being ignored, ect) the the default_value will be returned
  This is so that control logic can be written to behave in a known fashion if a board is being ignored

  bit_select - used to select the slave board, status register word, and bit you want to check
  0x0[board_id][Register Select][Bit Select]
  For example the Gun Driver(board 0x7), Top OV error (Fault Register - 0x1) (bit - 0x6)
  Would be defined as 0x0716

  Will return 0xFFFF if the selected bit is set, 0x0000 otherwise
*/

// bits are (highest nibble - unused) (next nibble - board_id) (next nibble - register select) (next nibble - bit id)

#define HEATER_MAGNET_HEATER_OK_BIT            0x0400 // DPARKER Get the correct bits
#define ION_PUMP_OVER_CURRENT_ACTIVE_BIT       0x0100
#define COOLING_INTERFACE_FLOW_OK_BIT          0x0300
#define MAGNETRON_CURRENT_FALSE_TRIGGER        0x0611  
#define GUN_DRIVER_HEATER_RAMP_COMPLETE        0x0500





typedef struct {
  unsigned int hvps_set_point;                    // transmitted to HVPS Interface
  unsigned int electromagnet_set_point;           // transmitted to HEATER/MAGNET Interface
  unsigned int gun_driver_pulse_top_voltage;      // transmitted to Gun Driver Interfacce
  unsigned int gun_driver_cathode_voltage;        // transmitted to Gun Driver Interfacce
  unsigned int trigger_delay_spare;               // Used on System Controller
  unsigned int trigger_delay_afc;                 // Used on System Controller
  unsigned int trigger_grid_start_min_dose;       // Used on System Controller
  unsigned int trigger_grid_start_max_dose;       // Used on System Controller
  unsigned int trigger_grid_stop_min_dose;        // Used on System Controller
  unsigned int trigger_grid_stop_max_dose;        // Used on System Controller
  unsigned int afc_home_poistion;                 // transmitted to AFC Inerterface
  unsigned int self_trigger_prf;                  // Used on System Controller
  unsigned int unused_2;
  unsigned int unused_1;
  unsigned int unused_0;
  unsigned int crc_do_not_write;
} TYPE_DOSE_LEVEL;


typedef struct {
  unsigned int magnetron_heater_current_at_standby;         // Used on System Controller, but the scalled seeting is sent to HEATER/MAGNET Interface
  unsigned int gun_driver_heater_voltage;         // transmitted to Gun Driver Interfacce
  unsigned int trigger_hvps_start;                // Used on System Controller
  unsigned int trigger_hvps_stop;                 // Used on System Controller
  unsigned int trigger_pfn;                       // Used on System Controller
  unsigned int trigger_magnetron_and_target_current_start;  // Used on System Controller
  unsigned int trigger_magnetron_and_target_current_stop;   // Used on System Controller
  unsigned int x_ray_run_time_in_automated_mode;  // Used on System Controller
  unsigned int gun_driver_bias_voltage;           // transmitted to Gun Driver Interfacce
  unsigned int afc_aux_control_or_offset;         // transmitted to AFC Inerterface
  unsigned int afc_manual_target_position;        // transmitted to AFC Inerterface
  unsigned int afc_locked_mode;                   // used by AFC to lock position to home
  unsigned int unused_2;                          // DO NOT USED - NOT MAPPED TO ANYTHING
  unsigned int unused_1;                          // DO NOT USED - NOT MAPPED TO ANYTHING
  unsigned int unused_0;                          // DO NOT USED - NOT MAPPED TO ANYTHING
  unsigned int crc_do_not_write;
} TYPE_ALL_DOSE_LEVELS;


typedef struct {
  unsigned int aux_set_point_0;                   // FUTURE USE
  unsigned int aux_set_point_1;                   // FUTURE USE
  unsigned int aux_set_point_2;                   // FUTURE USE
  unsigned int aux_set_point_3;                   // FUTURE USE
  unsigned int aux_set_point_4;                   // FUTURE USE
  unsigned int aux_set_point_5;                   // FUTURE USE
  unsigned int aux_set_point_6;                   // FUTURE USE
  unsigned int aux_set_point_7;                   // FUTURE USE

  unsigned int unused_6;                          // DO NOT USED - NOT MAPPED TO ANYTHING
  unsigned int unused_5;                          // DO NOT USED - NOT MAPPED TO ANYTHING
  unsigned int unused_4;                          // DO NOT USED - NOT MAPPED TO ANYTHING
  unsigned int unused_3;                          // DO NOT USED - NOT MAPPED TO ANYTHING
  unsigned int unused_2;                          // DO NOT USED - NOT MAPPED TO ANYTHING
  unsigned int unused_1;                          // DO NOT USED - NOT MAPPED TO ANYTHING
  unsigned int unused_0;                          // DO NOT USED - NOT MAPPED TO ANYTHING
  unsigned int crc_do_not_write;
} TYPE_AUX_SET_POINTS;


typedef struct {
  unsigned int             compensation_0;
  unsigned int             compensation_1;
  unsigned int             compensation_2;
  unsigned int             compensation_3;
  unsigned int             compensation_4;
  unsigned int             compensation_5;
  unsigned int             compensation_6;
  unsigned int             compensation_7;
  unsigned int             compensation_8;
  unsigned int             compensation_9;
  unsigned int             compensation_10;
  unsigned int             compensation_11;
  unsigned int             compensation_12;
  unsigned int             compensation_13;
  unsigned int             compensation_14;
  unsigned int             crc_do_not_write;
} TYPE_DOSE_COMP;


typedef struct {
  unsigned int             ecb_agile_number_high_word;
  unsigned int             ecb_agile_number_low_word;
  unsigned int             ecb_agile_dash_number;
  unsigned int             ecb_agile_rev_ASCII_x2;
  unsigned int             ecb_serial_number_high_word;
  unsigned int             ecb_serial_number_low_word;
  unsigned int             firmware_agile_rev;
  unsigned int             firmware_branch;
  unsigned int             firmware_branch_rev;
  unsigned int             system_serial_letter;
  unsigned int             system_serial_number_high_word;
  unsigned int             system_serial_number_low_word;
  unsigned int             date_of_atp; // upper 7 bits (years since 2000), lower 9 bits, day of year
  unsigned int             atp_technician;
  unsigned int             unused;
  unsigned int             crc_do_not_write;
} TYPE_ECB_INFO;


typedef struct {
  unsigned long            arc_counter;
  unsigned long            hv_on_seconds;
  unsigned long            powered_seconds;
  unsigned long            xray_on_seconds;
  unsigned long            last_warmup_seconds;
  unsigned long            warmup_status;
  // Higest 12 bits = thyratron warmup, middle 10 bits = magnetron heater warmup, lowest 10 bits = gun heater warmup
  unsigned long long       pulse_counter;       // Only the highest 48 bits are available
  // The lowest word of the pulse_counter is in fact the CRC
  // when increasing the pulse counter add 0x00010000, instead of 1
} TYPE_SYSTEM_COUNTERS;


//extern ETMCanBoardDebuggingData debug_data_ecb;

typedef struct {
  unsigned b0_pwr_c_flt:1;
  unsigned b1_pwr_b_flt:1;
  unsigned b2_pwr_a_flt:1;
  unsigned b3_beam_enable_status:1;
  unsigned b4_spare_input_ok:1;
  unsigned b5_interlock_2_open:1;
  unsigned b6_interlock_1_open:1;
  unsigned b7_unused:1;
  
  unsigned a0_phase_monitor_flt:1;
  unsigned a1_24v_monitor_flt:1;
  unsigned a2_ac_contactor_open:1;
  unsigned a3_hv_contactor_open:1;
  unsigned a4_estop_1_open:1;
  unsigned a5_estop_2_open:1;
  unsigned a6_panel_switch_open:1;
  unsigned a7_keylock_open:1;
} TYPE_IO_EXPANDER;

#define b4_gun_contactor_open b4_spare_input_ok

typedef struct {
  ETMCanStatusRegister status;
  TYPE_SYSTEM_COUNTERS system_counters;
  TYPE_DOSE_LEVEL      dose_level_0;
  TYPE_DOSE_LEVEL      dose_level_1;
  TYPE_DOSE_LEVEL      dose_level_2;
  TYPE_DOSE_LEVEL      dose_level_3;
  TYPE_ALL_DOSE_LEVELS dose_level_all;
  TYPE_DOSE_COMP       dose_compensation_group_a;
  TYPE_DOSE_COMP       dose_compensation_group_b;
  TYPE_ECB_INFO        config;
  TYPE_AUX_SET_POINTS  aux_set_points;
  TYPE_IO_EXPANDER     io_expander_inputs;
  unsigned int         control_state;
  unsigned int         system_configuration_select;
  unsigned int         spare_ecb_data_to_slaves;
  unsigned int         safety_self_test;
  unsigned int         cpu_inputs;
  unsigned int         cpu_outputs;
} TYPE_ECB_DATA;

#define NUMBER_OF_DATA_MIRRORS 10  //10 Slave Can Channels
extern TYPE_ECB_DATA ecb_data;
extern ETMCanBoardData local_data_mirror[NUMBER_OF_DATA_MIRRORS];
extern ETMCanBoardDebuggingData debug_data_slave_mirror;
extern ETMCanBoardDebuggingData debug_data_ecb;

/*
#define _CONTROL_NOT_READY            ecb_data.status.control_notice_bits.control_not_ready
#define _CONTROL_NOT_CONFIGURED       ecb_data.status.control_notice_bits.control_not_configured
#define _CONTROL_SELF_CHECK_ERROR     ecb_data.status.control_notice_bits.control_self_check_error


#define _NOTICE_0                     ecb_data.status.control_notice_bits.notice_0
#define _NOTICE_1                     ecb_data.status.control_notice_bits.notice_1
#define _NOTICE_2                     ecb_data.status.control_notice_bits.notice_2
#define _NOTICE_3                     ecb_data.status.control_notice_bits.notice_3
#define _NOTICE_4                     ecb_data.status.control_notice_bits.notice_4
#define _NOTICE_5                     ecb_data.status.control_notice_bits.notice_5
#define _NOTICE_6                     ecb_data.status.control_notice_bits.notice_6
#define _NOTICE_7                     ecb_data.status.control_notice_bits.notice_7

#define _FAULT_0                      ecb_data.status.fault_bits.fault_0
#define _FAULT_1                      ecb_data.status.fault_bits.fault_1
#define _FAULT_2                      ecb_data.status.fault_bits.fault_2
#define _FAULT_3                      ecb_data.status.fault_bits.fault_3
#define _FAULT_4                      ecb_data.status.fault_bits.fault_4
#define _FAULT_5                      ecb_data.status.fault_bits.fault_5
#define _FAULT_6                      ecb_data.status.fault_bits.fault_6
#define _FAULT_7                      ecb_data.status.fault_bits.fault_7
#define _FAULT_8                      ecb_data.status.fault_bits.fault_8
#define _FAULT_9                      ecb_data.status.fault_bits.fault_9
#define _FAULT_A                      ecb_data.status.fault_bits.fault_A
#define _FAULT_B                      ecb_data.status.fault_bits.fault_B
#define _FAULT_C                      ecb_data.status.fault_bits.fault_C
#define _FAULT_D                      ecb_data.status.fault_bits.fault_D
#define _FAULT_E                      ecb_data.status.fault_bits.fault_E
#define _FAULT_F                      ecb_data.status.fault_bits.fault_F

#define _LOGGED_0                     ecb_data.status.warning_bits.warning_0
#define _LOGGED_1                     ecb_data.status.warning_bits.warning_1
#define _LOGGED_2                     ecb_data.status.warning_bits.warning_2
#define _LOGGED_3                     ecb_data.status.warning_bits.warning_3
#define _LOGGED_4                     ecb_data.status.warning_bits.warning_4
#define _LOGGED_5                     ecb_data.status.warning_bits.warning_5
#define _LOGGED_6                     ecb_data.status.warning_bits.warning_6
#define _LOGGED_7                     ecb_data.status.warning_bits.warning_7
#define _LOGGED_8                     ecb_data.status.warning_bits.warning_8
#define _LOGGED_9                     ecb_data.status.warning_bits.warning_9
#define _LOGGED_A                     ecb_data.status.warning_bits.warning_A
#define _LOGGED_B                     ecb_data.status.warning_bits.warning_B
#define _LOGGED_C                     ecb_data.status.warning_bits.warning_C
#define _LOGGED_D                     ecb_data.status.warning_bits.warning_D
#define _LOGGED_E                     ecb_data.status.warning_bits.warning_E
#define _LOGGED_F                     ecb_data.status.warning_bits.warning_F

#define _NOT_LOGGED_0                 ecb_data.status.not_logged_bits.not_logged_0
#define _NOT_LOGGED_1                 ecb_data.status.not_logged_bits.not_logged_1
#define _NOT_LOGGED_2                 ecb_data.status.not_logged_bits.not_logged_2
#define _NOT_LOGGED_3                 ecb_data.status.not_logged_bits.not_logged_3
#define _NOT_LOGGED_4                 ecb_data.status.not_logged_bits.not_logged_4
#define _NOT_LOGGED_5                 ecb_data.status.not_logged_bits.not_logged_5
#define _NOT_LOGGED_6                 ecb_data.status.not_logged_bits.not_logged_6
#define _NOT_LOGGED_7                 ecb_data.status.not_logged_bits.not_logged_7
#define _NOT_LOGGED_8                 ecb_data.status.not_logged_bits.not_logged_8
#define _NOT_LOGGED_9                 ecb_data.status.not_logged_bits.not_logged_9
#define _NOT_LOGGED_A                 ecb_data.status.not_logged_bits.not_logged_A
#define _NOT_LOGGED_B                 ecb_data.status.not_logged_bits.not_logged_B
#define _NOT_LOGGED_C                 ecb_data.status.not_logged_bits.not_logged_C
#define _NOT_LOGGED_D                 ecb_data.status.not_logged_bits.not_logged_D
#define _NOT_LOGGED_E                 ecb_data.status.not_logged_bits.not_logged_E
#define _NOT_LOGGED_F                 ecb_data.status.not_logged_bits.not_logged_F

#define _CONTROL_REGISTER             *(unsigned int*)&ecb_data.status.control_notice_bits
#define _FAULT_REGISTER               *(unsigned int*)&ecb_data.status.fault_bits
#define _WARNING_REGISTER             *(unsigned int*)&ecb_data.status.warning_bits
#define _NOT_LOGGED_REGISTER          *(unsigned int*)&ecb_data.status.not_logged_bits

*/


//void ETMCanSlaveStatusUpdateBitNotReady(unsigned int value);
//#define NOT_READY         1
//#define READY             0

//void ETMCanSlaveStatusSetNoticeBit(unsigned int notice_bit);

//unsigned int ETMCanSlaveStatusCheckNotConfigured(void);
// Will return 0xFFFF if not configured, 0 otherwise


void ETMCanMasterStatusFaultResetAll(void);

void ETMCanMasterStatusUpdateFaultBit(unsigned int fault_bit, unsigned int value);

unsigned int ETMCanMasterStatusReadFaultBit(unsigned int fault_bit);
// Will return 0xFFFF if the fault bit is set, 0 otherwise

unsigned int ETMCanMasterStatusReadFaultRegister(void);
// Will return 0xFFFF if ANY fault bit is set, 0 otherwise

void ETMCanMasterStatusUpdateLoggedBit(unsigned int logged_bit, unsigned int value);

unsigned int ETMCanMasterStatusReadLoggedBit(unsigned int logged_bit);
// Will return 0xFFFF if the bit is set, 0 otherwise

void ETMCanMasterStatusUpdateNotLoggedBit(unsigned int not_logged_bit, unsigned int value);

unsigned int ETMCanMasterStatusReadNotLoggedBit(unsigned int not_logged_bit);
// Will return 0xFFFF if the bit is set, 0 otherwise




#define _FAULT_CAN_COMMUNICATION      0x0001
#define _FAULT_1                      0x0002
#define _FAULT_2                      0x0004 
#define _FAULT_3                      0x0008
#define _FAULT_4                      0x0010
#define _FAULT_5                      0x0020
#define _FAULT_6                      0x0040
#define _FAULT_7                      0x0080
#define _FAULT_8                      0x0100
#define _FAULT_9                      0x0200
#define _FAULT_A                      0x0400
#define _FAULT_B                      0x0800
#define _FAULT_C                      0x1000
#define _FAULT_D                      0x2000
#define _FAULT_E                      0x4000
#define _FAULT_F                      0x8000

/*
#define _NOTICE_0                     0x0100
#define _NOTICE_1                     0x0200
#define _NOTICE_2                     0x0400
#define _NOTICE_3                     0x0800
#define _NOTICE_4                     0x1000
#define _NOTICE_5                     0x2000
#define _NOTICE_6                     0x4000
#define _NOTICE_7                     0x8000

*/


#define _LOGGED_STATUS_0              0x0001
#define _LOGGED_STATUS_1              0x0002
#define _LOGGED_STATUS_2              0x0004
#define _LOGGED_STATUS_3              0x0008
#define _LOGGED_STATUS_4              0x0010
#define _LOGGED_STATUS_5              0x0020
#define _LOGGED_STATUS_6              0x0040
#define _LOGGED_STATUS_7              0x0080
#define _LOGGED_STATUS_8              0x0100
#define _LOGGED_STATUS_9              0x0200
#define _LOGGED_STATUS_A              0x0400
#define _LOGGED_STATUS_B              0x0800
#define _LOGGED_STATUS_C              0x1000
#define _LOGGED_STATUS_D              0x2000
#define _LOGGED_STATUS_E              0x4000
#define _LOGGED_STATUS_F              0x8000

#define _NOT_LOGGED_STATUS_0          0x0001
#define _NOT_LOGGED_STATUS_1          0x0002
#define _NOT_LOGGED_STATUS_2          0x0004
#define _NOT_LOGGED_STATUS_3          0x0008
#define _NOT_LOGGED_STATUS_4          0x0010
#define _NOT_LOGGED_STATUS_5          0x0020
#define _NOT_LOGGED_STATUS_6          0x0040
#define _NOT_LOGGED_STATUS_7          0x0080
#define _NOT_LOGGED_STATUS_8          0x0100
#define _NOT_LOGGED_STATUS_9          0x0200
#define _NOT_LOGGED_STATUS_A          0x0400
#define _NOT_LOGGED_STATUS_B          0x0800
#define _NOT_LOGGED_STATUS_C          0x1000
#define _NOT_LOGGED_STATUS_D          0x2000
#define _NOT_LOGGED_STATUS_E          0x4000
#define _NOT_LOGGED_STATUS_F          0x8000


#endif
