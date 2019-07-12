#ifndef __A36507_H
#define __A36507_H

#include <xc.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>
#include <uart.h>

#include "ETM.h"
//#include "TCPmodbus.h"
#include "ETM_LINAC_MODBUS.h"



//#include "ETMmodbus.h"
#include "P1395_CAN_MASTER.h"


//#define __WATCHDOG_MASTER_EMULATION

#define FCY_CLK  20000000


typedef struct {
  unsigned int hvps_voltage;
  unsigned int electromagnet_current;
  unsigned int gun_driver_pulse_top_voltage;
  unsigned int gun_cathode_voltage;
  unsigned int spare_trigger_on;
  unsigned int gun_pulse_mid_point_and_afc_sample_time;
  unsigned int gun_pulse_min_width;
  unsigned int gun_pulse_max_width;
  unsigned int afc_home_position;
  unsigned int prf_set_point;
  unsigned int unused_0;
  unsigned int unused_1;
  unsigned int unused_2;
  unsigned int unused_3;
  unsigned int unused_4;
} TYPE_DOSE_LEVEL;

typedef struct {
  unsigned int  magnetron_heater_current_at_standby;
  unsigned int  gun_driver_heater_voltage;
  unsigned int  afc_offset_control_comp;
  unsigned int  pfn_trigger_time;
  unsigned int  hvps_trigger_time_1;
  unsigned int  hvps_trigger_time_2;
  unsigned int  magnetron_current_sample_time_1;
  unsigned int  magnetron_current_sample_time_2;
  unsigned long run_time_set_point;
  unsigned int  unused_0;
  unsigned int  unused_1;
  unsigned int  unused_2;
  unsigned int  unused_3;
  unsigned int  unused_4;
} TYPE_DOSE_FIXED_SETTINGS;


typedef struct {
  unsigned int dose_comp_0[15];
  unsigned int dose_comp_1[15];
  unsigned int dose_comp_2[15];
  unsigned int dose_comp_3[15];
} TYPE_DOSE_COMPENSATION;


typedef struct {
  TYPE_DOSE_LEVEL          dose_level_0;
  TYPE_DOSE_LEVEL          dose_level_1;
  TYPE_DOSE_LEVEL          dose_level_2;
  TYPE_DOSE_LEVEL          dose_level_3;
  TYPE_DOSE_FIXED_SETTINGS fixed_dose_settings;
  TYPE_DOSE_COMPENSATION   dose_compensation;
} TYPE_RF_CONFIGURATION;


typedef struct {
  unsigned long system_counter_seconds_hv_on;
  unsigned long system_counter_seconds_powered;
  unsigned long system_counter_seconds_xray_on;
  unsigned long system_counter_arc_total;
  unsigned long system_counter_pulse_total;
  unsigned int  gun_driver_warmup_remaining;
  unsigned int  magnetron_warmup_remaining;
  unsigned int  pfn_warmup_remaining;
} TYPE_SYSTEM_COUNTERS;


typedef struct {
  ETMCanStatusRegister             status;
  unsigned int                     log_data[24];
  unsigned int                     config_data[8];
} TYPE_ETM_CAL_SLAVE_MIRROR;

typedef struct {
  unsigned int                     custom[16];
  unsigned int                     standard[48];
  unsigned int                     memory[4];
  unsigned int                     power_rail[4];
} TYPE_DEBUG_DATA;


#define SCOPE_DATA_SIZE   64
typedef struct {
  unsigned int                    buffer_a[SCOPE_DATA_SIZE];
  unsigned int                    buffer_b[SCOPE_DATA_SIZE];
  unsigned char                   buffer_a_ready_to_send;
  unsigned char                   buffer_a_sent;
  unsigned char                   buffer_b_ready_to_send;
  unsigned char                   buffer_b_sent;
} TYPE_SCOPE_DATA;

#define HVPS_DATA_SIZE   256
typedef struct {
  unsigned char                   buffer_a[HVPS_DATA_SIZE];
  unsigned char                   buffer_b[HVPS_DATA_SIZE];
  unsigned char                   buffer_a_ready_to_send;
  unsigned char                   buffer_a_sent;
  unsigned char                   buffer_b_ready_to_send;
  unsigned char                   buffer_b_sent;
} TYPE_HV_VMON_DATA;


typedef struct {
  unsigned int                    pulse_id;
  unsigned int                    pulse_info; // what there an arc on this pulse???
  unsigned int                    data[50];
  unsigned char                   data_n;                    
} TYPE_MAGNETRON_CURRENT_DATA;


/*
  Hardware Module Resource Usage

  CAN1 - Can module
  CAN2 - Reserved in case we need CAN 2

  Timer1 - Used by ETMTick

  Timer4 - Used to time CAN transmits - This is configured by ETM CAN module
  Timer5 - Used as timeoeout on status update receives - This is configured by ETM CAN module

  Timer2 - Unused at this time

  UART1 - Reserved for TCU Communication - Used for Bidirection Watchdog Function
  UART2 - Reserved for Serial GUI


 */









// ----------------- IO PIN CONFIGURATION -------------------- //
/*
  All unused pins will be set to outputs and logic zero
  LAT values default to 0 at startup so they do not need to be manually set
*/



// ----------------- DIGITAL INPUT PINS --------------- //
/*
  RA9  (Accidentally left grounded)
  RG0  (Unused Can Pin)
  RG1  (Unused Can Pin)
  RG14 (Reset Detect)
  RB14 (Analog Input)
  RB15 (Analog Input)
  
  Pins that are overidden by a hardware module and should be left as inputs during port configuration
  RB0 PROGRAM
  RB1 PROGRAM

  RF0 CAN 1
  RF1 CAN 1
  RF2 UART 1
  RF3 UART 1
  RF4 UART 2
  RF5 UART 2
  RF6 SPI 1
  RF7 SPI 1
  RF8 SPI 1

  RG2 I2C
  RG3 I2C

  Pins that are configured by other software modules and should be left as inputs during port configuration
  RA14 (Ethernet Module Interrupt Input)
  RA15 (Ethernet Module Reset Output)
  RD14 (Ethernet Module Clock Input)
  RD15 (Ethernet Module CS Output)
  

*/

#define A36507_TRISA_VALUE 0b1100001000000000 
#define A36507_TRISB_VALUE 0b0110000000000011 
#define A36507_TRISC_VALUE 0b0000000000000000 
#define A36507_TRISD_VALUE 0b1100000000000000 
#define A36507_TRISF_VALUE 0b0000000111111111 
#define A36507_TRISG_VALUE 0b0100000000001111


#define PIN_OUT_ETM_UART_1_DE                 _LATD7
#define PIN_OUT_ETM_UART_2_DE                 _LATD6
#define OLL_UART_TX_DRIVER_ENABLE             1

#define PIN_IN_ETM_RESET_DETECT               _RG14
#define PIN_OUT_ETM_RESET_DETECT              _LATG14
#define TRIS_PIN_ETM_RESET_RETECT             _TRISG14

#define PIN_OUT_ETM_LED_OPERATIONAL_GREEN     _LATA7
#define PIN_OUT_ETM_LED_TEST_POINT_A_RED      _LATG12
#define PIN_OUT_ETM_LED_TEST_POINT_B_GREEN    _LATG13
#define OLL_LED_ON                            0

#define PIN_OUT_TP_13                         _LATC15
#define PIN_OUT_TP_14                         _LATB7
#define PIN_OUT_TP_15                         _LATB8
#define PIN_OUT_TP_16                         _LATB9


// --------------- CONFIGURE TMR2 MODULE ----------------------- //
#define T2CON_VALUE                    (T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_8 & T2_32BIT_MODE_OFF & T2_SOURCE_INT)
#define PR2_PERIOD_US                  10000   // 10mS
#define PR2_VALUE_10_MILLISECONDS      ((FCY_CLK/1000000)*PR2_PERIOD_US/8)



// ------------------------ CONFIGURE ADC MODULE ------------------- //
#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_AVDD_EXT & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_16 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_10Tcy)

#define ADPCFG_SETTING          (ENABLE_AN13_ANA & ENABLE_AN14_ANA)
#define ADCSSL_SETTING          (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN5 & SKIP_SCAN_AN6 &  SKIP_SCAN_AN7 & SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN15)

#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN13 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN14 & ADC_CH0_NEG_SAMPLEB_VREFN)








// ---------------------- FAULTS/WARNINGS ------------------------ //
#define FAULT_A36507_CAN_TIMEOUT              0b0000 0000 0000 0001
#define FAULT_A36507_CAN_ETHERNET_TIMEOUT     0b0000 0000 0000 0010
//#define FAULT_A36507_     




typedef struct {
  // This is a 16 word Block that is written / read from EEPROM as a group
  unsigned long long pulse_counter_48_bit;  // DPARKER Change this to a 64 bit number and ignore the high word and reaarage the data so the high word isn't sent to the EEPROM
  unsigned long arc_counter;
  unsigned long system_hv_on_seconds;
  unsigned long system_powered_seconds;
  unsigned long system_xray_on_seconds;
  unsigned long last_recorded_warmup_seconds;
  unsigned long holding_bits_for_warmup; // Higest 12 bits = thyratron warmup, middle 10 bits = magnetron heater warmup, lowest 10 bits = gun heater warmup
  unsigned int reserved_crc_eeprom_page_0;

  unsigned int thyratron_warmup_remaining; 
  unsigned int magnetron_warmup_remaining;
  unsigned int gun_warmup_remaining;

  
  TYPE_PUBLIC_ANALOG_INPUT analog_input_5v_mon;                    // 1mV per LSB
  TYPE_PUBLIC_ANALOG_INPUT analog_input_3v3_mon;                   // 1mV per LSB

  unsigned int control_state;
  //unsigned int thyratron_warmup_counter_seconds;
  //unsigned int magnetron_heater_warmup_counter_seconds;
  //unsigned int gun_driver_heater_warmup_counter_seconds;

  //unsigned int millisecond_counter;
  unsigned int warmup_timer_stage;
  unsigned int warmup_done;
  unsigned int gun_heater_holdoff_timer;

  
  RTC_TIME time_now;
  //unsigned long time_seconds_now;
  
  //unsigned int send_pulse_sync_config;
  unsigned int drive_up_timer;

  //unsigned int average_output_power_watts;
  unsigned int event_log_counter;
  
  unsigned int startup_counter;


  unsigned int no_connect_count_ion_pump_board;
  unsigned int no_connect_count_magnetron_current_board;
  unsigned int no_connect_count_pulse_sync_board;
  unsigned int no_connect_count_hv_lambda_board;
  unsigned int no_connect_count_afc_board;
  unsigned int no_connect_count_cooling_interface_board;
  unsigned int no_connect_count_heater_magnet_board;
  unsigned int no_connect_count_gun_driver_board;

  unsigned int buffer_a_ready_to_send;
  unsigned int buffer_b_ready_to_send;
  unsigned int buffer_a_sent;
  unsigned int buffer_b_sent;

  unsigned int reset_requested;
  
  unsigned int personality_select_from_pulse_sync;

  unsigned int drive_up_fault_counter;
  unsigned int high_voltage_on_fault_counter;
  unsigned int reset_hold_timer;

  unsigned int system_serial_number;
  unsigned int most_recent_ref_detector_reading;

  unsigned int eeprom_failure;

  unsigned int most_recent_watchdog_reading;

  unsigned int access_mode;
  unsigned int service_passcode;
  unsigned int etm_passcode;

  unsigned int eeprom_write_status;

  
} A36507GlobalVars;

#define ECB_COUNTER_AND_TIMERS_RAM_POINTER (((unsigned int*)(&global_data_A36507.pulse_counter_48_bit)) + 1)


/*
//#define thyratron_warmup_counter_seconds                     local_data_ecb.log_data[4]
//#define magnetron_heater_warmup_counter_seconds              local_data_ecb.log_data[5]
//#define gun_driver_heater_warmup_counter_seconds             local_data_ecb.log_data[6]
#define system_powered_seconds                               (*(unsigned long*)&local_data_ecb.log_data[8])
#define system_hv_on_seconds                                 (*(unsigned long*)&local_data_ecb.log_data[10])
#define system_xray_on_seconds                               (*(unsigned long*)&local_data_ecb.log_data[12])

#define personality_select_from_pulse_sync                   local_data_ecb.log_data[15]

*/
#define average_output_power_watts                           local_data_ecb.log_data[14]


extern A36507GlobalVars global_data_A36507;


#define _FAULT_X_RAY_ON_LOGIC_ERROR                     _FAULT_0
#define _FAULT_REPEATED_DRIVE_UP_FAULT                  _FAULT_1
#define _FAULT_REPEATED_HV_ON_FAULT                     _FAULT_2
#define _FAULT_EEPROM_FAILURE                           _FAULT_3
// FAULT 4 is reserved for magnetron over power
#define _FAULT_WATCHDOG_ERROR                           _FAULT_5

// DPAKRER  - Need to evaluate how these are used under new control system
#define _STATUS_PERSONALITY_LOADED                      _LOGGED_0
#define _STATUS_DRIVE_UP_TIMEOUT                        _LOGGED_1

#define _STATUS_LAST_RESET_WAS_POWER_CYCLE              _NOT_LOGGED_0




// These are computed from Filament Lookup Table worksheet
// https://docs.google.com/spreadsheets/d/18de5OHQ0gJUx2U1b8VjYTvutx2ACGTG0XtN_QEIc9WI/
#define FILAMENT_LOOK_UP_TABLE_VALUES_FOR_MG7095_V2 0xFFF,0xFF1,0xFE2,0xFD2,0xFC2,0xFB1,0xFA0,0xF8D,0xF7B,0xF67,0xF53,0xF3F,0xF29,0xF13,0xEFD,0xEE6,0xECE,0xEB6,0xE9D,0xE83,0xE69,0xE4E,0xE33,0xE16,0xDFA,0xDDC,0xDBE,0xDA0,0xD81,0xD61,0xD40,0xD1F,0xCFE,0xCDB,0xCB8,0xC95,0xC71,0xC4C,0xC26,0xC00,0xBDA,0xBB2,0xB8A,0xB62,0xB39,0xB0F,0xAE4,0xAB9,0xA8E,0xA62,0xA35,0xA07,0x9D9,0x9AA,0x97B,0x94B,0x91A,0x8E9,0x8B7,0x885,0x851,0x81E,0x7E9,0x7B4


#define FILAMENT_LOOK_UP_TABLE_VALUES_FOR_MG7095 0xFFF,0xFD6,0xFD6,0xFAE,0xFAE,0xF85,0xF5C,0xF33,0xF0A,0xF0A,0xEE1,0xEB8,0xE8F,0xE66,0xE3D,0xE14,0xDEB,0xDC2,0xD70,0xD47,0xD1E,0xCF5,0xCA3,0xC7A,0xC51,0xBFF,0xBD6,0xB85,0xB5C,0xB0A,0xAB8,0xA8F,0xA3D,0x9EB,0x9C2,0x970,0x91E,0x8CC,0x87A,0x828,0x7D7,0x785,0x733,0x6E1,0x68F,0x63D,0x5EB,0x599,0x51E,0x4CC,0x47A,0x3FF,0x3AE,0x35C,0x2E1,0x28F,0x214,0x199,0x147,0xCC,0x7A,0x0,0x0,0x0 


#define FILAMENT_LOOK_UP_TABLE_VALUES_FOR_MG5193 0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xF85,0xF0A,0xE8F,0xE14,0xD70,0xCF5,0xC7A,0xBFF,0xB85,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0


#define EEPROM_PAGE_SYSTEM_CONFIG_HTR_MAG_AFC               0x000
#define EEPROM_PAGE_SYSTEM_CONFIG_HV_LAMBDA                 0x001
#define EEPROM_PAGE_SYSTEM_CONFIG_GUN_DRV                   0x002
#define EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1          0x003
#define EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_2          0x004
#define EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_3          0x005
#define EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_4          0x006
#define EEPROM_PAGE_ON_TIME                                 0x007
#define EEPROM_PAGE_HEATER_TIMERS                           0x008
// EEPROM PAGES reserved for future use                     9->F


#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_HTR_MAG_AFC               0x020
#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_HV_LAMBDA                 0x021
#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_GUN_DRV                   0x022
#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_1          0x023
#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_2          0x024
#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_3          0x025
#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_4          0x026




#define STATE_STARTUP                                0x10
#define STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC   0x12
#define STATE_WAITING_FOR_INITIALIZATION             0x15
#define STATE_WARMUP                                 0x20
#define STATE_STANDBY                                0x30
#define STATE_DRIVE_UP                               0x40
#define STATE_READY                                  0x50
#define STATE_XRAY_ON                                0x60


#define STATE_FAULT_HOLD                             0x80
#define STATE_FAULT_RESET_HOLD                       0x86
#define STATE_FAULT_LATCH_DECISION                   0x8A
//#define STATE_FAULT_RESET                            0x90
#define STATE_FAULT_SYSTEM                           0xA0
#define STATE_FAULT_WARMUP                           0xB0
#define STATE_FAULT_STANDBY                          0xC0





#define EEPROM_REGISTER_HTR_MAG_HEATER_CURRENT                      0x0000
#define EEPROM_REGISTER_HTR_MAG_MAGNET_CURRENT_HIGH_ENERGY          0x0001
#define EEPROM_REGISTER_HTR_MAG_MAGNET_CURRENT_LOW_ENERGY           0x000C
#define EEPROM_REGISTER_AFC_HOME_POSITION                           0x0005
#define EEPROM_REGISTER_AFC_OFFSET                                  0x0009
#define EEPROM_REGISTER_AFC_AFT_CONTROL_VOLTAGE_HIGH_ENERGY         0x000A
#define EEPROM_REGISTER_AFC_AFT_CONTROL_VOLTAGE_LOW_ENERGY          0x000B

#define EEPROM_REGISTER_LAMBDA_HIGH_ENERGY_SET_POINT                0x0010
#define EEPROM_REGISTER_LAMBDA_LOW_ENERGY_SET_POINT                 0x0011
#define EEPROM_REGISTER_REMOTE_IP_ADDRESS                           0x0018
#define EEPROM_REGISTER_IP_ADDRESS                                  0x001A
#define EEPROM_REGISTER_EEPROM_OK_CHECK                             0x001D
#define EEPROM_REGISTER_TOP_LEVEL_SERIAL_NUMBER                     0x001F

#define EEPROM_REGISTER_GUN_DRV_HTR_VOLTAGE                         0x0020
#define EEPROM_REGISTER_GUN_DRV_HIGH_PULSE_TOP                      0x0021
#define EEPROM_REGISTER_GUN_DRV_LOW_PULSE_TOP                       0x0022
#define EEPROM_REGISTER_GUN_DRV_CATHODE                             0x0023


#endif
