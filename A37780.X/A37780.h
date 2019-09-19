#ifndef __A37780_H
#define __A37780_H

#include <xc.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>
#include <uart.h>

#include "ETM.h"
//#include "TCPmodbus.h"
//#include "ETM_LINAC_MODBUS.h"



//#include "ETMmodbus.h"
#include "P1395_CAN_MASTER.h"


//#define __WATCHDOG_MASTER_EMULATION

#define FCY_CLK  20000000

/*
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
*/

/*

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
*/


/*

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
*/


/*
  Hardware Module Resource Usage

  CAN1 - Can module

  Timer1 - Used by ETMTick

  Timer2 - USED FOR TIMING TRIGGER DELAYS (Output Compare Module) and IC Module
  Timer3 - Reserved for IC Module (IF NEEDED)

  Timer4 - Setting aside for PRF Limit
  Timer5 - Unused at this time (Previously reserved for Can Module)


  UART1 - Reserved for Future Usage
  UART2 - Serial Dose

  INT1 - Trigger Input
  INT2 - Trigger 2 Input
  INT3 - Ethernet Chip Interrupt (not used)
  INT4 - Customer I/O Interupt (if needed)
  
  SPI1 - Ethernet Chip
  SPI2 - EEPROM and I/O Expander

  I2C  - Not used/enabled
  
  IC1 - Trigger Input
  IC3 - Fan Tac Sensor
  IC5 - Arc Detect 2
  IC6 - Arc Detect 1

  Analog Module
  AN8 - Off Time Analog

  Trigger Modules
  OC1 - Grid Trigger
  OC2 - PFN Trigger
  OC3 - HVPS Inhibit
  OC4 - Magnetron Current Sample
  OC5 - AFC Sample
  OC6 - Target Current Sample
  OC7 - Spare Trigger
  OC8 - Balanced Output 1

 */


// ----------------- IO PIN CONFIGURATION -------------------- //
/*
  ALL Pins to be left as inputs unless specificly needed as outputs
*/


// ----------------- DIGITAL OUTPUT PINS --------------- //
/*

  RA6  - LED Operation
  RA7  - PIC LAMP OUT SPARE 1
  RA9  - PIC DIGITAL OUT 6
  RA10 - PIC DIGITAL OUT 5

  RB6  - PIC DIGITAL OUT 2  
  RB7  - PIC DIGITAL OUT 1
  RB9  - PIC DIGITAL OUT 4
  RB10 - PIC DIGITAL OUT 3
  RB11 - PIC DIGITAL OUT 8
  RB12 - PIC DIGITAL OUT 7
  RB13 - PIC DIGITAL OUT 9
  RB14 - DELAY PGM CLK
  RB15 - DELAY PGM DO

  RC13 - HV Contactor Enable
  RC14 - AC Contactor Enable
  RC15 - LED GRN TEST POINT B


  RD0  -  OC1 - GRID TRIGGER
  RD1  -  OC2 - PFN TRIGGER
  RD2  -  OC3 - HVPS INHIBIT
  RD3  -  OC4 - MAGNETRON CURRENT SAMPLE
  RD4  -  OC5 - AFC SAMPLE
  RD5  -  OC6 - TARGET CURRENT SAMPLE
  RD6  -  OC7 - SPARE TRIGGER
  RD7  -  OC8 - BALANCED OUT 1
  RD11 - Gun Contactor Enable
  RD14 - GRID STOP PROG EN
  RD15 - GRID START PROG EN


  RG0  - LED Red Test Point A
  RG2  - UART 1 RX SOURCE SELECT
  RG9  - BALANCED OUT 2
  RG12 - PIC EXT 24V ENABLE
  RG13 - PIC LAMP OUT SPARE 2
  RG14 - PIC LAMP OUT X-RAY ON





*/

// ----------------- DIGITAL INPUT PINS --------------- //
/*
  RG15 - PIC INPUT 1
  RC1  - PIN INPUT 2
  RB5  - PIN INPUT 3
  RB4  - PIN INPUT 4
  RB3  - PIN INPUT 6
  RB2  - PIN INPUT 7

  RA12 - TRIGGER 1
  RD12 - TRIGGER 1
  RA15 - PIN INPUT 5 / INT 4

  RG1  - HV ON CMD
  
  RD10 - PIC FAN SENSOR
  RD9  - SPARE READY 1

  
  

*/


#define A37780_TRISA_VALUE 0b1111100100111111
#define A37780_TRISB_VALUE 0b0000000100111111
#define A37780_TRISC_VALUE 0b0001111111111111
#define A37780_TRISD_VALUE 0b0011011100000000
#define A37780_TRISF_VALUE 0b1111111111111111
#define A37780_TRISG_VALUE 0b1000110111111010



#define PIN_OUT_LED_GRN_OPERATION              _LATA6
#define PIN_OUT_LED_GRN_TEST_POINT_B           _LATC15
#define PIN_OUT_LED_RED_TEST_POINT_A           _LATG0
#define OLL_LED_ON                             0


#define PIN_OUT_UART_1_RX_SOURCE_SELECT        _LATG2
#define PIN_OUT_BALANCED_OUT_2                 _LATG9
#define PIN_OUT_PIC_EXT_24V_ENABLE             _LATG12
#define PIN_OUT_PIC_LAMP_OUT_SPARE_1           _LATA7
#define PIN_OUT_PIC_LAMP_OUT_SPARE_2           _LATG13
#define PIN_OUT_PIC_LAMP_OUT_X_RAY_ON          _LATG14


#define PIN_OUT_PIC_DIGITAL_OUT_1              _LATB7
#define PIN_OUT_PIC_DIGITAL_OUT_2              _LATB6
#define PIN_OUT_PIC_DIGITAL_OUT_3              _LATB10
#define PIN_OUT_PIC_DIGITAL_OUT_4              _LATB9
#define PIN_OUT_PIC_DIGITAL_OUT_5              _LATA10
#define PIN_OUT_PIC_DIGITAL_OUT_6              _LATA9
#define PIN_OUT_PIC_DIGITAL_OUT_7              _LATB12
#define PIN_OUT_PIC_DIGITAL_OUT_8              _LATB11
#define PIN_OUT_PIC_DIGITAL_OUT_9              _LATB13

#define PIN_OUT_DELAY_PGM_CLK                  _LATB14
#define PIN_OUT_DELAY_PGM_DO                   _LATB15
#define PIN_OUT_GRID_STOP_PROG_EN              _LATD14
#define PIN_OUT_GRID_START_PROG_EN             _LATD15

#define PIN_OUT_HV_CONTACTOR_ENABLE            _LATC13
#define PIN_OUT_AC_CONTACTOR_ENABLE            _LATC14
#define PIN_OUT_GUN_CONTACTOR_ENABLE           _LATD11


#define PIN_IN_PIC_INPUT_1                     _RG15
#define PIN_IN_PIC_INPUT_2                     _RC1
#define PIN_IN_PIC_INPUT_3                     _RB5
#define PIN_IN_PIC_INPUT_4                     _RB4
#define PIN_IN_PIC_INPUT_5                     _RA15
#define PIN_IN_PIC_INPUT_6                     _RB3
#define PIN_IN_PIC_INPUT_7                     _RB2

#define PIN_IN_HV_ON_CMD                       _RG1
#define PIN_IN_PIC_FAN_SENSOR                  _RD10
#define PIN_IN_SPARE_READY_1                   _RD9
#define PIN_IN_TRIGGER_1                       _RA12



#define OLL_FRONT_PANEL_LIGHT_ON               0
#define OLL_FRONT_PANEL_LIGHT_OFF              1
#define FRONT_PANEL_AC_POWER                   PIN_OUT_PIC_LAMP_OUT_SPARE_1
#define FRONT_PANEL_BEAM_ENABLE                PIN_OUT_PIC_LAMP_OUT_SPARE_2
#define FRONT_PANEL_X_RAY_ON                   PIN_OUT_PIC_LAMP_OUT_X_RAY_ON

#define OLL_KEYLOCK_PANEL_SWITCH_POWER_DISABLED 1
#define OLL_KEYLOCK_PANEL_SWITCH_POWER_ENABLED  0
#define KEYLOCK_PANEL_SWITCH_EN                PIN_OUT_PIC_EXT_24V_ENABLE

#define OLL_DISCRETE_OUTPUT_LOW                1
#define OLL_DISCRETE_OUTPUT_HIGH               0
#define DISCRETE_OUTPUT_FAULT                  PIN_OUT_PIC_DIGITAL_OUT_5
#define DISCRETE_OUTPUT_POWER_ON               PIN_OUT_PIC_DIGITAL_OUT_7
#define DISCRETE_OUTPUT_WARMUP                 PIN_OUT_PIC_DIGITAL_OUT_9
#define DISCRETE_OUTPUT_STANDBY                PIN_OUT_PIC_DIGITAL_OUT_3
#define DISCRETE_OUTPUT_READY                  PIN_OUT_PIC_DIGITAL_OUT_2
#define DISCRETE_OUTPUT_X_RAY_ON               PIN_OUT_PIC_DIGITAL_OUT_6
#define DISCRETE_OUTPUT_SPARE                  0



#define DISCRETE_INPUT_X_RAY_ON                PIN_IN_PIC_INPUT_3
#define DISCRETE_INPUT_X_RAY_OFF               PIN_IN_PIC_INPUT_4
#define ILL_X_RAY_ON_XRAY_ENABLED              1


#define DISCRETE_INPUT_SYSTEM_ENABLE           PIN_IN_PIC_INPUT_1
#define ILL_SYSTEM_ENABLE                      0

#define BEAM_ENABLE_INPUT                      PIN_IN_HV_ON_CMD
#define ILL_BEAM_ENABLE                        0
#define ILL_BEAM_DISABLED                      1

#define PIN_TRIGGER_IN                         PIN_IN_TRIGGER_1
#define ILL_TRIGGER_ACTIVE                     1



#define PIN_GRID_TRIGGER                       _LATD0
#define PIN_PFN_TRIGGER                        _LATD1
#define PIN_HVPS_INHIBIT                       _LATD2
#define PIN_MAGNETRON_CURRENT_SAMPLE           _LATD3
#define PIN_AFC_SAMPLE                         _LATD4
#define PIN_TARGET_CURRENT_SAMPLE              _LATD5
#define PIN_SPARE_TRIGGER                      _LATD6
#define PIN_BALANCED_OUT_1                     _LATD7


#define OLL_INHIBIT_HVPS                       1



#define CONTACTOR_OPEN                         0
#define CONTACTOR_CLOSED                       1



















// --------------- CONFIGURE TMR2 MODULE ----------------------- //
#define T2CON_VALUE                    (T2_OFF & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_1 & T2_32BIT_MODE_OFF & T2_SOURCE_INT)

#define T2CON_VALUE_TIMER_ON_SCALE_1_1  (T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_1 & T2_32BIT_MODE_OFF & T2_SOURCE_INT)
#define T2CON_VALUE_TIMER_OFF_SCALE_1_1 (T2_OFF & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_1 & T2_32BIT_MODE_OFF & T2_SOURCE_INT)


// ------------------------ CONFIGURE ADC MODULE ------------------- //
#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_SAMPLES_PER_INT_16 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_10Tcy)
#define ADPCFG_SETTING          (ENABLE_AN8_ANA)
#define ADCSSL_SETTING          0x0000
#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN8 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN8 & ADC_CH0_NEG_SAMPLEB_VREFN)


typedef struct {
  /*
  // This is a 16 word Block that is written / read from EEPROM as a group
  unsigned long long pulse_counter_48_bit;  // DPARKER Change this to a 64 bit number and ignore the high word and reaarage the data so the high word isn't sent to the EEPROM
  unsigned long arc_counter;
  unsigned long system_hv_on_seconds;
  unsigned long system_powered_seconds;
  unsigned long system_xray_on_seconds;
  unsigned long last_recorded_warmup_seconds;
  unsigned long holding_bits_for_warmup; // Higest 12 bits = thyratron warmup, middle 10 bits = magnetron heater warmup, lowest 10 bits = gun heater warmup
  unsigned int reserved_crc_eeprom_page_0;
  */

  unsigned int thyratron_warmup_remaining; 
  unsigned int magnetron_warmup_remaining;
  unsigned int gun_warmup_remaining;
  unsigned int gun_heater_holdoff_timer;
  unsigned int drive_up_timer;
  
  unsigned int startup_counter;

  unsigned long time_seconds_now;

  unsigned int reset_requested;  //DPARKER evaulate this
  unsigned int reset_hold_timer;
  unsigned int warmup_done;

  unsigned int access_mode;
  unsigned int service_passcode;
  unsigned int etm_passcode;

  unsigned int eeprom_write_status;


  unsigned int shutdown_counter;

  unsigned char dose_level;
  unsigned char pulse_counter;

  unsigned int single_dual_energy_mode_selection;

  unsigned int high_voltage_on_fault_counter;
  unsigned int drive_up_fault_counter;
  unsigned int eeprom_failure;
  
  unsigned int x_ray_on_off_mismatch_counter;
  unsigned int x_ray_on_while_beam_disabled_counter;
  unsigned int x_ray_on_wrong_state_counter;

  unsigned int adc_reading_at_turn_on;

  
  TYPE_DIGITAL_INPUT pfn_fan_fault_input;
  TYPE_DIGITAL_INPUT x_ray_on_mismatch_input;
  TYPE_DIGITAL_INPUT x_ray_on_without_beam_enable_input;
  TYPE_DIGITAL_INPUT x_ray_on_wrong_state_input;


  unsigned int *ram_ptr_a;
  unsigned int *ram_ptr_b;
  unsigned int *ram_ptr_c;
  
} A37780GlobalVars;


typedef struct {
  unsigned int trigger_period_too_short_count;
  unsigned int trigger_width_too_short_count;
  unsigned int trigger_not_valid_count;
  unsigned int external_trigger_when_internal_selected_count;
} FAULTVars;


#define average_output_power_watts                           local_data_ecb.log_data[14]



#define TRIGGER_GRID_TRIGGER             1
#define TRIGGER_PFN_TRIGGER              2
#define TRIGGER_HVPS_INHIBIT             3
#define TRIGGER_MAGNETRON_I_SAMP         4
#define TRIGGER_AFC_SAMPLE               5
#define TRIGGER_TARGET_I_SAMP            6
#define TRIGGER_SPARE                    7
#define TRIGGER_BALANCED_OUT_1           8

#define DOSE_LEVEL_CARGO_HIGH            0b11
#define DOSE_LEVEL_CARGO_LOW             0b10
#define DOSE_LEVEL_CAB_HIGH              0b01
#define DOSE_LEVEL_CAB_LOW               0b00




extern A37780GlobalVars global_data_A37780;


#define _FAULT_WAVEGUIDE_ARC                            _FAULT_0
#define _FAULT_REPEATED_DRIVE_UP_FAULT                  _FAULT_1
#define _FAULT_REPEATED_HV_ON_FAULT                     _FAULT_2
#define _FAULT_EEPROM_FAILURE                           _FAULT_3
// FAULT 4 is reserved for magnetron over power
#define _FAULT_WATCHDOG_ERROR                           _FAULT_5
#define _FAULT_X_RAY_MISMATCH                           _FAULT_6
#define _FAULT_X_RAY_ON_WRONG_STATE                     _FAULT_7
#define _FAULT_X_RAY_ON_BEAM_DISABLED                   _FAULT_8

#define _FAULT_PFN_FAN_FAULT                            _FAULT_9





// DPAKRER  - Need to evaluate how these are used under new control system
#define _STATUS_PERSONALITY_LOADED                      _LOGGED_STATUS_0
#define _STATUS_DRIVE_UP_TIMEOUT                        _LOGGED_STATUS_1

#define _STATUS_LAST_RESET_WAS_POWER_CYCLE              _NOT_LOGGED_STATUS_0




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
#define STATE_SAFETY_SELF_TEST                       0x12
#define STATE_WAITING_FOR_POWER_ON                   0x16 
#define STATE_WAITING_FOR_INITIALIZATION             0x18
#define STATE_WARMUP                                 0x20
#define STATE_STANDBY                                0x30
#define STATE_DRIVE_UP                               0x40
#define STATE_READY                                  0x50
#define STATE_XRAY_ON                                0x60


#define STATE_FAULT_WARMUP                           0xA0
#define STATE_FAULT_SYSTEM                           0xA2
#define STATE_FAULT_HOLD                             0xA4
#define STATE_FAULT_RESET                            0xA6
#define STATE_SAFE_POWER_DOWN                        0xA8



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





















#endif
