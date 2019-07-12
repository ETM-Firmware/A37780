#include "A37780.h"
#include "FIRMWARE_VERSION.h"
#include "A37780_CONFIG.h"
#include "TCPmodbus.h"
#include "ETM_ANALOG.h"
#include "ETM_TICK.h"


#define EEPROM_PAGE_ECB_COUNTER_AND_TIMERS           0x00
#define EEPROM_PAGE_ECB_BOARD_CONFIGURATION          0x7E
#define EEPROM_PAGE_ECB_DOSE_SETTING_0               0x40
#define EEPROM_PAGE_ECB_DOSE_SETTING_1               0x41
#define EEPROM_PAGE_ECB_DOSE_SETTING_ALL             0x50

#define EEPROM_PAGE_ECB_DOSE_SETTING_0_FACTORY_DEFAULT               0x60
#define EEPROM_PAGE_ECB_DOSE_SETTING_1_FACTORY_DEFAULT               0x61
#define EEPROM_PAGE_ECB_DOSE_SETTING_ALL_FACTORY_DEFAULT             0x70

#define EEPROM_PAGE_ECB_DOSE_SETTING_0_CUSTOMER_BACKUP               0x10
#define EEPROM_PAGE_ECB_DOSE_SETTING_1_CUSTOMER_BACKUP               0x11
#define EEPROM_PAGE_ECB_DOSE_SETTING_ALL_CUSTOMER_BACKUP             0x20



#define EEPROM_WRITE_SUCCESSFUL                      0x100
#define EEPROM_WRITE_FAILURE                         0x200
#define EEPROM_WRITE_WAITING                         0x300



#define ACCESS_MODE_DEFAULT           100
#define ACCESS_MODE_SERVICE           200
#define ACCESS_MODE_ETM               300

#define ACCESS_MODE_SERVICE_PW_FIXED  0xF1A7
#define ACCESS_MODE_ETM_PW_FIXED      0x117F

#define WATCHDOG_TIMEOUT_MILLISEC             65
unsigned long watchdog_timeout_holding_var;
unsigned long ten_millisecond_holding_var;

unsigned int watchdog_test_counter;  //DPARKER REMOVE THIS

//TYPE_PUBLIC_ANALOG_INPUT analog_3_3V_vmon;
TYPE_PUBLIC_ANALOG_INPUT analog_5V_vmon;
//unsigned int test_0;
//unsigned int test_1;
//unsigned int test_2;
//unsigned int test_3;

//unsigned int scope_data_raw[260];

//TYPE_SCOPE_DATA   *scope_channel_1;  // Points to scope_data_raw[0];
//TYPE_SCOPE_DATA   *scope_channel_2;  // Points to scope_data_raw[130]
//TYPE_HV_VMON_DATA *scope_hvps;       // Points to scope_data_raw[0];

//TYPE_MAGNETRON_CURRENT_DATA  magnetron_current_data[4]; // We can save up to 4 pulses in ram


unsigned int test_uart_data_recieved;
unsigned int test_ref_det_recieved;
unsigned int test_ref_det_good_message;



void SendWatchdogResponse(unsigned int pulse_count);
unsigned int LookForWatchdogMessage(void);
void TestSendWatchdogMessage(void);


void CopyCurrentConfig(unsigned int destination);
void LoadConfig(unsigned int source);

#define USE_FACTORY_DEFAULTS 0
#define USE_CUSTOMER_BACKUP  1





void CRCTest(void);


//BUFFERBYTE64 uart1_input_buffer;
BUFFERBYTE64 uart2_input_buffer;
BUFFERBYTE64 uart2_output_buffer;


unsigned int a_ready;
unsigned int a_sent;
unsigned int b_ready;
unsigned int b_sent;



// ------------------ PROCESSOR CONFIGURATION ------------------------//
_FOSC(ECIO_PLL16 & CSW_FSCM_OFF);                                         // 5Mhz External Osc created 20Mhz FCY
_FWDT(WDT_OFF & WDTPSA_512 & WDTPSB_8);                                    // 8 Second watchdog timer 
_FBORPOR(PWRT_4 & NONE & PBOR_OFF & MCLR_EN);                             // 4ms Power up timer, Low Voltage Reset disabled
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);  // 
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);         //
_FGS(CODE_PROT_OFF);                                                      //
_FICD(PGD);                                                               //


const unsigned int FilamentLookUpTable[64] = {FILAMENT_LOOK_UP_TABLE_VALUES_FOR_MG7095_V2};



// ---------------------- Control Functions ---------------------------- //
void DoStateMachine(void);
/*
  This runs the state machine.
  Typical loop time is a couple hundred uS
  This can be extended significantly by spi/i2c communication if waiting for those functions to complete
*/

unsigned int CheckWarmupFailure(void);
unsigned int CheckWarmupFault(void);
unsigned int CheckConfigurationFault(void);
unsigned int CheckStandbyFault(void);
unsigned int CheckFaultLatching(void);
unsigned int CheckHVOnFault(void);
unsigned int CheckCoolingFault(void);
unsigned int CheckGunHeaterOffFault(void);


void DoA36507(void);
/*
  This is called every time through the control loop.
  Some tasks are executed every time this function is called
  Some tasks are executed once every 10ms  - those tasks inside "if (_T2IF)"
*/

void ExecuteEthernetCommand(void); 
/*
  This looks to see if there any ethernet commands to execute
  If a command is available this executes that command
*/

unsigned int CalculatePulseEnergyMilliJoules(unsigned int lambda_voltage); 
/*
  This caluclates the energy per pulse based on a lambda voltage
  This is just .5*C*V^2 (where C is the PFN capacitance)
*/

void UpdateHeaterScale(void);
/*
  This function grabs the highest lambda set point (from mode A or mode B) and calulates the energy per pulse from that voltage
  Energy per pulse is multiplied by PRF (from Pulse Sync board)
  The energy is used to look up the heater scale factor from the FilamentLookUpTable
  The heater set poing scaled is then set by multipling the set point by the scale factor
*/




// -------------------- STARTUP Helper Functions ---------------------- //
void InitializeA36507(void);
/*
  This initialized the processor and all of it's internal/external hardware and software modules
  It should only be called once
*/

void CalculateHeaterWarmupTimers(void);
/*
  This reads the last time a heater was warm from EEPROM and caluclates how long it needs to be warmed up for
*/

void ReadSystemConfigurationFromEEProm(void);
/*
  This loads the configuration for a given personality from the external EEPROM
*/

void FlashLeds(void);
/*
  LED Helper function
  This is called while the ECB is initializing and causes the LED to flash in the "initializing" pattern
*/




// -------------------------------- Ethernet Interface helper Functions ---------------------------- //
void ZeroSystemPoweredTime(void);
/*
  Sets the system powered, hv on, and xray on second counter to Zero.
*/

void LoadDefaultSystemCalibrationToEEProm(void);
/*
  Loads the default system calibration parmaeters from flash program memmory into the EEPROM
  This is used to setup the eeprom when the machine is first set up or to return the EEPORM to a known state.
*/


// ------------------- Global Variables ------------------------------ //
A36507GlobalVars global_data_A36507;         
/* 
   This is the Data structure for the global variables
   Every variable that is not a structure or on the stack should be within this structure
   This places variables at a fixed place in memory relative to each other which we depend on in some cases
   It also find that this makes it easier to work with debugger
*/


// -------------------- Local Structures ----------------------------- //
RTC_DS3231 U6_DS3231;                        
/* 
   This is the Data structure for the Real Time Clock
   See DS3231.h For more information
*/




int main(void) {
  
  global_data_A36507.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}



void DoStateMachine(void) {
  
  switch (global_data_A36507.control_state) {

    
  case STATE_STARTUP:
    InitializeA36507();
    global_data_A36507.gun_heater_holdoff_timer = 0;
    _SYNC_CONTROL_GUN_DRIVER_DISABLE_HTR = 1;
    global_data_A36507.control_state = STATE_WAITING_FOR_INITIALIZATION;
    SendToEventLog(LOG_ID_ENTERED_STATE_STARTUP);
    if (_STATUS_LAST_RESET_WAS_POWER_CYCLE) {
      _SYNC_CONTROL_CLEAR_DEBUG_DATA = 1;
    }
    break;

    
  case STATE_WAITING_FOR_INITIALIZATION:
    SendToEventLog(LOG_ID_ENTERED_STATE_WAITING_FOR_INITIALIZATION);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    _STATUS_PERSONALITY_LOADED = 1;
    personality_loaded = 1;
    if (global_data_A36507.eeprom_failure) {
      global_data_A36507.control_state = STATE_FAULT_SYSTEM;
    }
    while (global_data_A36507.control_state == STATE_WAITING_FOR_INITIALIZATION) {
      DoA36507();
      FlashLeds();
      if ((!CheckConfigurationFault()) && (global_data_A36507.startup_counter >= 300)) {
      	global_data_A36507.control_state = STATE_WARMUP;
	SendToEventLog(LOG_ID_ALL_MODULES_CONFIGURED);
      }
    }
    break;
    

  case STATE_WARMUP:
    // Note that the warmup timers start counting in "Waiting for Initialization"
    SendToEventLog(LOG_ID_ENTERED_STATE_WARMUP);
    _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (global_data_A36507.control_state == STATE_WARMUP) {
      DoA36507();
      if (global_data_A36507.warmup_done) {
	global_data_A36507.control_state = STATE_STANDBY;
	SendToEventLog(LOG_ID_WARMUP_DONE);
      }
      if (CheckWarmupFault()) {
	global_data_A36507.control_state = STATE_FAULT_WARMUP;
      }
    }
    break;


  case STATE_FAULT_WARMUP:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_WARMUP);
    _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (global_data_A36507.control_state == STATE_FAULT_WARMUP) {
      DoA36507();
      if (!CheckWarmupFault()) {
	global_data_A36507.control_state = STATE_WARMUP;
      }
      if (CheckWarmupFailure()) {
	global_data_A36507.control_state = STATE_FAULT_SYSTEM;
      }
    }
    break;
    

  case STATE_FAULT_SYSTEM:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_SYSTEM);
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (1) {
      DoA36507();
    }
    break;

    
  case STATE_STANDBY:
    SendToEventLog(LOG_ID_ENTERED_STATE_STANDBY);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
     while (global_data_A36507.control_state == STATE_STANDBY) {
      DoA36507();
      if (!_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_DRIVE_UP;
      }
      if (CheckStandbyFault()) {
	global_data_A36507.control_state = STATE_FAULT_LATCH_DECISION;
      }
     }
    break;


  case STATE_FAULT_STANDBY:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_STANDBY);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (global_data_A36507.control_state == STATE_FAULT_STANDBY) {
      DoA36507();
      if (!CheckStandbyFault()) {
	global_data_A36507.control_state = STATE_STANDBY;
      }
      if (_FAULT_X_RAY_ON_LOGIC_ERROR) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
      if (CheckWarmupFault()) {
	global_data_A36507.control_state = STATE_FAULT_WARMUP;
      }
      if ((global_data_A36507.thyratron_warmup_remaining > 0) ||
	  (global_data_A36507.magnetron_warmup_remaining > 0) ||
	  (global_data_A36507.gun_warmup_remaining > 0)) {
	global_data_A36507.control_state = STATE_FAULT_WARMUP;
      }

     }
    break;


  case STATE_DRIVE_UP:
    SendToEventLog(LOG_ID_ENTERED_STATE_DRIVE_UP);
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (global_data_A36507.control_state == STATE_DRIVE_UP) {
      DoA36507();
      if (!CheckHVOnFault()) {
	global_data_A36507.control_state = STATE_READY;
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_STANDBY;
      }
      if (CheckStandbyFault()) {
	global_data_A36507.drive_up_fault_counter++;
	global_data_A36507.control_state = STATE_FAULT_LATCH_DECISION;
      }
    }
    break;
    

  case STATE_READY:
    SendToEventLog(LOG_ID_ENTERED_STATE_READY);
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 0;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 1;
    global_data_A36507.drive_up_fault_counter = 0;
    _STATUS_DRIVE_UP_TIMEOUT = 0;
     while (global_data_A36507.control_state == STATE_READY) {
      DoA36507();
      if (_PULSE_SYNC_CUSTOMER_XRAY_OFF == 0) {
	global_data_A36507.control_state = STATE_XRAY_ON;
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_DRIVE_UP;
      }
      if (CheckHVOnFault()) {
	global_data_A36507.control_state = STATE_FAULT_LATCH_DECISION;
	global_data_A36507.high_voltage_on_fault_counter++;
      }
    }
    break;


  case STATE_XRAY_ON:
    SendToEventLog(LOG_ID_ENTERED_STATE_XRAY_ON);
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 0;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    global_data_A36507.high_voltage_on_fault_counter = 0;
    while (global_data_A36507.control_state == STATE_XRAY_ON) {
      DoA36507();
      if (_PULSE_SYNC_CUSTOMER_XRAY_OFF) {
	global_data_A36507.control_state = STATE_READY;
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_READY;
      }
      if (CheckHVOnFault()) {
	//global_data_A36507.control_state = STATE_FAULT_LATCH_DECISION;
	global_data_A36507.control_state = STATE_FAULT_HOLD;  // Why would you ever need to make this decision if X-Rays were on
      }
    }
    break;


  case STATE_FAULT_LATCH_DECISION:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_LATCH_DECISION);
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    global_data_A36507.reset_requested = 0;
    global_data_A36507.reset_hold_timer = 0;
    while (global_data_A36507.control_state == STATE_FAULT_LATCH_DECISION) {
      DoA36507();
      if (global_data_A36507.reset_hold_timer > MINIMUM_FAULT_HOLD_TIME) { 
	// Need to wait in this state for X_RAY_ON status to propigate from the pulse sync board
	global_data_A36507.control_state = STATE_FAULT_RESET_HOLD;
      }
      if (CheckFaultLatching()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
    }
    break;


  case STATE_FAULT_HOLD:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_HOLD);
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    global_data_A36507.reset_requested = 0;
      while (global_data_A36507.control_state == STATE_FAULT_HOLD) {
      DoA36507();
      if (global_data_A36507.reset_requested) {
	global_data_A36507.control_state = STATE_FAULT_RESET_HOLD;
      }
     }
    break;
    

  case STATE_FAULT_RESET_HOLD:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_RESET_HOLD);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    global_data_A36507.reset_requested = 0;
    global_data_A36507.reset_hold_timer = 0;
    while (global_data_A36507.control_state == STATE_FAULT_RESET_HOLD) {
      DoA36507();
      if (global_data_A36507.reset_hold_timer > FAULT_RESET_HOLD_TIME) { 
	global_data_A36507.control_state = STATE_FAULT_STANDBY;
      }
    }
    break;

    

    
  default:
    global_data_A36507.control_state = STATE_FAULT_SYSTEM;
    break;
  }
}


/*
  Logging Data

  * State Changes                - State changes are logged in the state machine
  * Board Connection Status      - Boards disconnecting and reconnecting is logged by the Can Master Module
  * Board Ready/Not Ready Status - This logged as part of Do10msTicToc
  * Board Faults                 - If a board is faulted ()



*/

unsigned int CheckWarmupFailure(void) {
  // DPARKER need to add this
  // Look for failure of magnetron or gun heater
  return 0;
}

unsigned int CheckWarmupFault(void) {
  // What can go wrong in warmup???

  // The gun driver can report that the heater is not on
  // The heater/magnet board can report that the heater is not on
  // We can loose communication with Gun Driver, Heater Magnet, or Ion Pump Board
  // The Gun Driver, Heater Magnet reports that it is not configured
  // The Ion Pump can detect too much current and shut down the gun heater  (We don't need to check this, because the gun heater will turn off)
  
  // There is nothing the user can do about any of these.  The can just get a status that something is not right
  // If the fault is unrecoverable(Gun Heater or Magnetron Heater) they can power cycle the system


#ifndef __IGNORE_HEATER_MAGNET_MODULE
  if (!_HEATER_MAGNET_HEATER_OK) {
    return 1;
  }
  if (!board_com_ok.heater_magnet_board) {
    return 1;
  }
  if (_HEATER_MAGNET_NOT_CONFIGURED) {
    return 1;
  }
#endif


#ifndef __IGNORE_GUN_DRIVER_MODULE
  if (!board_com_ok.gun_driver_board) {
    return 1;
  }
  if (_GUN_DRIVER_NOT_CONFIGURED) {
    return 1;
  }
#endif

#ifndef __IGNORE_ION_PUMP_MODULE
  if (!board_com_ok.ion_pump_board) {
    return 1;
  }
#endif

  return 0;
}


unsigned int CheckConfigurationFault(void) {
#ifndef __IGNORE_HV_LAMBDA_MODULE
  if (!board_com_ok.hv_lambda_board || _HV_LAMBDA_NOT_CONFIGURED) {
    return 1;
  }
#endif
  
#ifndef __IGNORE_ION_PUMP_MODULE  
  if (!board_com_ok.ion_pump_board || _ION_PUMP_NOT_CONFIGURED) {
    return 1;
  }
#endif
  
#ifndef __IGNORE_AFC_MODULE
  if (!board_com_ok.afc_board || _AFC_NOT_CONFIGURED) {
    return 1;
  }
#endif  

#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  if (!board_com_ok.cooling_interface_board || _COOLING_NOT_CONFIGURED) {
    return 1;
  }
#endif  

#ifndef __IGNORE_HEATER_MAGNET_MODULE
  if (!board_com_ok.heater_magnet_board || _HEATER_MAGNET_NOT_CONFIGURED) {
    return 1;
  }
#endif
  
#ifndef __IGNORE_GUN_DRIVER_MODULE
  if (!board_com_ok.gun_driver_board || _GUN_DRIVER_NOT_CONFIGURED) {
    return 1;
  }
#endif

#ifndef __IGNORE_PULSE_CURRENT_MODULE
  if (!board_com_ok.magnetron_current_board || _PULSE_MON_NOT_CONFIGURED) {
    return 1;
  }
#endif

#ifndef __IGNORE_PULSE_SYNC_MODULE
  if (!board_com_ok.pulse_sync_board || _PULSE_SYNC_NOT_CONFIGURED) {
    return 1;
  }
#endif
  
  return 0;
}


// DPARKER
unsigned int CheckFaultLatching(void) {
  if (_FAULT_REPEATED_DRIVE_UP_FAULT || _FAULT_REPEATED_HV_ON_FAULT || _FAULT_X_RAY_ON_LOGIC_ERROR) {
    return 1;
  }
  if (_PULSE_MON_FALSE_TRIGGER) {
    // If a false trigger is detected we must hold the fault
    return 1;
  }
  if (_PULSE_SYNC_FAULT_X_RAY_MISMATCH) {
    return 1;
  }
  if (!_PULSE_SYNC_CUSTOMER_XRAY_OFF) {
    // The fault happened when X-Rays were on, need to latch the fault
    return 1;
  }


  return 0;
}


unsigned int CheckStandbyFault(void) {
  unsigned int faults = 0;

  /*
    Loose Connection with any board - This is covered by CheckAllModulesConfigured
    Any Board Reports not configured - This is covered by CheckAllModulesConfigured
    Any Board Reports a fault
  */

  if (_FAULT_REGISTER) {
    return 1;
  }
  
  if (CheckConfigurationFault()) {
    return 1;
  }
  
#ifndef __IGNORE_HV_LAMBDA_MODULE
  faults |= _HV_LAMBDA_FAULT_REGISTER; 
#endif
  
#ifndef __IGNORE_ION_PUMP_MODULE  
  faults |= _ION_PUMP_FAULT_REGISTER;
#endif
  
#ifndef __IGNORE_AFC_MODULE
  faults |= _AFC_FAULT_REGISTER;
#endif  
  
#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  faults |= _COOLING_FAULT_REGISTER;
#endif  
  
#ifndef __IGNORE_HEATER_MAGNET_MODULE
  faults |= _HEATER_MAGNET_FAULT_REGISTER;
#endif
  
#ifndef __IGNORE_GUN_DRIVER_MODULE
  faults |= _GUN_DRIVER_FAULT_REGISTER;
#endif
  
#ifndef __IGNORE_PULSE_CURRENT_MODULE
  faults |= _PULSE_MON_FAULT_REGISTER;
#endif
  
#ifndef __IGNORE_PULSE_SYNC_MODULE
  faults |= _PULSE_SYNC_FAULT_REGISTER;
#endif
  
  if (faults) {
    return 1;
  }
  
  if (global_data_A36507.drive_up_timer > DRIVE_UP_TIMEOUT) {
    _STATUS_DRIVE_UP_TIMEOUT = 1;
    SendToEventLog(LOG_ID_DRIVE_UP_TIMEOUT);
    return 1;
  }

  return 0;
}


unsigned int CheckCoolingFault(void) {
  // Check to see if cooling is present
#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  if (!board_com_ok.cooling_interface_board) {
    return 1;
  }
  if (!_COOLING_FLOW_OK) {
    return 1;
  }
#endif
  return 0;
}


unsigned int CheckGunHeaterOffFault(void) {
  // Check to see if there is an active over current condition in the ion pump

#ifndef __IGNORE_ION_PUMP_MODULE
  if (_ION_PUMP_OVER_CURRENT_ACTIVE) {
    global_data_A36507.gun_heater_holdoff_timer = 0;
    return 1;
  }
  if (!board_com_ok.ion_pump_board) {
    return 1;
  }
  if (_ION_PUMP_NOT_CONFIGURED) {
    return 1;
  }
#endif
  if (global_data_A36507.gun_heater_holdoff_timer < GUN_HEATER_HOLDOFF_AT_STARTUP) {
    return 1;
  }
  if (global_data_A36507.thyratron_warmup_remaining > global_data_A36507.gun_warmup_remaining) {
    if (global_data_A36507.gun_heater_holdoff_timer < (GUN_HEATER_HOLDOFF_AT_STARTUP + GUN_HEATER_ADDITONAL_HOLDOFF_COLD)) {
      return 1;
    }
  }

  return 0;
}


unsigned int CheckHVOnFault(void) {
  /*
    CheckStandbyFault();
    Any Board Reports Not Ready
  */

  if (_FAULT_REGISTER) {
    return 1;
  }
  
  if (CheckStandbyFault()) {
    return 1;
  }
  
#ifndef __IGNORE_HV_LAMBDA_MODULE
  if (_HV_LAMBDA_NOT_READY) {
    return 1;
  }
#endif
  
#ifndef __IGNORE_ION_PUMP_MODULE  
  if (_ION_PUMP_NOT_READY) {
    return 1;
  }
#endif

#ifndef __IGNORE_AFC_MODULE
  if (_AFC_NOT_READY) {
    return 1;
  }
#endif  

#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  if (_COOLING_NOT_READY) {
    return 1;
  }
#endif  

#ifndef __IGNORE_HEATER_MAGNET_MODULE
  if (_HEATER_MAGNET_NOT_READY) {
    return 1;
  }
#endif
  
#ifndef __IGNORE_GUN_DRIVER_MODULE
  if (_GUN_DRIVER_NOT_READY) {
    return 1;
  }
#endif

#ifndef __IGNORE_PULSE_CURRENT_MODULE
  if (_PULSE_MON_NOT_READY) {
    return 1;
  }
#endif

#ifndef __IGNORE_PULSE_SYNC_MODULE
  if (_PULSE_SYNC_NOT_READY) {
    return 1;
  }
#endif

  return 0;
}


void UpdateDebugData(void) {
  /*
  debug_data_ecb.debug_reg[0x0] = global_data_A36507.high_voltage_on_fault_counter; 
  debug_data_ecb.debug_reg[0x1] = global_data_A36507.drive_up_fault_counter;
  debug_data_ecb.debug_reg[0x3] = test_ref_det_recieved; 
  debug_data_ecb.debug_reg[0x4] = test_ref_det_good_message;
  debug_data_ecb.debug_reg[0x5] = global_data_A36507.most_recent_ref_detector_reading; 
  debug_data_ecb.debug_reg[0x6] = ETMTCPModbusGetErrorInfo(ERROR_SM_PROCESS_RESPONSE_TIMEOUT_ID); 
  debug_data_ecb.debug_reg[0x7] = ETMTCPModbusGetErrorInfo(ERROR_COUNT_SM_PROCESS_RESPONSE_TIMEOUT); 
  debug_data_ecb.debug_reg[0x8] = ETMTCPModbusGetErrorInfo(ERROR_COUNT_SM_SOCKET_OBTAINED_TIMEOUT); 
  debug_data_ecb.debug_reg[0x9] = ETMTCPModbusGetErrorInfo(COUNT_SM_SOCKET_OBTAINED_MSG_TX); 
  debug_data_ecb.debug_reg[0xA] = ETMTCPModbusGetErrorInfo(COUNT_SM_PROCESS_RESPONSE_MSG_RX); 
  debug_data_ecb.debug_reg[0xB] = ETMTCPModbusGetErrorInfo(ERROR_COUNT_SM_DISCONNECT);
  debug_data_ecb.debug_reg[0xC] = 12;
  debug_data_ecb.debug_reg[0xD] = 12;
  debug_data_ecb.debug_reg[0xE] = 12;
  debug_data_ecb.debug_reg[0xF] = 12; 
  */
  
  debug_data_ecb.debug_reg[0x0] = local_hvps_set_point_dose_0;
  debug_data_ecb.debug_reg[0x1] = local_hvps_set_point_dose_1;
  debug_data_ecb.debug_reg[0x2] = etm_can_master_next_pulse_level;
  debug_data_ecb.debug_reg[0x3] = etm_can_master_next_pulse_count;
  debug_data_ecb.debug_reg[0x4] = average_output_power_watts;
  //debug_data_ecb.debug_reg[0x5] = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_I2C_COUNT);
  //debug_data_ecb.debug_reg[0x6] = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_I2C_ERROR);
  //debug_data_ecb.debug_reg[0x7] = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_I2C_COUNT);
  debug_data_ecb.debug_reg[0x8] = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_I2C_ERROR);
  debug_data_ecb.debug_reg[0x9] = ETMTCPModbusGetErrorInfo(ERROR_COUNT_SM_PROCESS_RESPONSE_TIMEOUT);
  debug_data_ecb.debug_reg[0xA] = ETMTCPModbusGetErrorInfo(ERROR_COUNT_SM_SOCKET_OBTAINED_TIMEOUT);
  debug_data_ecb.debug_reg[0xB] = ETMTCPModbusGetErrorInfo(ERROR_SM_PROCESS_RESPONSE_TIMEOUT_ID);
  debug_data_ecb.debug_reg[0xC] = ETMTCPModbusGetErrorInfo(COUNT_SM_SOCKET_OBTAINED_MSG_TX);
  debug_data_ecb.debug_reg[0xD] = ETMTCPModbusGetErrorInfo(COUNT_SM_PROCESS_RESPONSE_MSG_RX);
  debug_data_ecb.debug_reg[0xE] = ETMTCPModbusGetErrorInfo(ERROR_COUNT_SM_DISCONNECT);
  debug_data_ecb.debug_reg[0xF] = global_data_A36507.system_serial_number;

  debug_data_ecb.debug_reg[0x7] = watchdog_test_counter;

  
}


void DoA36507(void) {
  
#ifdef __WATCHDOG_MASTER_EMULATION
  static unsigned long watchdog_transmit_timer_holding_var;

  ClrWdt();
  if (ETMTickRunOnceEveryNMilliseconds(20, &watchdog_transmit_timer_holding_var)) {
    TestSendWatchdogMessage();
  }
#endif
  
  
  if (LookForWatchdogMessage()) {
    // Clear the watchdog counter
    watchdog_test_counter++;
    watchdog_timeout_holding_var = ETMTickGet();
#ifndef __WATCHDOG_MASTER_EMULATION
    SendWatchdogResponse(watchdog_test_counter);
    if (_SYNC_CONTROL_RESET_ENABLE) {
      _FAULT_WATCHDOG_ERROR = 0;
    }
#endif
  }
  
  if (ETMTickGreaterThanNMilliseconds(WATCHDOG_TIMEOUT_MILLISEC, watchdog_timeout_holding_var)) {
    // There is a watchdog timeout
    if (_FAULT_WATCHDOG_ERROR == 1) {
      // There is a new watchdog fault, send to the event log
      //SendToEventLog(0xFF00);
    }
    _FAULT_WATCHDOG_ERROR = 1;
  }

  etm_can_master_sync_message.sync_1_ecb_state_for_fault_logic = global_data_A36507.control_state;
  etm_can_master_sync_message.sync_2 = 0x0123;
  etm_can_master_sync_message.sync_3 = 0x4567;

  ETMCanMasterDoCan();
  ETMLinacModbusUpdate();
  ExecuteEthernetCommand();

  if (global_data_A36507.eeprom_failure) {
    _FAULT_EEPROM_FAILURE = 1;
  }


  // Figure out if the customer has enabled XRAYs before they should have
  // If so set a fault that can only be cleared with a reset command
  if (!_PULSE_SYNC_CUSTOMER_XRAY_OFF) { 
    if ((global_data_A36507.control_state == STATE_WARMUP) ||
	(global_data_A36507.control_state == STATE_FAULT_WARMUP) ||
	(global_data_A36507.control_state == STATE_FAULT_STANDBY)) { 
      // Customer Enabled XRAYs when not ready
      _FAULT_X_RAY_ON_LOGIC_ERROR = 1;
    }
    if ((global_data_A36507.control_state == STATE_STANDBY) || 
	(global_data_A36507.control_state == STATE_FAULT_HOLD)) {
      
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	// Customer Enabled XRAYS, but not High Voltage durring one of the standby states
	_FAULT_X_RAY_ON_LOGIC_ERROR = 1;
      }
    }
  }  
    
  if (global_data_A36507.drive_up_fault_counter > MAX_DRIVE_UP_FAULTS) {
    _FAULT_REPEATED_DRIVE_UP_FAULT = 1;
  }
  
  if (global_data_A36507.high_voltage_on_fault_counter > MAX_HV_ON_FAULTS) {
    _FAULT_REPEATED_HV_ON_FAULT = 1;
  }
  
  // Update the cooling fault sync bit
  if (CheckCoolingFault()) {
    _SYNC_CONTROL_COOLING_FAULT = 1;
  } else {
    _SYNC_CONTROL_COOLING_FAULT = 0;
  }

  // Update the Gun Driver Heater Enable sync bit
  // DPARKER need to update the libraries to use the Gun Heater Disable Bit
  if (CheckGunHeaterOffFault()) {
    _SYNC_CONTROL_GUN_DRIVER_DISABLE_HTR = 1;
  } else {
    _SYNC_CONTROL_GUN_DRIVER_DISABLE_HTR = 0;
  }

    
  if (ETMTickRunOnceEveryNMilliseconds(10, &ten_millisecond_holding_var)) {
    // 10ms Timer has expired -- run periodic checks and updates

    // Load Local data into the registers for logging
  
    // Load log_data Memory for types that can not be mapped directly into memory
    local_data_ecb.log_data[0] = global_data_A36507.control_state;
    local_data_ecb.log_data[3] = ETMCanMasterGetPulsePRF();
    local_data_ecb.log_data[4] = global_data_A36507.thyratron_warmup_remaining;
    local_data_ecb.log_data[5] = global_data_A36507.magnetron_warmup_remaining;
    local_data_ecb.log_data[6] = global_data_A36507.gun_warmup_remaining;
    local_data_ecb.log_data[7] = _SYNC_CONTROL_WORD;
    (*(unsigned long*)&local_data_ecb.log_data[8])  = global_data_A36507.system_powered_seconds;
    (*(unsigned long*)&local_data_ecb.log_data[10]) = global_data_A36507.system_hv_on_seconds;
    (*(unsigned long*)&local_data_ecb.log_data[12]) = global_data_A36507.system_xray_on_seconds;  
    local_data_ecb.log_data[16] = *(unsigned int*)&board_com_ok;
    local_data_ecb.log_data[17] = global_data_A36507.most_recent_ref_detector_reading;
    local_data_ecb.log_data[19] = global_data_A36507.system_serial_number;
    mirror_cooling.local_data[0] = MAX_SF6_REFILL_PULSES_IN_BOTTLE;

    local_data_ecb.local_data[4] = global_data_A36507.access_mode;
    
    UpdateDebugData();  // Load the customized debugging data into the debugging registers
    
    if (global_data_A36507.control_state == STATE_DRIVE_UP) {
      global_data_A36507.drive_up_timer++;
    } else {
      global_data_A36507.drive_up_timer = 0;
    }
    global_data_A36507.startup_counter++;
    global_data_A36507.reset_hold_timer++;
    
    // Update the heater current based on Output Power
    UpdateHeaterScale();

    if (global_data_A36507.gun_heater_holdoff_timer <= (GUN_HEATER_HOLDOFF_AT_STARTUP + GUN_HEATER_ADDITONAL_HOLDOFF_COLD)) {
      global_data_A36507.gun_heater_holdoff_timer++;
    }


    /*
      The following tasks require use of the i2c bus which can hold the processor for a lot of time
      Need to schedule them at different point of a 1 second period
    */
    can_master_millisecond_counter += 10;

    // Run at 1 second interval
    if (can_master_millisecond_counter >= 1000) {
      can_master_millisecond_counter = 0;
      //SendToEventLog(0xD1DF);
    }

    // Run once a second at 0 milliseconds
    if (can_master_millisecond_counter == 0) {
      // Read Date/Time from RTC and update the warmup up counters

      ReadDateAndTime(&U6_DS3231, &global_data_A36507.time_now);
      mem_time_seconds_now = RTCDateToSeconds(&global_data_A36507.time_now);
      
    } // End of tasks that happen when millisecond = 0
    

    // Run once a second at 500 milliseconds
    if (can_master_millisecond_counter == 500) {

      // Update the warmup counters
      if (!_PULSE_SYNC_PFN_FAN_FAULT) {
	if (global_data_A36507.thyratron_warmup_remaining > 0) {
	  global_data_A36507.thyratron_warmup_remaining--;
	}
      } else {
	global_data_A36507.thyratron_warmup_remaining += 2;
      }
      if (global_data_A36507.thyratron_warmup_remaining >= THYRATRON_WARM_UP_TIME) {
	global_data_A36507.thyratron_warmup_remaining = THYRATRON_WARM_UP_TIME;
      }
      
      if ((board_com_ok.heater_magnet_board) && (_HEATER_MAGNET_HEATER_OK)) {
	// The Magnetron heater is on
	if (global_data_A36507.magnetron_warmup_remaining > 0) {
	  global_data_A36507.magnetron_warmup_remaining--;
	}
      } else {
	global_data_A36507.magnetron_warmup_remaining += 2;
      }
      if (global_data_A36507.magnetron_warmup_remaining >= MAGNETRON_HEATER_WARM_UP_TIME) {
	global_data_A36507.magnetron_warmup_remaining = MAGNETRON_HEATER_WARM_UP_TIME;
      }
      
      if (board_com_ok.gun_driver_board && _GUN_DRIVER_HEATER_RAMP_COMPLETE) {
	// The gun heater is on
	if (global_data_A36507.gun_warmup_remaining > 0) {
	  global_data_A36507.gun_warmup_remaining--;
	}
      } else {
	global_data_A36507.gun_warmup_remaining += 2;
      }
      if (global_data_A36507.gun_warmup_remaining >= GUN_DRIVER_HEATER_WARM_UP_TIME) {
	global_data_A36507.gun_warmup_remaining = GUN_DRIVER_HEATER_WARM_UP_TIME;
      }
      
#ifdef __IGNORE_HEATER_MAGNET_MODULE
      global_data_A36507.magnetron_warmup_remaining = 0;
#endif
      
#ifdef __IGNORE_GUN_DRIVER_MODULE
      global_data_A36507.gun_warmup_remaining = 0;
#endif

      if ((global_data_A36507.thyratron_warmup_remaining) || (global_data_A36507.magnetron_warmup_remaining) || (global_data_A36507.gun_warmup_remaining)) {
	global_data_A36507.warmup_done = 0;
      } else {
	global_data_A36507.warmup_done = 1;
      }

      // Update the system power on counters
      global_data_A36507.system_powered_seconds++;

      if (global_data_A36507.control_state == STATE_READY) {
	global_data_A36507.system_hv_on_seconds++;
      }
      
      if (global_data_A36507.control_state == STATE_XRAY_ON) {
	global_data_A36507.system_hv_on_seconds++;
	global_data_A36507.system_xray_on_seconds++;
      }
      
      // Write System timers, Arc Counter, Pulse Counter, and warmup timers to EEPROM
      global_data_A36507.last_recorded_warmup_seconds = mem_time_seconds_now;

      if (global_data_A36507.gun_warmup_remaining >= 0x3FF) {
	global_data_A36507.gun_warmup_remaining = 0x3FF;
      }

      if (global_data_A36507.magnetron_warmup_remaining >= 0x3FF) {
	global_data_A36507.magnetron_warmup_remaining = 0x3FF;
      }

      if (global_data_A36507.thyratron_warmup_remaining >= 0xFFF) {
	global_data_A36507.thyratron_warmup_remaining = 0xFFF;
      }
      
      global_data_A36507.holding_bits_for_warmup = global_data_A36507.thyratron_warmup_remaining;
      global_data_A36507.holding_bits_for_warmup <<= 10;
      global_data_A36507.holding_bits_for_warmup += global_data_A36507.magnetron_warmup_remaining;
      global_data_A36507.holding_bits_for_warmup <<= 10;
      global_data_A36507.holding_bits_for_warmup += global_data_A36507.gun_warmup_remaining;

      if (global_data_A36507.eeprom_failure == 0) {
	// Do not overwrite the values if we were unable to read them properly at boot
	ETMEEPromWritePageFast(EEPROM_PAGE_ECB_COUNTER_AND_TIMERS, ECB_COUNTER_AND_TIMERS_RAM_POINTER);
      }
    } // End of tasks that happen when millisecond = 500
    
  } // End of 10ms Tasks
}


unsigned int CalculatePulseEnergyMilliJoules(unsigned int lambda_voltage) {
  /*
    The Pulse Energy is Calculated for Each Pulse
    The Pulse Energy is then multiplied by the PRF to generate the power.
    The filament heater voltage is generated from the power.

    Power = 1/2 * C * V^2
    C = 90nF
    In Floating Point Math
    power(milli_joule) = .5 * 90e-9 * V^2 * 1000

    power_milli_joule = .5 * 90e-9 * V^2 * 1000
                      = v^2/22222.22
		      = v*v / 2^6 / 347.22
		      = v*v / 2^6 * 47 / 2^14 (.4% fixed point error)
		      



    New Method . . . 
    power_milli_joule = V^2 / 27275
    = V^2 * 4920 / 2^27


  */

  /*
  unsigned long power_milli_joule;
  unsigned int return_data;

  power_milli_joule = lambda_voltage;
  power_milli_joule *= lambda_voltage;
  power_milli_joule >>= 6;
  power_milli_joule *= 47;
  power_milli_joule >>= 14;

  if (power_milli_joule >= 0xFFFF) {
    power_milli_joule = 0xFFFF;
  }
  power_milli_joule &= 0xFFFF;

  return_data = power_milli_joule;

  return return_data;
  */

  unsigned long long power_calc;
  
  power_calc = lambda_voltage;
  power_calc *= lambda_voltage;
  power_calc *= 4920;
  power_calc >>= 27;
  return power_calc;
}


void UpdateHeaterScale() {
  unsigned long long power_calc;
  unsigned int temp16;

  // DPARKER - UPDATE THIS TO USE THE ENERGY OF THE SELECTED MODE
  // Load the energy per pulse into temp32
  // Use the higher of High/Low Energy set point

  if (etm_can_master_next_pulse_level) {
    power_calc = CalculatePulseEnergyMilliJoules(local_hvps_set_point_dose_0);
  } else {
    power_calc = CalculatePulseEnergyMilliJoules(local_hvps_set_point_dose_1);
  }

  debug_data_ecb.debug_reg[0x5] = power_calc;
  debug_data_ecb.debug_reg[0x6] = ETMCanMasterGetPulsePRF();
  
  // Multiply the Energy per Pulse times the PRF (in deci-Hz)
  power_calc *= ETMCanMasterGetPulsePRF();
  if (global_data_A36507.control_state != STATE_XRAY_ON) {
    // Set the power to zero if we are not in the X-RAY ON state
    power_calc = 0;  // DPARKER - TESTING ONLY - CALCULATE POWER WITHOUR FIRING THE SYSTEM
  }

  power_calc *= 839; 
  power_calc >>= 23; // power_calc is now Magnetron Power (in Watts)
  
  average_output_power_watts = power_calc;
  temp16 = average_output_power_watts;

  temp16 >>= 7; // Convert to index for our rolloff table
  if (temp16 >= 0x3F) {
    // Prevent Rollover of the index
    // This is a maximum magnitron power of 8064 Watts
    // If the Magnritron power is greater thatn 8064 it will rolloff as if the power was 8064 watts
    temp16 = 0x3F;
  }
  
  local_heater_current_scaled_set_point = ETMScaleFactor16(local_magnetron_heater_current_dose_all,
							   FilamentLookUpTable[temp16],
							   0);
}


void InitializeA36507(void) {
  unsigned int loop_counter;
  unsigned int eeprom_read[16];

  _FAULT_REGISTER      = 0;
  _CONTROL_REGISTER    = 0;
  _WARNING_REGISTER    = 0;
  _NOT_LOGGED_REGISTER = 0;

  global_data_A36507.access_mode = ACCESS_MODE_DEFAULT;
  global_data_A36507.service_passcode = ACCESS_MODE_SERVICE_PW_FIXED;
  global_data_A36507.etm_passcode = ACCESS_MODE_ETM_PW_FIXED;
  
  // Set the not connected bits for all boards
  *(unsigned int*)&board_com_ok = 0x0000;
    
  // Check it reset was a result of full power cycle
  _STATUS_LAST_RESET_WAS_POWER_CYCLE = 0;
  if (PIN_IN_ETM_RESET_DETECT) {
    // The power was off for more than a couple hundered milliseconds
    // All values in RAM are random.
    _STATUS_LAST_RESET_WAS_POWER_CYCLE = 1;
  }


  // Initialize all I/O Registers
  TRISA = A36507_TRISA_VALUE;
  TRISB = A36507_TRISB_VALUE;
  TRISC = A36507_TRISC_VALUE;
  TRISD = A36507_TRISD_VALUE;
  TRISF = A36507_TRISF_VALUE;
  TRISG = A36507_TRISG_VALUE;


  // Initialize the reset detect circuit
  TRIS_PIN_ETM_RESET_RETECT = 0;  // Pin is an output
  PIN_OUT_ETM_RESET_DETECT = 0;   // Pin is low so that reset detect capacitor is charged 


  ETMTickInitialize(FCY_CLK, ETM_TICK_USE_TIMER_1);
  
  // manually clock out I2C CLK to clear any connected processors that may have been stuck on a reset  
  _TRISG2 = 0; // g2 is output
  for (loop_counter = 0; loop_counter <= 100; loop_counter++) {
    _LATG2 = 0;
    __delay32(25);
    _LATG2 = 1;
    __delay32(25);
  }
  global_data_A36507.eeprom_failure = 0;
  ETMEEPromUseI2C();
  ETMEEPromConfigureI2CDevice(EEPROM_SIZE_8K_BYTES,
			      FCY_CLK,
			      ETM_I2C_400K_BAUD,
			      EEPROM_I2C_ADDRESS_0,
			      I2C_PORT_1);
  ConfigureDS3231(&U6_DS3231, I2C_PORT_1, RTC_DEFAULT_CONFIG, FCY_CLK, ETM_I2C_400K_BAUD);
  
  // Read the on timers, pulse counters, and warmup timers stored in the EEPROM
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_COUNTER_AND_TIMERS, ECB_COUNTER_AND_TIMERS_RAM_POINTER) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_COUNTER_AND_TIMERS, ECB_COUNTER_AND_TIMERS_RAM_POINTER) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_COUNTER_AND_TIMERS, ECB_COUNTER_AND_TIMERS_RAM_POINTER) == 0) {
	// We need to create an error
	_FAULT_EEPROM_FAILURE = 1;
	global_data_A36507.eeprom_failure = 1;
      }
    }
  }

  global_data_A36507.gun_warmup_remaining = (global_data_A36507.holding_bits_for_warmup & 0x03FF);
  global_data_A36507.holding_bits_for_warmup >>= 10;
  global_data_A36507.magnetron_warmup_remaining = (global_data_A36507.holding_bits_for_warmup & 0x3FF);
  global_data_A36507.holding_bits_for_warmup >>= 10;
  global_data_A36507.thyratron_warmup_remaining = (global_data_A36507.holding_bits_for_warmup & 0xFFF);

  
  ClrWdt();

  // what to do here
  ReadSystemConfigurationFromEEProm();

  
  // Read the current time
  ReadDateAndTime(&U6_DS3231, &global_data_A36507.time_now);
  mem_time_seconds_now = RTCDateToSeconds(&global_data_A36507.time_now);

  CalculateHeaterWarmupTimers();     // Calculate all of the warmup counters based on previous warmup counters

  
  // Initialize the Can module
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, &eeprom_read[0]) == 0) {
    eeprom_read[0] = 0x2121; // !!
    eeprom_read[1] = 0xFFFF;
    eeprom_read[2] = 0xFFFF;
  }

  ETMCanMasterInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_ETHERNET_BOARD, _PIN_RG13, 4);
  ETMCanMasterLoadConfiguration(36507, SOFTWARE_DASH_NUMBER, eeprom_read[0], FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV, eeprom_read[1]);
  global_data_A36507.system_serial_number = eeprom_read[2];



  // Initialize TCPmodbus Module
#if 0
  ip_config.remote_ip_addr   = ETMEEPromReadWord(EEPROM_REGISTER_REMOTE_IP_ADDRESS);
  ip_config.remote_ip_addr <<= 16;
  ip_config.remote_ip_addr  += ETMEEPromReadWord(EEPROM_REGISTER_REMOTE_IP_ADDRESS + 1);
  ip_config.ip_addr          = ETMEEPromReadWord(EEPROM_REGISTER_IP_ADDRESS);
  ip_config.ip_addr        <<= 16;
  ip_config.ip_addr         += ETMEEPromReadWord(EEPROM_REGISTER_IP_ADDRESS + 1);

  Nop();
  Nop();

  //ip_config.remote_ip_addr = 0x0F46A8C0;  // 192.168.70.15
  //ip_config.ip_addr        = 0x6346A8C0;  // 192.168.70.99


  if ((ip_config.remote_ip_addr == 0xFFFFFFFF) || (ip_config.remote_ip_addr == 0x00000000)) {
    ip_config.remote_ip_addr = DEFAULT_REMOTE_IP_ADDRESS;
  }
  if ((ip_config.ip_addr == 0xFFFFFFFF) || (ip_config.ip_addr == 0x00000000)) {
    ip_config.ip_addr = DEFAULT_IP_ADDRESS;
  }

  TCPmodbus_init(&ip_config);
#endif
  
  ETMLinacModbusInitialize();
  
  
  //Initialize the internal ADC for Startup Power Checks
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters

  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCSSL = ADCSSL_SETTING;

  _ADIF = 0;
  _ADON = 1;

  
  ETMAnalogInputInitialize(&analog_5V_vmon, 
			   MACRO_DEC_TO_SCALE_FACTOR_16(2.440215),
			   ETM_ANALOG_OFFSET_ZERO,
			   ETM_ANALOG_AVERAGE_8_SAMPLES);



  // Wait for data to be read
  while (_ADIF == 0);
  
  
  ETMAnalogInputUpdate(&analog_5V_vmon, ADCBUF0);
  ETMAnalogInputUpdate(&analog_5V_vmon, ADCBUF2);
  ETMAnalogInputUpdate(&analog_5V_vmon, ADCBUF4);
  ETMAnalogInputUpdate(&analog_5V_vmon, ADCBUF6);
  ETMAnalogInputUpdate(&analog_5V_vmon, ADCBUF8);
  ETMAnalogInputUpdate(&analog_5V_vmon, ADCBUFA);
  ETMAnalogInputUpdate(&analog_5V_vmon, ADCBUFC);
  ETMAnalogInputUpdate(&analog_5V_vmon, ADCBUFE);
  ETMAnalogInputUpdate(&analog_5V_vmon, ADCBUFE);

  //test_0 = ETMAnalogInputGetReading(&analog_5V_vmon);

  //global_data_A36507.analog_input_5v_mon.filtered_adc_reading  = ADCBUF0 + ADCBUF2 + ADCBUF4 + ADCBUF6 + ADCBUF8 + ADCBUFA + ADCBUFC + ADCBUFE;
  //global_data_A36507.analog_input_3v3_mon.filtered_adc_reading = ADCBUF1 + ADCBUF3 + ADCBUF5 + ADCBUF7 + ADCBUF9 + ADCBUFB + ADCBUFD + ADCBUFF;

  //global_data_A36507.analog_input_5v_mon.filtered_adc_reading  <<= 1;
  //global_data_A36507.analog_input_3v3_mon.filtered_adc_reading <<= 1;  



  //ETMAnalogScaleCalibrateADCReading(&global_data_A36507.analog_input_5v_mon);
  //ETMAnalogScaleCalibrateADCReading(&global_data_A36507.analog_input_3v3_mon);

  
  _ADON = 0;


  // Setup uart2 for watchdog
  
#define UART2_BAUDRATE             112000        // 113K Baud Rate
#define A36507_U2MODE_VALUE        (UART_EN & UART_IDLE_STOP & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_1STOPBIT)
#define A36507_U2STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A36507_U2BRG_VALUE         (((FCY_CLK/UART2_BAUDRATE)/16)-1)


  _U2RXIF = 0;
  _U2RXIE = 1;
  _U2RXIP = 3;

  _U2TXIF = 0;
  _U2TXIE = 1;
  _U2TXIP = 3;

  U2BRG = A36507_U2BRG_VALUE;
  U2STA = A36507_U2STA_VALUE;
  U2MODE = A36507_U2MODE_VALUE;
  U2STAbits.UTXEN = 1;

  PIN_OUT_ETM_UART_2_DE = OLL_UART_TX_DRIVER_ENABLE;
  
  BufferByte64Initialize(&uart2_input_buffer);
  BufferByte64Initialize(&uart2_output_buffer);

}
 
 
void CalculateHeaterWarmupTimers(void) {
  unsigned long difference;
  
  // Calculate new warm up time remaining
  difference = mem_time_seconds_now - global_data_A36507.last_recorded_warmup_seconds;
  if (difference >= 0x0E00) {
    difference = 0x0E00;
  }
  difference *= 2;

  global_data_A36507.thyratron_warmup_remaining += difference;
  global_data_A36507.magnetron_warmup_remaining += difference;
  global_data_A36507.gun_warmup_remaining += difference;

}


void ReadSystemConfigurationFromEEProm(void) {
  unsigned int dose_setting_data[16];

  // Read DOSE settings Zero, this is "HIGH DOSE" for MagneTX
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_0, &dose_setting_data[0]) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_0, &dose_setting_data[0]) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_0, &dose_setting_data[0]) == 0) {
	_FAULT_EEPROM_FAILURE = 1;
	global_data_A36507.eeprom_failure = 1;
      }
    }
  }

  local_hvps_set_point_dose_0                = dose_setting_data[0];
  local_magnet_current_set_point_dose_0      = dose_setting_data[1];
  local_gun_drv_top_v_dose_0                 = dose_setting_data[2];
  local_gun_drv_cathode_v_dose_0             = dose_setting_data[3];
  local_pulse_sync_afc_trig_dose_0           = dose_setting_data[5];
  local_pulse_sync_gun_trig_start_max_dose_0 = dose_setting_data[7];
  local_pulse_sync_gun_trig_stop_max_dose_0  = dose_setting_data[9];
  local_afc_home_position_dose_0             = dose_setting_data[10];


  // Read DOSE settings One, this is "LOW DOSE" for MagneTX
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_1, &dose_setting_data[0]) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_1, &dose_setting_data[0]) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_1, &dose_setting_data[0]) == 0) {
	_FAULT_EEPROM_FAILURE = 1;
	global_data_A36507.eeprom_failure = 1;
      }
    }
  }

  local_hvps_set_point_dose_1                = dose_setting_data[0];
  local_magnet_current_set_point_dose_1      = dose_setting_data[1];
  local_gun_drv_top_v_dose_1                 = dose_setting_data[2];
  local_gun_drv_cathode_v_dose_1             = dose_setting_data[3];
  local_pulse_sync_afc_trig_dose_1           = dose_setting_data[5];
  local_pulse_sync_gun_trig_start_max_dose_1 = dose_setting_data[7];
  local_pulse_sync_gun_trig_stop_max_dose_1  = dose_setting_data[9];
  local_afc_home_position_dose_1             = dose_setting_data[10];

  
  // Read DOSE settings that apply to ALL ENERGY levels
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, &dose_setting_data[0]) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, &dose_setting_data[0]) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, &dose_setting_data[0]) == 0) {
	_FAULT_EEPROM_FAILURE = 1;
	global_data_A36507.eeprom_failure = 1;
      }
    }
  }

  local_magnetron_heater_current_dose_all        = dose_setting_data[0];
  local_gun_drv_heater_v_dose_all                = dose_setting_data[1];
  local_pulse_sync_hvps_trig_start_dose_all      = dose_setting_data[2];
  local_pulse_sync_pfn_trig_dose_all             = dose_setting_data[4];
  local_pulse_sync_pulse_mon_trig_start_dose_all = dose_setting_data[5];
  local_afc_aft_control_voltage_dose_all         = dose_setting_data[9];
  
}


void FlashLeds(void) {
  switch (((global_data_A36507.startup_counter >> 4) & 0b11)) {
    
  case 0:
    PIN_OUT_ETM_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_A_RED = !OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_B_GREEN = !OLL_LED_ON;
    break;
    
  case 1:
    PIN_OUT_ETM_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_A_RED = !OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_B_GREEN = !OLL_LED_ON;
    break;
    
  case 2:
    PIN_OUT_ETM_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_A_RED = OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_B_GREEN = !OLL_LED_ON;
    break;
    
  case 3:
    PIN_OUT_ETM_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_A_RED = OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_B_GREEN = OLL_LED_ON;
    break;
  }
}


void ZeroSystemPoweredTime(void) {
  // These values will be written to EEPROM sometime in the next second.
  global_data_A36507.system_powered_seconds = 0;
  global_data_A36507.system_hv_on_seconds = 0;
  global_data_A36507.system_xray_on_seconds = 0;
}


void LoadDefaultSystemCalibrationToEEProm(void) {
  unsigned int eeprom_data[16];

  // Update dose setting Zero - High Dose data

  eeprom_data[0]  = DEFAULT_HV_LAMBDA_SET_POINT;
  eeprom_data[1]  = DEFAULT_ELECTROMAGNET_CURRENT;
  eeprom_data[2]  = DEFAULT_GUN_DRIVER_PULSE_TOP;
  eeprom_data[3]  = DEFAULT_GUN_DRIVER_CATHODE_VOLTAGE;
  eeprom_data[4]  = DEFAULT_SPARE_TRIGGER;
  eeprom_data[5]  = DEFAULT_PULSE_SYNC_AFC_SAMPLE_DELAY;
  eeprom_data[6]  = 0;
  eeprom_data[7]  = DEFAULT_GUN_START_MIN_DOSE;
  eeprom_data[8]  = DEFAULT_GUN_START_MAX_DOSE;
  eeprom_data[9]  = DEFAULT_GUN_STOP_MIN_DOSE;
  eeprom_data[10] = DEFAULT_GUN_STOP_MAX_DOSE;
  eeprom_data[11] = DEFAULT_AFC_HOME_POSITION;
  eeprom_data[12] = DEFAULT_PRF;
  eeprom_data[13] = 0;
  eeprom_data[14] = 0;
  
  global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
  
  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_0, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_0, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_0, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;
      }
    }
  }


  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_1, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_1, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_1, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;
      }
    }
  }

  
  eeprom_data[0]  = DEFAULT_MAGNETRON_HEATER_CURRENT;
  eeprom_data[1]  = DEFAULT_GUN_DRIVER_HEATER_CURRENT;
  eeprom_data[2]  = DEFAULT_HVPS_TRIGGER_START;
  eeprom_data[3]  = DEFAULT_HVPS_TRIGGER_STOP;
  eeprom_data[4]  = DEFAULT_TRIGGER_PFN;
  eeprom_data[5]  = DEFAULT_TRIGGER_MAGNETRON_AND_TARGET_CURRENT_START;
  eeprom_data[6]  = DEFAULT_TRIGGER_MAGNETRON_AND_TARGET_CURRENT_STOP;
  eeprom_data[7]  = DEFAULT_X_RAY_ON_TIME;
  eeprom_data[8]  = DEFAULT_GUN_BIAS_VOLTAGE;
  eeprom_data[9]  = DEFAULT_AFT_CONTROL_VOLTAGE;
  eeprom_data[10] = 0;
  eeprom_data[11] = 0;
  eeprom_data[12] = 0;
  eeprom_data[13] = 0;
  eeprom_data[14] = 0;


  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;
      }
    }
  }


  eeprom_data[0]  = 0;
  eeprom_data[1]  = 0;
  eeprom_data[2]  = 0;
  eeprom_data[3]  = 0;
  eeprom_data[4]  = 0;
  eeprom_data[5]  = 0;
  eeprom_data[6]  = 0;
  eeprom_data[7]  = 0;
  eeprom_data[8]  = 0;
  eeprom_data[9]  = 0;
  eeprom_data[10] = 0;
  eeprom_data[11] = 0;
  eeprom_data[12] = 0;
  eeprom_data[13] = 0;
  eeprom_data[14] = 0;


  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_COUNTER_AND_TIMERS, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_COUNTER_AND_TIMERS, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_COUNTER_AND_TIMERS, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;
      }
    }
  }

  eeprom_data[0]  = 0x21;
  eeprom_data[1]  = 0xFFFF;
  eeprom_data[2]  = 0xFFFF;
  eeprom_data[3]  = 0;
  eeprom_data[4]  = 0;
  eeprom_data[5]  = 0;
  eeprom_data[6]  = 0;
  eeprom_data[7]  = 0;
  eeprom_data[8]  = 0;
  eeprom_data[9]  = 0;
  eeprom_data[10] = 0;
  eeprom_data[11] = 0;
  eeprom_data[12] = 0;
  eeprom_data[13] = 0;
  eeprom_data[14] = 0;
  

  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;
      }
    }
  }




}





#define REGISTER_HVPS_SET_POINT_DOSE_0 0x0400
#define REGISTER_ELECTROMAGNET_CURRENT_DOSE_0 0x0401
#define REGISTER_GUN_DRIVER_PULSE_TOP_VOLTAGE_DOSE_0 0x0402
#define REGISTER_GUN_DRIVER_CATHODE_VOLTAGE_DOSE_0 0x0403
#define REGISTER_PULSE_SYNC_SPARE_TRIG_DOSE_0 0x0404
#define REGISTER_PULSE_SYNC_AFC_TRIGGER_DOSE_0 0x0405
#define REGISTER_PULSE_SYNC_GRID_START_MIN_DOSE_0 0x0406
#define REGISTER_PULSE_SYNC_GRID_START_MAX_DOSE_0 0x0407
#define REGISTER_PULSE_SYNC_GRID_STOP_MIN_DOSE_0 0x0408
#define REGISTER_PULSE_SYNC_GRID_STOP_MAX_DOSE_0 0x0409
#define REGISTER_AFC_HOME_POSITION_DOSE_0 0x40A
#define REGISTER_PULSE_SYNC_PRF_DOSE_0 0x40B

#define REGISTER_HVPS_SET_POINT_DOSE_1 0x0410
#define REGISTER_ELECTROMAGNET_CURRENT_DOSE_1 0x0411
#define REGISTER_GUN_DRIVER_PULSE_TOP_VOLTAGE_DOSE_1 0x0412
#define REGISTER_GUN_DRIVER_CATHODE_VOLTAGE_DOSE_1 0x0413
#define REGISTER_PULSE_SYNC_SPARE_TRIG_DOSE_1 0x0414
#define REGISTER_PULSE_SYNC_AFC_TRIGGER_DOSE_1 0x0415
#define REGISTER_PULSE_SYNC_GRID_START_MIN_DOSE_1 0x0416
#define REGISTER_PULSE_SYNC_GRID_START_MAX_DOSE_1 0x0417
#define REGISTER_PULSE_SYNC_GRID_STOP_MIN_DOSE_1 0x0418
#define REGISTER_PULSE_SYNC_GRID_STOP_MAX_DOSE_1 0x0419
#define REGISTER_AFC_HOME_POSITION_DOSE_1 0x41A
#define REGISTER_PULSE_SYNC_PRF_DOSE_1 0x41B

#define REGISTER_MAGNETRON_HEATER_CURRENT_DOSE_ALL 0x0500
#define REGISTER_GUN_DRIVER_HEATER_VOLTAGE_DOSE_ALL 0x0501
#define REGISTER_PULSE_SYNC_HVPS_TRIGGER_START_DOSE_ALL 0x0502
#define REGISTER_PULSE_SYNC_HVPS_TRIGGER_STOP_DOSE_ALL 0x0503
#define REGISTER_PULSE_SYNC_PFN_TRIGGER_DOSE_ALL 0x0504
#define REGISTER_PULSE_SYNC_MAGNETRON_AND_TARGET_CURRENT_TRIGGER_START_DOSE_ALL 0x0505
#define REGISTER_PULSE_SYNC_MAGNETRON_AND_TARGET_CURRENT_TRIGGER_STOP_DOSE_ALL 0x0506
#define REGISTER_X_RAY_ON_TIME_DOSE_ALL 0x0507
#define REGISTER_GUN_BIAS_VOLTAGE_DOSE_ALL 0x0508
#define REGISTER_AFC_AFT_CONTROL_VOLTAGE_DOSE_ALL 0x0509

#define REGISTER_CMD_ECB_RESET_FAULTS 0x1000
#define REGISTER_CMD_COOLANT_INTERFACE_ALLOW_25_MORE_SF6_PULSES 0x1001
#define REGISTER_SET_ACCESS_MODE_DEFAULT 0x1002
#define REGISTER_SET_ACCESS_MODE_SERVICE 0x1003
#define REGISTER_SET_ACCESS_MODE_ETM 0x1004
#define REGISTER_CLEAR_EEPROM_WRITE_STATUS 0x1005

#define REGISTER_CMD_AFC_SELECT_AFC_MODE 0x1100
#define REGISTER_CMD_AFC_SELECT_MANUAL_MODE 0x1101
#define REGISTER_CMD_AFC_MANUAL_TARGET_POSITION 0x1102
#define REGISTER_CMD_COOLANT_INTERFACE_ALLOW_SF6_PULSES_WHEN_PRESSURE_BELOW_LIMIT 0x1103
#define REGISTER_CMD_COOLANT_INTERFACE_SET_SF6_PULSES_IN_BOTTLE 0x1104
#define REGISTER_SYSTEM_SET_TIME 0x1105
#define REGISTER_SYSTEM_ENABLE_HIGH_SPEED_LOGGING 0x1106
#define REGISTER_SYSTEM_DISABLE_HIGH_SPEED_LOGGING 0x1107
#define REGISTER_SYSTEM_LOAD_FACTORY_DEFAULTS_AND_REBOOT 0x1108
#define REGISTER_SYSTEM_SAVE_CURRENT_SETTINGS_TO_CUSTOMER_SAVE 0x1109
#define REGISTER_SYSTEM_LOAD_CUSTOMER_SETTINGS_SAVE_AND_REBOOT 0x110A
#define REGISTER_REMOTE_IP_ADDRESS 0x110B
#define REGISTER_IP_ADDRESS 0x110C

#define REGISTER_DEBUG_TOGGLE_RESET_DEBUG 0x1200
#define REGISTER_DEBUG_RESET_MCU 0x1201
#define REGISTER_ETM_SYSTEM_SERIAL_NUMBER 0x1202
#define REGISTER_DEBUG_GUN_DRIVER_RESET_FPGA 0x1203
#define REGISTER_ETM_ECB_RESET_ARC_AND_PULSE_COUNT 0x1204
#define REGISTER_ETM_ECB_RESET_SECONDS_POWERED_HV_ON_XRAY_ON 0x1205
#define REGISTER_ETM_ECB_LOAD_DEFAULT_SYSTEM_SETTINGS_AND_REBOOT 0x1206
#define REGISTER_ETM_SET_REVISION_AND_SERIAL_NUMBER 0x1207
#define REGISTER_ETM_SAVE_CURRENT_SETTINGS_TO_FACTORY_DEFAULT 0x1208





void ExecuteEthernetCommand(void) {
  ETMEthernetMessageFromGUI next_message;
  unsigned long temp_long;
  RTC_TIME set_time;
  unsigned int eeprom_read[16];
  
  next_message = GetNextMessageFromGUI();
  if (next_message.index == 0xFFFF) {
    // there was no message
    return;
  }
  
  // This message needs to be processsed by the ethernet control board

  switch (next_message.index) {
    
  case REGISTER_CMD_ECB_RESET_FAULTS:
    global_data_A36507.reset_requested = 1;
    _FAULT_REGISTER = 0;
    global_data_A36507.drive_up_fault_counter = 0;
    global_data_A36507.high_voltage_on_fault_counter = 0;
    break;
    
  case REGISTER_CMD_COOLANT_INTERFACE_ALLOW_25_MORE_SF6_PULSES:
    ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_COOLING_INTERFACE_BOARD << 2)),
			ETM_CAN_REGISTER_COOLING_CMD_SF6_PULSE_LIMIT_OVERRIDE,
			0,
			0,
			0);
    break;
    
  case REGISTER_SET_ACCESS_MODE_DEFAULT:
    global_data_A36507.access_mode = ACCESS_MODE_DEFAULT;
    break;
    
  case REGISTER_SET_ACCESS_MODE_SERVICE:
    if (next_message.data_2 == global_data_A36507.service_passcode) {
      global_data_A36507.access_mode = ACCESS_MODE_SERVICE;
    }
    break;
    
  case REGISTER_SET_ACCESS_MODE_ETM:
    if (next_message.data_2 == global_data_A36507.etm_passcode) {
      global_data_A36507.access_mode = ACCESS_MODE_ETM;
    }
    break;


  case REGISTER_CLEAR_EEPROM_WRITE_STATUS:
    global_data_A36507.eeprom_write_status = EEPROM_WRITE_WAITING;
    break;
  }
  
  if (global_data_A36507.access_mode == ACCESS_MODE_ETM) {
    switch (next_message.index) {
      
    case REGISTER_ETM_SYSTEM_SERIAL_NUMBER:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_BOARD_CONFIGURATION << 4) + 2), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	global_data_A36507.system_serial_number = next_message.data_2;
      }
      break;
      
    case REGISTER_DEBUG_GUN_DRIVER_RESET_FPGA:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_GUN_DRIVER_BOARD << 2)),
			  0x8202,
			  0,
			  0,
			  0);
      break;

    case REGISTER_ETM_ECB_RESET_ARC_AND_PULSE_COUNT:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD << 2)),
			  0x2200, // DPARKER ADD THIS TO CAN CORE WITH APPROPRIATE NAME
			  0,
			  0,
			  0);
      break;

    case REGISTER_ETM_ECB_RESET_SECONDS_POWERED_HV_ON_XRAY_ON:
      ZeroSystemPoweredTime();      
      break;

    case REGISTER_ETM_ECB_LOAD_DEFAULT_SYSTEM_SETTINGS_AND_REBOOT:
      LoadDefaultSystemCalibrationToEEProm();
      __delay32(1000000);
      __asm__ ("Reset");
      break;

    case REGISTER_DEBUG_RESET_MCU:
      if ((global_data_A36507.control_state < STATE_DRIVE_UP) || (global_data_A36507.control_state > STATE_XRAY_ON)) {
	if (next_message.data_2 == ETM_CAN_ADDR_ETHERNET_BOARD) {
	  __asm__ ("Reset");
	} else {
	  SendSlaveReset(next_message.data_2);
	}
      }
      break;
      
      /*
	case REGISTER_DEBUG_TEST_PULSE_FAULT:
	ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD << 2)),
	0x22FF,
	0,
	0,
	0);
	break;
      */

    
    case REGISTER_DEBUG_TOGGLE_RESET_DEBUG:
      if (_SYNC_CONTROL_CLEAR_DEBUG_DATA) {
	_SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
      } else {
	_SYNC_CONTROL_CLEAR_DEBUG_DATA = 1;
      }
      break;


    case REGISTER_ETM_SET_REVISION_AND_SERIAL_NUMBER:
      if (next_message.data_2 == ETM_CAN_ADDR_ETHERNET_BOARD) {
	// Set the rev and S/N for the ECB
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
	if (ETMEEPromReadPage(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, &eeprom_read[0]) == 0xFFFF) {
	  eeprom_read[0] = next_message.data_1;  // Set The Rev
	  eeprom_read[1] = next_message.data_0;  // Set the SN
	  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, &eeprom_read[0]) == 0xFFFF) {
	    // The update was successful
	    global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	  }
	}
      } else {
	// Set the rev and S/N for the Slave
	if (next_message.data_2 <= 0x000F) {
	  ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (next_message.data_2 << 2)),
			      (next_message.data_2 << 12) + 0x180,
			      0,
			      next_message.data_1,
			      next_message.data_0);
	}
      }
      break;


    case REGISTER_ETM_SAVE_CURRENT_SETTINGS_TO_FACTORY_DEFAULT:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;
      while (global_data_A36507.eeprom_write_status == EEPROM_WRITE_FAILURE) {
	CopyCurrentConfig(USE_FACTORY_DEFAULTS);
      }
      break;
      
    }
  }
  
  
  if ((global_data_A36507.access_mode == ACCESS_MODE_SERVICE) || (global_data_A36507.access_mode == ACCESS_MODE_ETM)) {
    
    switch (next_message.index) {

      // -------------------- DOSE 0 SETTINGS ----------------------- //
      
    case REGISTER_HVPS_SET_POINT_DOSE_0:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 0), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_hvps_set_point_dose_0 = next_message.data_2;
      }
      break;

    case REGISTER_ELECTROMAGNET_CURRENT_DOSE_0:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 1), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_magnet_current_set_point_dose_0 = next_message.data_2;
      }
      break;
    
    case REGISTER_GUN_DRIVER_PULSE_TOP_VOLTAGE_DOSE_0:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 2), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_gun_drv_top_v_dose_0 = next_message.data_2;
      }
      break;

    case REGISTER_GUN_DRIVER_CATHODE_VOLTAGE_DOSE_0:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 3), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_gun_drv_cathode_v_dose_0 = next_message.data_2;
      }
      break;

      // NOT IMPLIMENTED REGISTER_PULSE_SYNC_SPARE_TRIG_DOSE_0

    case REGISTER_PULSE_SYNC_AFC_TRIGGER_DOSE_0:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 5), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_pulse_sync_afc_trig_dose_0 = next_message.data_2;
      }
      break;
    
    
      // NOT IMPLIMENTED REGISTER_PULSE_SYNC_GRID_START_MIN_DOSE_0
    
    case REGISTER_PULSE_SYNC_GRID_START_MAX_DOSE_0:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 7), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_pulse_sync_gun_trig_start_max_dose_0 = next_message.data_2;
      }
      break;

      // NOT IMPLIMENTED REGISTER_PULSE_SYNC_GRID_STOP_MIN_DOSE_0

    case REGISTER_PULSE_SYNC_GRID_STOP_MAX_DOSE_0:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 9), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_pulse_sync_gun_trig_stop_max_dose_0 = next_message.data_2;
      }
      break;    

    case REGISTER_AFC_HOME_POSITION_DOSE_0:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 10), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_afc_home_position_dose_0 = next_message.data_2;
      }
      break;

      // NOT IMPLIMENTED REGISTER_PULSE_SYNC_PRF_DOSE_0


      // -------------------- DOSE 1 SETTINGS ----------------------- //

      
    case REGISTER_HVPS_SET_POINT_DOSE_1:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 0), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_hvps_set_point_dose_1 = next_message.data_2;
      }
      break;

    case REGISTER_ELECTROMAGNET_CURRENT_DOSE_1:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 1), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_magnet_current_set_point_dose_1 = next_message.data_2;
      }
      break;
    
    case REGISTER_GUN_DRIVER_PULSE_TOP_VOLTAGE_DOSE_1:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 2), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_gun_drv_top_v_dose_1 = next_message.data_2;
      }
      break;

    case REGISTER_GUN_DRIVER_CATHODE_VOLTAGE_DOSE_1:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 3), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_gun_drv_cathode_v_dose_1 = next_message.data_2;
      }
      break;

      // NOT IMPLIMENTED REGISTER_PULSE_SYNC_SPARE_TRIG_DOSE_1

    case REGISTER_PULSE_SYNC_AFC_TRIGGER_DOSE_1:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 5), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_pulse_sync_afc_trig_dose_1 = next_message.data_2;
      }
      break;
    
    
      // NOT IMPLIMENTED REGISTER_PULSE_SYNC_GRID_START_MIN_DOSE_1
    
    case REGISTER_PULSE_SYNC_GRID_START_MAX_DOSE_1:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 7), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_pulse_sync_gun_trig_start_max_dose_1 = next_message.data_2;
      }
      break;

      // NOT IMPLIMENTED REGISTER_PULSE_SYNC_GRID_STOP_MIN_DOSE_1

    case REGISTER_PULSE_SYNC_GRID_STOP_MAX_DOSE_1:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 9), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_pulse_sync_gun_trig_stop_max_dose_1 = next_message.data_2;
      }
      break;    

    case REGISTER_AFC_HOME_POSITION_DOSE_1:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 10), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_afc_home_position_dose_1 = next_message.data_2;
      }
      break;

      // NOT IMPLIMENTED REGISTER_PULSE_SYNC_PRF_DOSE_1



      
      // -------------------- DOSE ALL COMMANDS ----------------------- //
    
    
    case REGISTER_MAGNETRON_HEATER_CURRENT_DOSE_ALL:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 0), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_magnetron_heater_current_dose_all = next_message.data_2;
      }
      break;

    case REGISTER_GUN_DRIVER_HEATER_VOLTAGE_DOSE_ALL:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 1), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_gun_drv_heater_v_dose_all = next_message.data_2;
      }
      break;

    case REGISTER_PULSE_SYNC_HVPS_TRIGGER_START_DOSE_ALL:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 2), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_pulse_sync_hvps_trig_start_dose_all = next_message.data_2;
      }
      break;

      // NOT IMPLIMENTED REGISTER_PULSE_SYNC_HVPS_TRIGGER_STOP_DOSE_ALL
    
    case REGISTER_PULSE_SYNC_PFN_TRIGGER_DOSE_ALL:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 4), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_pulse_sync_pfn_trig_dose_all = next_message.data_2;
      }
      break;

    case REGISTER_PULSE_SYNC_MAGNETRON_AND_TARGET_CURRENT_TRIGGER_START_DOSE_ALL:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 5), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_pulse_sync_pulse_mon_trig_start_dose_all = next_message.data_2;
      }
      break;

      // NOT IMPLIMENTED REGISTER_PULSE_SYNC_MAGNETRON_AND_TARGET_CURRENT_TRIGGER_STOP_DOSE_ALL

      // NOT IMPLIMENTED REGISTER_X_RAY_ON_TIME_DOSE_ALL

      // NOT IMPLIMENTED REGISTER_GUN_BIAS_VOLTAGE_DOSE_ALL

    case REGISTER_AFC_AFT_CONTROL_VOLTAGE_DOSE_ALL:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 9), next_message.data_2) == 0xFFFF) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	local_afc_aft_control_voltage_dose_all = next_message.data_2;
      }
      break;

    case REGISTER_REMOTE_IP_ADDRESS:
      //ETMEEPromWriteWord(next_message.index, next_message.data_2);
      //ETMEEPromWriteWord(next_message.index + 1, next_message.data_1);
      break;
      
    case REGISTER_IP_ADDRESS:
      //ETMEEPromWriteWord(next_message.index, next_message.data_2);
      //ETMEEPromWriteWord(next_message.index + 1, next_message.data_1);
      break;
      
    case REGISTER_CMD_AFC_SELECT_AFC_MODE:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_AFC_CONTROL_BOARD << 2)),
			  ETM_CAN_REGISTER_AFC_CMD_SELECT_AFC_MODE,
			  0,
			  0,
			  0);
      break;

    case REGISTER_CMD_AFC_SELECT_MANUAL_MODE:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_AFC_CONTROL_BOARD << 2)),
			  ETM_CAN_REGISTER_AFC_CMD_SELECT_MANUAL_MODE,
			  0,
			  0,
			  0);
      break;

    case REGISTER_CMD_AFC_MANUAL_TARGET_POSITION:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_AFC_CONTROL_BOARD << 2)),
			  ETM_CAN_REGISTER_AFC_CMD_SET_MANUAL_TARGET_POSITION,
			  0,
			  0,
			  next_message.data_2);
      break;

    case REGISTER_CMD_COOLANT_INTERFACE_ALLOW_SF6_PULSES_WHEN_PRESSURE_BELOW_LIMIT:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_COOLING_INTERFACE_BOARD << 2)),
			  ETM_CAN_REGISTER_COOLING_CMD_SF6_LEAK_LIMIT_OVERRIDE,
			  0,
			  0,
			  next_message.data_2);
      break;

    case REGISTER_CMD_COOLANT_INTERFACE_SET_SF6_PULSES_IN_BOTTLE:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_COOLING_INTERFACE_BOARD << 2)),
			  ETM_CAN_REGISTER_COOLING_CMD_RESET_BOTTLE_COUNT,
			  0,
			  0,
			  MAX_SF6_REFILL_PULSES_IN_BOTTLE);
      break;
      
    case REGISTER_SYSTEM_SAVE_CURRENT_SETTINGS_TO_CUSTOMER_SAVE:
      global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;
      while (global_data_A36507.eeprom_write_status == EEPROM_WRITE_FAILURE) {
	CopyCurrentConfig(USE_CUSTOMER_BACKUP);
      }
      break;
      
    case REGISTER_SYSTEM_LOAD_FACTORY_DEFAULTS_AND_REBOOT:
      if ((global_data_A36507.control_state < STATE_DRIVE_UP) || (global_data_A36507.control_state > STATE_XRAY_ON)) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;
	LoadConfig(USE_FACTORY_DEFAULTS);
	if (global_data_A36507.eeprom_write_status == EEPROM_WRITE_SUCCESSFUL) {
	  __delay32(1000000);
	  __asm__ ("Reset");
	}
      }
      break;

    case REGISTER_SYSTEM_LOAD_CUSTOMER_SETTINGS_SAVE_AND_REBOOT:
      if ((global_data_A36507.control_state < STATE_DRIVE_UP) || (global_data_A36507.control_state > STATE_XRAY_ON)) {
	global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;
	LoadConfig(USE_CUSTOMER_BACKUP);
	if (global_data_A36507.eeprom_write_status == EEPROM_WRITE_SUCCESSFUL) {
	  __delay32(1000000);
	  __asm__ ("Reset");
	}
      }
      break;

      /*
	case REGISTER_ETM_ECB_SEND_SLAVE_RELOAD_EEPROM_WITH_DEFAULTS:
	break;
       */
      
      
    case REGISTER_SYSTEM_SET_TIME:
      temp_long = next_message.data_2;
      temp_long <<= 16;
      temp_long += next_message.data_1;
      RTCSecondsToDate(temp_long, &set_time);
      SetDateAndTime(&U6_DS3231, &set_time);
      break;

    case REGISTER_SYSTEM_ENABLE_HIGH_SPEED_LOGGING:
      // Clear the Logging registers
      ETMCanMasterClearHighSpeedLogging();
      _SYNC_CONTROL_HIGH_SPEED_LOGGING = 1;
      break;
      
    case REGISTER_SYSTEM_DISABLE_HIGH_SPEED_LOGGING:
      _SYNC_CONTROL_HIGH_SPEED_LOGGING = 0;
      break;

    }
  }
}





void CopyCurrentConfig(unsigned int destination) {
  unsigned int dose_setting_0_destination_page;
  unsigned int dose_setting_1_destination_page;
  unsigned int dose_setting_all_destination_page;
  unsigned int dose_setting_data[16];
  
  global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;

  if (destination == USE_FACTORY_DEFAULTS) {
    dose_setting_0_destination_page = EEPROM_PAGE_ECB_DOSE_SETTING_0_FACTORY_DEFAULT;
    dose_setting_1_destination_page = EEPROM_PAGE_ECB_DOSE_SETTING_1_FACTORY_DEFAULT;
    dose_setting_all_destination_page = EEPROM_PAGE_ECB_DOSE_SETTING_ALL_FACTORY_DEFAULT;
  } else if (destination == USE_CUSTOMER_BACKUP) {
    dose_setting_0_destination_page = EEPROM_PAGE_ECB_DOSE_SETTING_0_CUSTOMER_BACKUP;
    dose_setting_1_destination_page = EEPROM_PAGE_ECB_DOSE_SETTING_1_CUSTOMER_BACKUP;
    dose_setting_all_destination_page = EEPROM_PAGE_ECB_DOSE_SETTING_ALL_CUSTOMER_BACKUP;
  }

  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_0, &dose_setting_data[0]) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_0, &dose_setting_data[0]) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_0, &dose_setting_data[0]) == 0) {
	// Failed to read for 3 attempts
	return;
      }
    }
  }
  
  if (ETMEEPromWritePageWithConfirmation(dose_setting_0_destination_page, &dose_setting_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(dose_setting_0_destination_page, &dose_setting_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(dose_setting_0_destination_page, &dose_setting_data[0]) == 0) {
	// Unable to write the data
	return;
      }
    }
  }

  
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_1, &dose_setting_data[0]) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_1, &dose_setting_data[0]) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_1, &dose_setting_data[0]) == 0) {
	// Failed to read for 3 attempts
	return;
      }
    }
  }
  
  if (ETMEEPromWritePageWithConfirmation(dose_setting_1_destination_page, &dose_setting_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(dose_setting_1_destination_page, &dose_setting_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(dose_setting_1_destination_page, &dose_setting_data[0]) == 0) {
	// Unable to write the data
	return;
      }
    }
  }  
  

  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, &dose_setting_data[0]) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, &dose_setting_data[0]) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, &dose_setting_data[0]) == 0) {
	// Failed to read for 3 attempts
	return;
      }
    }
  }
  
  if (ETMEEPromWritePageWithConfirmation(dose_setting_all_destination_page, &dose_setting_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(dose_setting_all_destination_page, &dose_setting_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(dose_setting_all_destination_page, &dose_setting_data[0]) == 0) {
	// Unable to write the data
	return;
      }
    }
  }  
  
  global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;

}


void LoadConfig(unsigned int source) {
  unsigned int dose_setting_0_source_page;
  unsigned int dose_setting_1_source_page;
  unsigned int dose_setting_all_source_page;
  unsigned int dose_setting_data[16];
  
  global_data_A36507.eeprom_write_status = EEPROM_WRITE_FAILURE;

  if (source == USE_FACTORY_DEFAULTS) {
    dose_setting_0_source_page = EEPROM_PAGE_ECB_DOSE_SETTING_0_FACTORY_DEFAULT;
    dose_setting_1_source_page = EEPROM_PAGE_ECB_DOSE_SETTING_1_FACTORY_DEFAULT;
    dose_setting_all_source_page = EEPROM_PAGE_ECB_DOSE_SETTING_ALL_FACTORY_DEFAULT;
  } else if (source == USE_CUSTOMER_BACKUP) {
    dose_setting_0_source_page = EEPROM_PAGE_ECB_DOSE_SETTING_0_CUSTOMER_BACKUP;
    dose_setting_1_source_page = EEPROM_PAGE_ECB_DOSE_SETTING_1_CUSTOMER_BACKUP;
    dose_setting_all_source_page = EEPROM_PAGE_ECB_DOSE_SETTING_ALL_CUSTOMER_BACKUP;
  }

  
  if (ETMEEPromReadPage(dose_setting_0_source_page, &dose_setting_data[0]) == 0) {
    if (ETMEEPromReadPage(dose_setting_0_source_page, &dose_setting_data[0]) == 0) {
      if (ETMEEPromReadPage(dose_setting_0_source_page, &dose_setting_data[0]) == 0) {
	// Failed to read for 3 attempts
	return;
      }
    }
  }
  
  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_0, &dose_setting_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_0, &dose_setting_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_0, &dose_setting_data[0]) == 0) {
	// Unable to write the data
	return;
      }
    }
  }

  if (ETMEEPromReadPage(dose_setting_1_source_page, &dose_setting_data[0]) == 0) {
    if (ETMEEPromReadPage(dose_setting_1_source_page, &dose_setting_data[0]) == 0) {
      if (ETMEEPromReadPage(dose_setting_1_source_page, &dose_setting_data[0]) == 0) {
	// Failed to read for 3 attempts
	return;
      }
    }
  }
  
  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_1, &dose_setting_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_1, &dose_setting_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_1, &dose_setting_data[0]) == 0) {
	// Unable to write the data
	return;
      }
    }
  }

  if (ETMEEPromReadPage(dose_setting_all_source_page, &dose_setting_data[0]) == 0) {
    if (ETMEEPromReadPage(dose_setting_all_source_page, &dose_setting_data[0]) == 0) {
      if (ETMEEPromReadPage(dose_setting_all_source_page, &dose_setting_data[0]) == 0) {
	// Failed to read for 3 attempts
	return;
      }
    }
  }
  
  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, &dose_setting_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, &dose_setting_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, &dose_setting_data[0]) == 0) {
	// Unable to write the data
	return;
      }
    }
  }


  global_data_A36507.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
}




void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) _U2RXInterrupt(void) {
  _U2RXIF = 0;
  while (U2STAbits.URXDA) {
    BufferByte64WriteByte(&uart2_input_buffer, U2RXREG);
  }
}


void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) _U2TXInterrupt(void) {
  _U2TXIF = 0;
  while ((!U2STAbits.UTXBF) && (BufferByte64BytesInBuffer(&uart2_output_buffer))) {
    /*
      There is at least one byte available for writing in the outputbuffer and the transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
    */
    U2TXREG = BufferByte64ReadByte(&uart2_output_buffer);
  }
}


void __attribute__((interrupt, no_auto_psv)) _OscillatorFail(void) {
  // Clearly should not get here without a major problem occuring
  
  Nop();
  Nop();
  __asm__ ("Reset");
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  
  Nop();
  Nop();
  __asm__ ("Reset");
}



void SendWatchdogResponse(unsigned int pulse_count) {
  unsigned char data_to_send[10];
  unsigned int crc_calc;
  data_to_send[0] = 0xF1;
  data_to_send[1] = 0xF2;
  data_to_send[2] = 0xF3;
  data_to_send[3] = (pulse_count >> 8);
  data_to_send[4] = pulse_count;
  data_to_send[5] = 0xF4;
  crc_calc = ETMCRC16(&data_to_send[0], 6);

  BufferByte64WriteByte(&uart2_output_buffer, data_to_send[0]);
  BufferByte64WriteByte(&uart2_output_buffer, data_to_send[1]);
  BufferByte64WriteByte(&uart2_output_buffer, data_to_send[2]);
  BufferByte64WriteByte(&uart2_output_buffer, data_to_send[3]);
  BufferByte64WriteByte(&uart2_output_buffer, data_to_send[4]);
  BufferByte64WriteByte(&uart2_output_buffer, data_to_send[5]);
  BufferByte64WriteByte(&uart2_output_buffer, (crc_calc >> 8));
  BufferByte64WriteByte(&uart2_output_buffer, (crc_calc & 0x00FF));
  
  
  if (!U2STAbits.UTXBF) {
    /*
      The transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
      All subsequent bytes will be moved from the output buffer to the transmit buffer by the U1 TX Interrupt
    */
    U2TXREG = BufferByte64ReadByte(&uart2_output_buffer);
  }
}



void TestSendWatchdogMessage(void) {
  BufferByte64WriteByte(&uart2_output_buffer, 0xF1);
  BufferByte64WriteByte(&uart2_output_buffer, 0xF2);
  BufferByte64WriteByte(&uart2_output_buffer, 0xF3);
  BufferByte64WriteByte(&uart2_output_buffer, 0x00);
  BufferByte64WriteByte(&uart2_output_buffer, 0x00);
  BufferByte64WriteByte(&uart2_output_buffer, 0xF4);
  BufferByte64WriteByte(&uart2_output_buffer, 0x1E);
  BufferByte64WriteByte(&uart2_output_buffer, 0x37);

  if (!U2STAbits.UTXBF) {
    /*
      The transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
      All subsequent bytes will be moved from the output buffer to the transmit buffer by the U1 TX Interrupt
    */
    U2TXREG = BufferByte64ReadByte(&uart2_output_buffer);
  }
}


unsigned int LookForWatchdogMessage(void) {
  unsigned int crc_received = 0;
  unsigned int crc_calc = 0;
  unsigned int message_received = 0;
  // Look for messages in the UART Buffer;
  // If multiple messages are found the old data is overwritten by the newer data
  unsigned char message[9];
  
  while (BufferByte64BytesInBuffer(&uart2_input_buffer) >= 8) {
    // Look for message
    test_uart_data_recieved++;
    message[0] = BufferByte64ReadByte(&uart2_input_buffer);
    if (message[0] != 0xF1) {
      continue;
    }
    message[1] = BufferByte64ReadByte(&uart2_input_buffer);
    if (message[1] != 0xF2) {
      continue;
    }
    message[2] = BufferByte64ReadByte(&uart2_input_buffer);
    if (message[2] != 0xF3) {
      continue;
    }
    message[3] = BufferByte64ReadByte(&uart2_input_buffer);
    message[4] = BufferByte64ReadByte(&uart2_input_buffer);


    message[5] = BufferByte64ReadByte(&uart2_input_buffer);
    if (message[5] != 0xF4) {
      continue;
    }
    message[6] = BufferByte64ReadByte(&uart2_input_buffer);
    message[7] = BufferByte64ReadByte(&uart2_input_buffer);
    
    crc_received = message[7];
    crc_received <<= 8;
    crc_received += message[6];

    crc_calc = ETMCRC16(&message[0], 6);
    test_ref_det_recieved++;
    if (crc_received == crc_calc) {
      test_ref_det_good_message++;
      // The CRC Matched
      global_data_A36507.most_recent_watchdog_reading = message[4];
      global_data_A36507.most_recent_watchdog_reading <<= 8;
      global_data_A36507.most_recent_watchdog_reading += message[3];
      message_received = 1;
      BufferByte64Initialize(&uart2_input_buffer);
    }
  }
  return message_received;
}
