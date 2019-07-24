#include "A37780.h"
#include "FIRMWARE_VERSION.h"
#include "A37780_CONFIG.h"
#include "TCPmodbus.h"
#include "ETM_ANALOG.h"
#include "ETM_TICK.h"



unsigned int mode_select_internal_trigger;  // DPARKER create structure for run time configuration parameters


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

unsigned long ten_millisecond_holding_var;

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
_FOSC(ECIO_PLL8 & CSW_FSCM_OFF);                                           // 10hz External Osc created 20Mhz FCY
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


void SetACContactor(unsigned int contactor_state);
void SetHVContactor(unsigned int contactor_state);
void SetGUNContactor(unsigned int contactor_state);
void DisableTriggers(void);
void EnableTriggers(void);
unsigned int CheckXRayOn(void);
void SetDoseLevelTiming(void);
void SetTriggerTiming(unsigned int trigger_type, unsigned int start_time, unsigned int stop_time);



void SetACContactor(unsigned int contactor_state) {}
void SetHVContactor(unsigned int contactor_state) {}
void SetGUNContactor(unsigned int contactor_state) {}
unsigned int CheckXRayOn(void) {
  return 0;
}



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


FAULTVars fault_data;


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
    SendToEventLog(LOG_ID_ENTERED_STATE_STARTUP);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    KEYLOCK_PANEL_SWITCH_EN    = OLL_KEYLOCK_PANEL_SWITCH_POWER_DISABLED;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_OPEN);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_OPEN);
    DisableTriggers();
    InitializeA36507();
    global_data_A36507.gun_heater_holdoff_timer = 0;
    _SYNC_CONTROL_GUN_DRIVER_DISABLE_HTR = 1;
    global_data_A36507.control_state = STATE_SAFETY_SELF_TEST;
    SendToEventLog(LOG_ID_ENTERED_STATE_STARTUP);
    if (_STATUS_LAST_RESET_WAS_POWER_CYCLE) {
      _SYNC_CONTROL_CLEAR_DEBUG_DATA = 1;
    }
    break;


  case STATE_SAFETY_SELF_TEST:
    SendToEventLog(LOG_ID_ENTERED_STATE_SAFETY_SELF_TEST);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    KEYLOCK_PANEL_SWITCH_EN    = OLL_KEYLOCK_PANEL_SWITCH_POWER_DISABLED;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_OPEN);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_OPEN);
    DisableTriggers();
    while (global_data_A36507.control_state == STATE_SAFETY_SELF_TEST) {
      /*
	DPARKER - What to test here
	
	Certainly want to look at the Keylock and the Panel Switch
	Do we test the E-STOP here or somewhere else?
	I think the E-STOP needs to be tested every time an AC Contactor turns on
	to verify it's state and all of the contact outputs make sense
      */

      //global_data_A36507.control_state = STATE_WAITING_FOR_POWER_ON;
      global_data_A36507.control_state = STATE_XRAY_ON;
    }
    break;
    
    
  case STATE_WAITING_FOR_POWER_ON:
    SendToEventLog(LOG_ID_ENTERED_STATE_WAITING_FOR_POWER_ON);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_OPEN);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_OPEN);
    DisableTriggers();
    while (global_data_A36507.control_state == STATE_WAITING_FOR_POWER_ON) {
      DoA36507();
      FlashLeds();
      if (DISCRETE_INPUT_SYSTEM_ENABLE == ILL_SYSTEM_ENABLE) {
	global_data_A36507.control_state = STATE_WAITING_FOR_INITIALIZATION;
      }
    }
    break;
    
    
  case STATE_WAITING_FOR_INITIALIZATION:
    SendToEventLog(LOG_ID_ENTERED_STATE_WAITING_FOR_INITIALIZATION);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
    global_data_A36507.startup_counter = 0;
    // DPARKER ADD THE FRONT PANEL LIGHT CONTROLS
    while (global_data_A36507.control_state == STATE_WAITING_FOR_INITIALIZATION) {
      DoA36507();
      FlashLeds();

      /* 
	 DPARKER
	 Add self test fucntionality
	 Certainly need to test all the fibers.
	 What else can be tested with High Voltage off????
      */
      
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
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_HIGH;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
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
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_HIGH;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
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

    
  case STATE_STANDBY:
    SendToEventLog(LOG_ID_ENTERED_STATE_STANDBY);
    _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
    while (global_data_A36507.control_state == STATE_STANDBY) {
      DoA36507();
      if (BEAM_ENABLE_INPUT == ILL_BEAM_ENABLE) {
	global_data_A36507.control_state = STATE_DRIVE_UP;
      }
      if (CheckStandbyFault()) {
	global_data_A36507.control_state = STATE_FAULT_RESET;
      }
      if (CheckFaultLatching()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
    }
    break;
    

  case STATE_DRIVE_UP:
    SendToEventLog(LOG_ID_ENTERED_STATE_DRIVE_UP);
    _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 0;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_CLOSED);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
    while (global_data_A36507.control_state == STATE_DRIVE_UP) {
      DoA36507();
      if (!CheckHVOnFault()) {
	global_data_A36507.control_state = STATE_READY;
      }
      if (BEAM_ENABLE_INPUT == ILL_BEAM_DISABLED) {
	global_data_A36507.control_state = STATE_STANDBY;
      }
      if (CheckStandbyFault()) {
	global_data_A36507.drive_up_fault_counter++;
	global_data_A36507.control_state = STATE_FAULT_RESET;
      }
      if (CheckFaultLatching()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
     }
    break;
    

  case STATE_READY:
    SendToEventLog(LOG_ID_ENTERED_STATE_READY);
    _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 0;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_CLOSED);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
    global_data_A36507.drive_up_fault_counter = 0;
    _STATUS_DRIVE_UP_TIMEOUT = 0;
     while (global_data_A36507.control_state == STATE_READY) {
      DoA36507();
      if (CheckXRayOn() == 1) {
	global_data_A36507.control_state = STATE_XRAY_ON;
      }
      if (BEAM_ENABLE_INPUT == ILL_BEAM_DISABLED) {
	global_data_A36507.control_state = STATE_DRIVE_UP;
      }
      if (CheckHVOnFault()) {
	global_data_A36507.control_state = STATE_FAULT_RESET;
	global_data_A36507.high_voltage_on_fault_counter++;
      }
      if (CheckFaultLatching()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
     }
    break;


  case STATE_XRAY_ON:
    SendToEventLog(LOG_ID_ENTERED_STATE_XRAY_ON);
    _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 0;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_ON;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_CLOSED);
    SetGUNContactor(CONTACTOR_CLOSED);
    EnableTriggers();
    SetTriggerTiming(TRIGGER_PFN_TRIGGER, 1, 500);
    SetTriggerTiming(TRIGGER_MAGNETRON_I_SAMP, 200, 400);
    SetTriggerTiming(TRIGGER_GRID_TRIGGER, 200, 400);
    SetTriggerTiming(TRIGGER_SPARE, 2, 502);
    global_data_A36507.high_voltage_on_fault_counter = 0;
    while (global_data_A36507.control_state == STATE_XRAY_ON) {
      DoA36507();
      if (CheckXRayOn() == 0) {
	//global_data_A36507.control_state = STATE_READY;
      }
      if (BEAM_ENABLE_INPUT == ILL_BEAM_DISABLED) {
	//global_data_A36507.control_state = STATE_READY;
      }
      if (CheckHVOnFault()) {
	//global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
    }
    break;


  case STATE_FAULT_SYSTEM:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_SYSTEM);
    _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_OPEN);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_OPEN);
    DisableTriggers();
    while (global_data_A36507.control_state == STATE_FAULT_SYSTEM) {
      DoA36507();
      
      if (DISCRETE_INPUT_SYSTEM_ENABLE == !ILL_SYSTEM_ENABLE) {
	global_data_A36507.control_state = STATE_SAFE_POWER_DOWN;
      }
    }
    break;
    

  case STATE_FAULT_HOLD:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_HOLD);
    _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
    global_data_A36507.reset_requested = 0;
    while (global_data_A36507.control_state == STATE_FAULT_HOLD) {
      DoA36507();
      if (global_data_A36507.reset_requested) {
	global_data_A36507.control_state = STATE_FAULT_RESET;
      }
    }
    break;
    

  case STATE_FAULT_RESET:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_RESET);
    _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
    global_data_A36507.reset_hold_timer = 0;
    while (global_data_A36507.control_state == STATE_FAULT_RESET) {
      DoA36507();
      if (global_data_A36507.reset_hold_timer > FAULT_RESET_HOLD_TIME) { 
	if (!CheckStandbyFault()) {
	  global_data_A36507.control_state = STATE_STANDBY;
	}
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
    
    
  case STATE_SAFE_POWER_DOWN:
    SendToEventLog(LOG_ID_ENTERED_STATE_SAFE_POWER_DOWN);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DisableTriggers();
    // NO Change to the Contactors yet
    // DPARKER - SHUT DOWN TCP CONNECTION
    global_data_A36507.shutdown_counter = 0;
    while (global_data_A36507.control_state == STATE_SAFE_POWER_DOWN) {
      DoA36507();
      
      if (global_data_A36507.shutdown_counter >= 100) {
	SetGUNContactor(CONTACTOR_OPEN);
	SetHVContactor(CONTACTOR_OPEN);
      }
      
      if (global_data_A36507.shutdown_counter >= 200) {
	SetACContactor(CONTACTOR_OPEN);
      }

      if (global_data_A36507.shutdown_counter >= 500) {
	__asm__ ("Reset");
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


  
}


void DoA36507(void) {
  
  etm_can_master_sync_message.sync_1_ecb_state_for_fault_logic = global_data_A36507.control_state;
  etm_can_master_sync_message.sync_2 = 0x0123;
  etm_can_master_sync_message.sync_3 = 0x4567;

  ETMCanMasterDoCan();
  //ETMLinacModbusUpdate();
  ExecuteEthernetCommand();

  if (global_data_A36507.eeprom_failure) {
    _FAULT_EEPROM_FAILURE = 1;
  }


  // Figure out if the customer has enabled XRAYs before they should have
  // If so set a fault that can only be cleared with a reset command
  if (!_PULSE_SYNC_CUSTOMER_XRAY_OFF) { 
    if ((global_data_A36507.control_state == STATE_WARMUP) ||
	(global_data_A36507.control_state == STATE_FAULT_WARMUP)) {
      // DPARKER FIX THIS LOGIC
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
    

  
  

  // Initialize all I/O Registers
  TRISA = A37780_TRISA_VALUE;
  TRISB = A37780_TRISB_VALUE;
  TRISC = A37780_TRISC_VALUE;
  TRISD = A37780_TRISD_VALUE;
  TRISF = A37780_TRISF_VALUE;
  TRISG = A37780_TRISG_VALUE;


  // Check it reset was a result of full power cycle
  /*
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
  */

  _ADON = 0;
  
  // DPARKER - MONITOR THE LENGTH OF TIME PROCESSOR WAS OFF FOR
  _STATUS_LAST_RESET_WAS_POWER_CYCLE = 0;
  // DPARKER Figure out how to set this status correctly


  
  ETMTickInitialize(FCY_CLK, ETM_TICK_USE_TIMER_1);
  
  ETMEEPromUseSPI();
  ETMEEPromConfigureSPIDevice(EEPROM_SIZE_8K_BYTES,
			      FCY_CLK,
			      SPI_CLK_10_MBIT,
			      ETM_SPI_PORT_2,
			      _PIN_RC3,
			      _PIN_NOT_CONNECTED,
			      _PIN_RC4);
  // This will also configure the I/O Expander to operate at the same bit rate
  
  // LoadDefaultSystemCalibrationToEEProm();  DPARKER need to uncomment this line to initialize boards until we have ethernet
  
  
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
  // DPARKER - Figure out how to set the time based on the off time and information from the GUI
  mem_time_seconds_now = 0;
  
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
  //DPARKER add back in method to configure IP ADDRESS
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


  // Initialize all of the output compare modules.
#define OCxCON_VALUE   0b0000000000000100 // TMR2, Single Output Pulse


  T2CON = T2CON_VALUE;
  PR2   = 0xFFFF;
  TMR2  = 0;
  
  //ETMLinacModbusInitialize();
  
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
    PIN_OUT_LED_GRN_OPERATION = !OLL_LED_ON;
    PIN_OUT_LED_RED_TEST_POINT_A = !OLL_LED_ON;
    PIN_OUT_LED_GRN_TEST_POINT_B = !OLL_LED_ON;
    break;
    
  case 1:
    PIN_OUT_LED_GRN_OPERATION = OLL_LED_ON;
    PIN_OUT_LED_RED_TEST_POINT_A = !OLL_LED_ON;
    PIN_OUT_LED_GRN_TEST_POINT_B = !OLL_LED_ON;
    break;
    
  case 2:
    PIN_OUT_LED_GRN_OPERATION = OLL_LED_ON;
    PIN_OUT_LED_RED_TEST_POINT_A = OLL_LED_ON;
    PIN_OUT_LED_GRN_TEST_POINT_B = !OLL_LED_ON;
    break;
    
  case 3:
    PIN_OUT_LED_GRN_OPERATION = OLL_LED_ON;
    PIN_OUT_LED_RED_TEST_POINT_A = OLL_LED_ON;
    PIN_OUT_LED_GRN_TEST_POINT_B = OLL_LED_ON;
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



// DPAKRER - Features to add to support Pulse Sync Functionality
/*
  Trigger Interrupt
    - Start all the output compare modules
    - Calculate the PRF
    - Calculate the MagnetronPower (this may require changes to existing code)

  Add Serial Dose Functions
    
  Set the Trigger Delays

  Look at the discrete inputs from customer

  Set discrete outputs to customer

  Manage IO Expander

  Monitor all of the internal data for faults.


 */


#define OPERATION_MODE_SINGLE_ENERGY   1



#define T2_CHARGE_TIME   42000 //2.1mS
#define T2_HOLDOFF_100US  2000


// External Trigger
void __attribute__((interrupt, shadow, no_auto_psv)) _INT1Interrupt(void) {
  unsigned int next_dose_level;

  if (mode_select_internal_trigger == 0) {
    if (_INT1EP) {
      // This was a negative going edge transistion
      // Start Charging the power supply
      // Start the power supply timer
      T2CON = T2CON_VALUE_TIMER_ON_SCALE_1_1;
      TMR2  = 0;
      PR2   = T2_CHARGE_TIME;
      _T2IF  = 0;
      _T2IE  = 1;
      _INT1IE = 0;
      PIN_HVPS_INHIBIT = !OLL_INHIBIT_HVPS;
    } else {
      // This was a positive going edge transistion
      // Start the triggers
      // Wait for 100uS
      // Enable negative trigger detection
      
      if (PIN_TRIGGER_IN == ILL_TRIGGER_ACTIVE) {
	// The Trigger Pulse is Valid
	T2CONbits.TON = 0;
	if (global_data_A36507.control_state == STATE_XRAY_ON) {
	  // Enable all of the triggers
	  OC1CON = OCxCON_VALUE;
	  OC2CON = OCxCON_VALUE;
	  // OC3CON = OCxCON_VALUE;  THE HVPS is Managed by something else
	  OC4CON = OCxCON_VALUE;
	  OC5CON = OCxCON_VALUE;
	  OC6CON = OCxCON_VALUE;
	  OC7CON = OCxCON_VALUE;
	  OC8CON = OCxCON_VALUE;
	}
	TMR2 = 0;
	T2CONbits.TON = 1;
	PR2 = T2_HOLDOFF_100US;
	_T2IE = 0;
	_T2IF = 0;
	// DPARKER - Measure the Period
	
	// Figure out the next dose level
	next_dose_level = (global_data_A36507.dose_level^0x0001);
	if (global_data_A36507.single_dual_energy_mode_selection == OPERATION_MODE_SINGLE_ENERGY) {
	  next_dose_level = global_data_A36507.dose_level;
	}
	global_data_A36507.dose_level = next_dose_level;
	SetDoseLevelTiming();
	
	while(!_T2IF) {} // Wait for Holdoff Period
	if (PIN_TRIGGER_IN != ILL_TRIGGER_ACTIVE) {
	  fault_data.trigger_width_too_short_count++;
	}
	_INT1IE = 0;
	_INT1EP = 1;  // Negative Transition
	_INT1IE = 1;
      } else {
	fault_data.trigger_not_valid_count++;
      }
    }
  } else {
    fault_data.external_trigger_when_internal_selected_count++;
  }
  _INT1IF  = 0;
}


void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
  // PFN Charge Should be completed
  // Inhibit the Lambda
  // Wait 100uS
  // Enable the Trigger
  PIN_HVPS_INHIBIT = OLL_INHIBIT_HVPS;
  TMR2 = 0;
  PR2  = T2_HOLDOFF_100US;
  _T2IF = 0;
  while (!_T2IF) {}
  
  _INT1EP  = 0; // Positive Transition Triggers
  _INT1IF  = 0; 
  _INT1IE = 1;
  _T2IE = 0;
}




void DoPostTriggerProcess(void) {
  // Set up the event Log
}

#define TRIGGER_STOP_TIME 400 // 20uS

void SetAllLevelTiming(void) {
  SetTriggerTiming(TRIGGER_PFN_TRIGGER, local_pulse_sync_pfn_trig_dose_all, TRIGGER_STOP_TIME);
  SetTriggerTiming(TRIGGER_MAGNETRON_I_SAMP, local_pulse_sync_pulse_mon_trig_start_dose_all, TRIGGER_STOP_TIME);
  SetTriggerTiming(TRIGGER_TARGET_I_SAMP, local_pulse_sync_pulse_mon_trig_start_dose_all, TRIGGER_STOP_TIME);
  SetTriggerTiming(TRIGGER_BALANCED_OUT_1, 0, TRIGGER_STOP_TIME);
}




void SetDoseLevelTiming(void) {
  
  switch (global_data_A36507.dose_level) {

  case DOSE_LEVEL_CARGO_HIGH:
    SetTriggerTiming(TRIGGER_GRID_TRIGGER, local_pulse_sync_gun_trig_start_max_dose_1, local_pulse_sync_gun_trig_stop_max_dose_1);
    SetTriggerTiming(TRIGGER_AFC_SAMPLE, local_pulse_sync_afc_trig_dose_1, TRIGGER_STOP_TIME);
    // SET SPARE TRIGGER IF NEEDED
    break;

  case DOSE_LEVEL_CARGO_LOW:
    SetTriggerTiming(TRIGGER_GRID_TRIGGER, local_pulse_sync_gun_trig_start_max_dose_0, local_pulse_sync_gun_trig_stop_max_dose_0);
    SetTriggerTiming(TRIGGER_AFC_SAMPLE, local_pulse_sync_afc_trig_dose_0, TRIGGER_STOP_TIME);
    break;

  case DOSE_LEVEL_CAB_HIGH:
    break;

  case DOSE_LEVEL_CAB_LOW:
    break;
  }
}


void SetTriggerTiming(unsigned int trigger_type, unsigned int start_time, unsigned int stop_time) {
  // DPARKER - error check start and stop time relative to each other and maximums to guarantee they happen

  switch (trigger_type) {

  case TRIGGER_GRID_TRIGGER:
    OC1R  = start_time;
    OC1RS = stop_time;
    // DPARKER - MORE WORK TO DO WITH THIS TRIGGER
    // MAY NEED TO ADD DYNAMIC DOSE HERE
    // ADD writing to the delay Lines
    break;

  case TRIGGER_PFN_TRIGGER:
    OC2R  = start_time;
    OC2RS = stop_time;
    break;

  case TRIGGER_HVPS_INHIBIT:
    OC3R  = start_time;
    OC3RS = stop_time;
    break;

  case TRIGGER_MAGNETRON_I_SAMP:
    OC4R  = start_time;
    OC4RS = stop_time;
    break;
    
  case TRIGGER_AFC_SAMPLE:
    OC5R  = start_time;
    OC5RS = stop_time;
    break;

  case TRIGGER_TARGET_I_SAMP:
    OC6R  = start_time;
    OC6RS = stop_time;
    break;

  case TRIGGER_SPARE:
    OC7R  = start_time;
    OC7RS = stop_time;
    break;

  case TRIGGER_BALANCED_OUT_1:
    OC8R  = start_time;
    OC8RS = stop_time;
    break;
  }
}







void DisableTriggers(void) {
  // DPARKER check state and fault out if this called from state X-Ray on

  OC1CON = 0;
  OC2CON = 0;
  OC3CON = 0;
  OC4CON = 0;
  OC5CON = 0;
  OC6CON = 0;
  OC7CON = 0;
  OC8CON = 0;

  PIN_GRID_TRIGGER = 0;
  PIN_PFN_TRIGGER = 0;
  PIN_HVPS_INHIBIT = 1;
  PIN_MAGNETRON_CURRENT_SAMPLE = 0;
  PIN_AFC_SAMPLE = 0;
  PIN_TARGET_CURRENT_SAMPLE = 0;
  PIN_SPARE_TRIGGER = 0;
  PIN_BALANCED_OUT_1 = 0;
}



void EnableTriggers(void) {
  _T2IE   = 0;
  _T2IF   = 0;
  _T2IP   = 5;
  _INT1EP = 0;  // Positive Transition
  _INT1IF = 0;
  _INT1IP = 6;
  _INT1IE = 1;
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



/*

unsigned int ETMEEPromPrivateReadStatusSPITest() {
  unsigned int spi_error;
  unsigned long temp;
  unsigned int return_data;

  ETMClearPin(external_eeprom_SPI.pin_chip_select_not);
  
  spi_error = 0;
  
  // FORCE 8 Bit MODE command word
  if (external_eeprom_SPI.spi_port == ETM_SPI_PORT_1) {
    SPI1CONbits.MODE16 = 0;
  } else {
    SPI2CONbits.MODE16 = 0;
  }

  // Send out the Read Status
  if (spi_error == 0) {
    temp = SendAndReceiveSPI(READ_STATUS_COMMAND_BYTE, external_eeprom_SPI.spi_port);
    if (temp == 0x11110000) {
      spi_error = 0b00000001;
    }
  }
  // Read in the status byte
  if (spi_error == 0) {
    temp = SendAndReceiveSPI(0, external_eeprom_SPI.spi_port);
    if (temp == 0x11110000) {
      spi_error = 0b00000001;
    }
  }

  ETMSetPin(external_eeprom_SPI.pin_chip_select_not);
  return_data = temp;
  
  return return_data;
}

*/
