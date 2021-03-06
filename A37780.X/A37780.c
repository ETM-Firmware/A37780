#include "A37780.h"
#include "FIRMWARE_VERSION.h"
#include "A37780_CONFIG.h"
//#include "ETM_ANALOG.h"
//#include "ETM_TICK.h"
#include "ETM_LINAC_MODBUS.h"

#define OPERATION_MODE_SINGLE_ENERGY      1
#define OPERATION_MODE_INTERLEAVE_ENERGY  2



void SendTestTriggers(void);


void SafetyRelayTestHVon(void);
void SafetyRelayTestPowerOn(void);

void UpdateDiscreteInputs(void); 
void UpdateEthernetInputs(void);
void LookForControlFaults(void);
void UpdateControlStatusBits(void);

void DoSafetyRelaySelfTest(void);


unsigned int ETMEEPromPrivateReadSinglePage(unsigned int page_number, unsigned int *page_data);
// From ETM_EEPROM - this module only needs access to this fuction





unsigned int  persistent_data_reset_check_a __attribute__ ((persistent));
unsigned int  persistent_data_reset_check_b __attribute__ ((persistent));

#define PERSISTENT_RESET_CHECK_SYSTEM_RESART      0xAEF1
#define PERSISTENT_RESET_CHECK_SYSTEM_RESART_XOR  0x510E
#define PERSISTENT_RESET_CHECK_OTHER_RESET        0x1CC7
#define PERSISTENT_RESET_CHECK_OTHER_RESET_XOR    0xE338


#define RAM_SIZE_WORDS  4096

unsigned int MCP23S18UpdateInputs(void);

TYPE_ECB_DATA ecb_data;

void GridDelayTrim(unsigned int start_trim, unsigned int stop_trim);

void MCP23S18Setup(unsigned long pin_chip_select_not,
		   unsigned char spi_port,
		   unsigned long fcy_clk,
		   unsigned long spi_bit_rate);
/*
  Setup the MCP for use
  Default Configuration to the IO Expander,
  BANK=0, All Inputs, No Inversion
*/


unsigned long mcp23S18_pin_chip_select_not;
unsigned char mcp23S18_spi_port;





unsigned int mode_select_internal_trigger;  // DPARKER create structure for run time configuration parameters


#define EEPROM_PAGE_ECB_COUNTER_AND_TIMERS           0x00
#define EEPROM_PAGE_ECB_BOARD_CONFIGURATION          0x7E
#define EEPROM_PAGE_ECB_DOSE_SETTING_0               0x40
#define EEPROM_PAGE_ECB_DOSE_SETTING_1               0x41
#define EEPROM_PAGE_ECB_DOSE_SETTING_2               0x42
#define EEPROM_PAGE_ECB_DOSE_SETTING_3               0x43
#define EEPROM_PAGE_ECB_DOSE_SETTING_ALL             0x50
#define EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS           0x51

#define EEPROM_PAGE_ECB_DOSE_COMPENSATION_A          0x7A
#define EEPROM_PAGE_ECB_DOSE_COMPENSATION_B          0x7B


#define EEPROM_PAGE_ECB_DOSE_SETTING_0_FACTORY_DEFAULT               0x60
#define EEPROM_PAGE_ECB_DOSE_SETTING_1_FACTORY_DEFAULT               0x61
#define EEPROM_PAGE_ECB_DOSE_SETTING_2_FACTORY_DEFAULT               0x62
#define EEPROM_PAGE_ECB_DOSE_SETTING_3_FACTORY_DEFAULT               0x63
#define EEPROM_PAGE_ECB_DOSE_SETTING_ALL_FACTORY_DEFAULT             0x70

#define EEPROM_PAGE_ECB_DOSE_SETTING_0_CUSTOMER_BACKUP               0x10
#define EEPROM_PAGE_ECB_DOSE_SETTING_1_CUSTOMER_BACKUP               0x11
#define EEPROM_PAGE_ECB_DOSE_SETTING_2_CUSTOMER_BACKUP               0x12
#define EEPROM_PAGE_ECB_DOSE_SETTING_3_CUSTOMER_BACKUP               0x13
#define EEPROM_PAGE_ECB_DOSE_SETTING_ALL_CUSTOMER_BACKUP             0x20



#define EEPROM_WRITE_SUCCESSFUL                      0x100
#define EEPROM_WRITE_FAILURE                         0x200
#define EEPROM_WRITE_WAITING                         0x300



#define ACCESS_MODE_DEFAULT           100
#define ACCESS_MODE_SERVICE           200
#define ACCESS_MODE_ETM               300

#define ACCESS_MODE_SERVICE_PW_FIXED  0xF1A7
#define ACCESS_MODE_ETM_PW_FIXED      0x117F



//TYPE_PUBLIC_ANALOG_INPUT analog_3_3V_vmon;
//TYPE_PUBLIC_ANALOG_INPUT analog_5V_vmon;
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
_FOSC(ECIO_PLL8 & CSW_FSCM_OFF);                                          // 10Mhz External Osc created 20Mhz FCY
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_16);                                   // 16 Second watchdog timer 
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
//unsigned int CheckConfigurationFault(void);
unsigned int CheckStandbyFault(void);
unsigned int CheckFaultLatching(void);
unsigned int CheckHVOnFault(void);
unsigned int CheckCoolingFault(void);
unsigned int CheckGunHeaterOffFault(void);


void DoA37780(void);
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





// -------------------- STARTUP Helper Functions ---------------------- //
void InitializeA37780(void);
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

void LoadDefaultSystemCalibrationToEEProm(void);
/*
  Loads the default system calibration parmaeters from flash program memmory into the EEPROM
  This is used to setup the eeprom when the machine is first set up or to return the EEPORM to a known state.
*/


// ------------------- Global Variables ------------------------------ //
A37780GlobalVars global_data_A37780;         
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
  
  ecb_data.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}



void DoStateMachine(void) {
  
  switch (ecb_data.control_state) {

  case STATE_STARTUP:
    SendToEventLog(LOG_ID_ENTERED_STATE_STARTUP);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 1);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 1);
    ETMCanMasterSyncSet(SYNC_BIT_GUN_HTR_DISABLE, 1);
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
    DISCRETE_OUTPUT_E_STOP     = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_BEAM_EN    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_KEYLOCK    = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_OPEN);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_OPEN);
    DisableTriggers();
    InitializeA37780();
    global_data_A37780.gun_heater_holdoff_timer = 0;
    SendToEventLog(LOG_ID_ENTERED_STATE_STARTUP);
    ecb_data.control_state = STATE_SAFETY_SELF_TEST;
    break;


  case STATE_SAFETY_SELF_TEST:
    SendToEventLog(LOG_ID_ENTERED_STATE_SAFETY_SELF_TEST);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 1);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 1);
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
    DISCRETE_OUTPUT_E_STOP     = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_BEAM_EN    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_KEYLOCK    = OLL_DISCRETE_OUTPUT_LOW;
    SetACContactor(CONTACTOR_OPEN);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_OPEN);
    DisableTriggers();
    ecb_data.safety_self_test = 0;
    DoSafetyRelaySelfTest();
    if (ecb_data.safety_self_test != 0) {
      // There was a relay/contactor failure
      ecb_data.control_state = STATE_SAFETY_SYSTEM_FAILURE;
    }
    ETMDigitalInitializeInput(&global_data_A37780.estop_mismatch_input, 0, 100);  //  Will time out after 1 second
    while (ecb_data.control_state == STATE_SAFETY_SELF_TEST) {
      DoA37780();
      MCP23S18UpdateInputs();
      ClrWdt();

      // Wait for an E-STOP reset to be pressed
      if ((ecb_data.io_expander_inputs.a5_estop_2_open == 0) && (ecb_data.io_expander_inputs.a4_estop_1_open == 0)) {
	// both of the E-Stop relays are closed.
	// someone must have pressed the estop reset
	ecb_data.control_state = STATE_WAITING_FOR_POWER_ON;
      }

      if (ecb_data.io_expander_inputs.a0_phase_monitor_flt == 0) {
	// power phase monitor was "good" when it should be off
	ecb_data.safety_self_test |= SAFETY_SELF_TEST_PHASE_MONITOR_OK;
      }
      
      if (ETMDigitalFilteredOutput(&global_data_A37780.estop_mismatch_input)) {
	ecb_data.safety_self_test |= SAFETY_SELF_TEST_ESTOP_MISMATCH;
      }
      
      if (ecb_data.safety_self_test) {
	ecb_data.control_state = STATE_SAFETY_SYSTEM_FAILURE;
      }
      
    }
    break;
    
    
  case STATE_WAITING_FOR_POWER_ON:
    SendToEventLog(LOG_ID_ENTERED_STATE_WAITING_FOR_POWER_ON);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 1);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 1);
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_E_STOP     = OLL_DISCRETE_OUTPUT_HIGH;
    SetACContactor(CONTACTOR_OPEN);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_OPEN);
    DisableTriggers();
    global_data_A37780.wait_for_power_on_timer = 0;
    while (ecb_data.control_state == STATE_WAITING_FOR_POWER_ON) {
      DoA37780();
      MCP23S18UpdateInputs();
      FlashLeds();
      ClrWdt();

      if (global_data_A37780.system_enable_on && global_data_A37780.wait_for_power_on_timer > 200) {
	ecb_data.control_state = STATE_WAITING_FOR_INITIALIZATION;
      }
      
      if ((ecb_data.io_expander_inputs.a0_phase_monitor_flt == 0) && (global_data_A37780.wait_for_power_on_timer > 100)) {
	// power phase monitor was "good" when it should be off
	ecb_data.safety_self_test |= SAFETY_SELF_TEST_PHASE_MONITOR_OK;
      }
            
      if (ecb_data.safety_self_test) {
	ecb_data.control_state = STATE_SAFETY_SYSTEM_FAILURE;
      }

      if (ecb_data.io_expander_inputs.a4_estop_1_open || ecb_data.io_expander_inputs.a5_estop_2_open) {
	__asm__ ("Reset");
      }
      
    }
    break;
    
    
  case STATE_WAITING_FOR_INITIALIZATION:
    SendToEventLog(LOG_ID_ENTERED_STATE_WAITING_FOR_INITIALIZATION);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE,1);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 1);
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_E_STOP     = OLL_DISCRETE_OUTPUT_HIGH;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_CLOSED);
    SendTestTriggers();
    global_data_A37780.startup_counter = 0;
    while (ecb_data.control_state == STATE_WAITING_FOR_INITIALIZATION) {
      DoA37780();
      FlashLeds();
      MCP23S18UpdateInputs();

      if ((ETMCanMasterCheckAllBoardsConfigured() == 0xFFFF) && (global_data_A37780.startup_counter >= 300)) {
      	ecb_data.control_state = STATE_WARMUP;
	SendToEventLog(LOG_ID_ALL_MODULES_CONFIGURED);
	ETMCanMasterSendSlaveClearPersistent();
      }

      if (global_data_A37780.system_enable_on == 0) {
	ecb_data.control_state = STATE_WAITING_FOR_POWER_ON;
      }
      
      if (ecb_data.io_expander_inputs.a4_estop_1_open || ecb_data.io_expander_inputs.a5_estop_2_open) {
	__asm__ ("Reset");
      }
      
    }
    break;

    
  case STATE_WARMUP:
    // Note that the warmup timers start counting in "Waiting for Initialization"
    SendToEventLog(LOG_ID_ENTERED_STATE_WARMUP);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 1);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 1);
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_HIGH;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_E_STOP     = OLL_DISCRETE_OUTPUT_HIGH;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
    SafetyRelayTestPowerOn();
    if (ecb_data.safety_self_test) {
      ecb_data.control_state = STATE_SAFETY_SYSTEM_FAILURE;
    }
    while (ecb_data.control_state == STATE_WARMUP) {
      DoA37780();
      if (global_data_A37780.warmup_done) {
	ecb_data.control_state = STATE_STANDBY;
	SendToEventLog(LOG_ID_WARMUP_DONE);
      }

      if (CheckWarmupFault()) {
	ecb_data.control_state = STATE_FAULT_WARMUP;
      }

      if (global_data_A37780.system_enable_on == 0) {
	ecb_data.control_state = STATE_WAITING_FOR_POWER_ON;
      }

      if (ecb_data.io_expander_inputs.a4_estop_1_open || ecb_data.io_expander_inputs.a5_estop_2_open) {
	__asm__ ("Reset");
      }
    }
    break;


  case STATE_FAULT_WARMUP:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_WARMUP);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE,1);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 1);
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_HIGH;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_E_STOP     = OLL_DISCRETE_OUTPUT_HIGH;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
    while (ecb_data.control_state == STATE_FAULT_WARMUP) {
      DoA37780();
      if (!CheckWarmupFault()) {
	ecb_data.control_state = STATE_WARMUP;
      }

      if (CheckWarmupFailure()) {
	ecb_data.control_state = STATE_FAULT_SYSTEM;
      }

      if (global_data_A37780.system_enable_on == 0) {
	ecb_data.control_state = STATE_WAITING_FOR_POWER_ON;
      }
      
      if (ecb_data.io_expander_inputs.a4_estop_1_open || ecb_data.io_expander_inputs.a5_estop_2_open) {
	__asm__ ("Reset");
      }
    }
    break;

    
  case STATE_STANDBY:
    SendToEventLog(LOG_ID_ENTERED_STATE_STANDBY);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 1);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 1);
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_E_STOP     = OLL_DISCRETE_OUTPUT_HIGH;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_OPEN);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
    while (ecb_data.control_state == STATE_STANDBY) {
      DoA37780();
      if (global_data_A37780.beam_enable_active) {
	ecb_data.control_state = STATE_DRIVE_UP;
      }
      if (CheckStandbyFault()) {
	ecb_data.control_state = STATE_FAULT_RESET;
      }
      if (CheckFaultLatching()) {
	ecb_data.control_state = STATE_FAULT_HOLD;
      }
      if (global_data_A37780.system_enable_on == 0) {
	ecb_data.control_state = STATE_WAITING_FOR_POWER_ON;
      }
      if (ecb_data.io_expander_inputs.a4_estop_1_open || ecb_data.io_expander_inputs.a5_estop_2_open) {
	__asm__ ("Reset");
      }
    }
    break;
    

  case STATE_DRIVE_UP:
    SendToEventLog(LOG_ID_ENTERED_STATE_DRIVE_UP);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 1);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 0);
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_OFF;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_E_STOP     = OLL_DISCRETE_OUTPUT_HIGH;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_CLOSED);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
    global_data_A37780.minimum_drive_up_timer = 0;
    while (ecb_data.control_state == STATE_DRIVE_UP) {
      DoA37780();
      if (!CheckHVOnFault() && (global_data_A37780.minimum_drive_up_timer > 300)) {
	ecb_data.control_state = STATE_READY;
      }
      if (global_data_A37780.beam_enable_active == 0) {
	ecb_data.control_state = STATE_STANDBY;
      }
      if (CheckStandbyFault()) {
	global_data_A37780.drive_up_fault_counter++;
	ecb_data.control_state = STATE_FAULT_RESET;
      }
      if (CheckFaultLatching()) {
	ecb_data.control_state = STATE_FAULT_HOLD;
      }
      if (global_data_A37780.system_enable_on == 0) {
	ecb_data.control_state = STATE_WAITING_FOR_POWER_ON;
      }
      if (ecb_data.io_expander_inputs.a4_estop_1_open || ecb_data.io_expander_inputs.a5_estop_2_open) {
	__asm__ ("Reset");
      }
     }
    break;
    

  case STATE_READY:
    SendToEventLog(LOG_ID_ENTERED_STATE_READY);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 0);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 0);
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_OFF;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_E_STOP     = OLL_DISCRETE_OUTPUT_HIGH;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_CLOSED);
    SetGUNContactor(CONTACTOR_CLOSED);
    DisableTriggers();
    global_data_A37780.drive_up_fault_counter = 0;
    ETMCanMasterStatusUpdateLoggedBit(_STATUS_DRIVE_UP_TIMEOUT, 0);
    SafetyRelayTestHVon();
    if (ecb_data.safety_self_test) {
      ecb_data.control_state = STATE_SAFETY_SYSTEM_FAILURE;
    }
    while (ecb_data.control_state == STATE_READY) {
      DoA37780();
      if (CheckXRayOn() == 1) {
	ecb_data.control_state = STATE_XRAY_ON;
      }
      if (global_data_A37780.beam_enable_active == 0) {
	ecb_data.control_state = STATE_DRIVE_UP;
      }
      if (CheckHVOnFault()) {
	ecb_data.control_state = STATE_FAULT_RESET;
	global_data_A37780.high_voltage_on_fault_counter++;
      }
      if (CheckFaultLatching()) {
	ecb_data.control_state = STATE_FAULT_HOLD;
      }
      if (global_data_A37780.system_enable_on == 0) {
	ecb_data.control_state = STATE_WAITING_FOR_POWER_ON;
      }
      if (ecb_data.io_expander_inputs.a4_estop_1_open || ecb_data.io_expander_inputs.a5_estop_2_open) {
	__asm__ ("Reset");
      }
     }
    break;


  case STATE_XRAY_ON:
    SendToEventLog(LOG_ID_ENTERED_STATE_XRAY_ON);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 0);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 0);
    FRONT_PANEL_AC_POWER       = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_BEAM_ENABLE    = OLL_FRONT_PANEL_LIGHT_ON;
    FRONT_PANEL_X_RAY_ON       = OLL_FRONT_PANEL_LIGHT_ON;
    DISCRETE_OUTPUT_FAULT      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_POWER_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_WARMUP     = OLL_DISCRETE_OUTPUT_LOW;      
    DISCRETE_OUTPUT_STANDBY    = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_READY      = OLL_DISCRETE_OUTPUT_LOW;
    DISCRETE_OUTPUT_X_RAY_ON   = OLL_DISCRETE_OUTPUT_HIGH;
    DISCRETE_OUTPUT_E_STOP     = OLL_DISCRETE_OUTPUT_HIGH;
    SetACContactor(CONTACTOR_CLOSED);
    SetHVContactor(CONTACTOR_CLOSED);
    SetGUNContactor(CONTACTOR_CLOSED);
    EnableTriggers();
    SetTriggerTiming(TRIGGER_PFN_TRIGGER, 1, 500);
    SetTriggerTiming(TRIGGER_MAGNETRON_I_SAMP, 200, 400);
    SetTriggerTiming(TRIGGER_GRID_TRIGGER, 200, 400);
    SetTriggerTiming(TRIGGER_SPARE, 2, 502);
    global_data_A37780.high_voltage_on_fault_counter = 0;
    while (ecb_data.control_state == STATE_XRAY_ON) {
      DoA37780();
      if (CheckXRayOn() == 0) {
	ecb_data.control_state = STATE_READY;
      }
      if (global_data_A37780.beam_enable_active == 0) {
	ecb_data.control_state = STATE_READY;
      }
      if (CheckHVOnFault()) {
	ecb_data.control_state = STATE_FAULT_HOLD;
      }
      if (global_data_A37780.system_enable_on == 0) {
	ecb_data.control_state = STATE_WAITING_FOR_POWER_ON;
      }
      if (ecb_data.io_expander_inputs.a4_estop_1_open || ecb_data.io_expander_inputs.a5_estop_2_open) {
	__asm__ ("Reset");
      }
    }
    break;


  case STATE_FAULT_SYSTEM:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_SYSTEM);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 0);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 1);
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
    while (ecb_data.control_state == STATE_FAULT_SYSTEM) {
      DoA37780();
      
      if (DISCRETE_INPUT_SYSTEM_ENABLE == !ILL_SYSTEM_ENABLE) {
	ecb_data.control_state = STATE_SAFE_POWER_DOWN;
      }
      if (global_data_A37780.system_enable_on == 0) {
	ecb_data.control_state = STATE_WAITING_FOR_POWER_ON;
      }
      if (ecb_data.io_expander_inputs.a4_estop_1_open || ecb_data.io_expander_inputs.a5_estop_2_open) {
	__asm__ ("Reset");
      }
    }
    break;

    //DPARKER WORK ON THIS STATE
  case STATE_SAFETY_SYSTEM_FAILURE:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_SYSTEM);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 0);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 1);
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
    KEYLOCK_PANEL_SWITCH_EN = OLL_KEYLOCK_PANEL_SWITCH_POWER_DISABLED;
    while (ecb_data.control_state == STATE_SAFETY_SYSTEM_FAILURE) {
      ClrWdt();
      DoA37780();
      // Wait for Hard Power Cycle
    }
    break;
    

  case STATE_FAULT_HOLD:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_HOLD);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 0);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 1);
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
    global_data_A37780.reset_requested = 0;
    while (ecb_data.control_state == STATE_FAULT_HOLD) {
      DoA37780();
      if (global_data_A37780.reset_requested) {
	ecb_data.control_state = STATE_FAULT_RESET;
      }
      if (global_data_A37780.system_enable_on == 0) {
	ecb_data.control_state = STATE_WAITING_FOR_POWER_ON;
      }
      if (ecb_data.io_expander_inputs.a4_estop_1_open || ecb_data.io_expander_inputs.a5_estop_2_open) {
	__asm__ ("Reset");
      }
    }
    break;
    

  case STATE_FAULT_RESET:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_RESET);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 1);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 1);
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
    global_data_A37780.reset_hold_timer = 0;
    while (ecb_data.control_state == STATE_FAULT_RESET) {
      DoA37780();
      if (global_data_A37780.reset_hold_timer > FAULT_RESET_HOLD_TIME) { 
	if (!CheckStandbyFault()) {
	  ecb_data.control_state = STATE_STANDBY;
	}
      }
      if (CheckWarmupFault()) {
	ecb_data.control_state = STATE_FAULT_WARMUP;
      }
      if ((global_data_A37780.thyratron_warmup_remaining > 0) ||
	  (global_data_A37780.magnetron_warmup_remaining > 0) ||
	  (global_data_A37780.gun_warmup_remaining > 0)) {
	ecb_data.control_state = STATE_FAULT_WARMUP;
      }
      if (global_data_A37780.system_enable_on == 0) {
	ecb_data.control_state = STATE_WAITING_FOR_POWER_ON;
      }
      if (ecb_data.io_expander_inputs.a4_estop_1_open || ecb_data.io_expander_inputs.a5_estop_2_open) {
	__asm__ ("Reset");
      }
    }
    break;
    
    
  case STATE_SAFE_POWER_DOWN:
    SendToEventLog(LOG_ID_ENTERED_STATE_SAFE_POWER_DOWN);
    ETMCanMasterSyncSet(SYNC_BIT_RESET_ENABLE, 0);
    ETMCanMasterSyncSet(SYNC_BIT_HV_DISABLE, 1);
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
    global_data_A37780.shutdown_counter = 0;
    while (ecb_data.control_state == STATE_SAFE_POWER_DOWN) {
      DoA37780();
      
      if (global_data_A37780.shutdown_counter >= 100) {
	SetGUNContactor(CONTACTOR_OPEN);
	SetHVContactor(CONTACTOR_OPEN);
      }
      
      if (global_data_A37780.shutdown_counter >= 200) {
	SetACContactor(CONTACTOR_OPEN);
	persistent_data_reset_check_a = PERSISTENT_RESET_CHECK_SYSTEM_RESART;
	persistent_data_reset_check_b = PERSISTENT_RESET_CHECK_SYSTEM_RESART_XOR;
      }

      if (global_data_A37780.shutdown_counter >= 500) {
	__asm__ ("Reset");
      }
    }
    break;

    

    
  default:
    ecb_data.control_state = STATE_FAULT_SYSTEM;
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


  if (ETMCanMasterCheckSlaveConfigured(ETM_CAN_ADDR_HEATER_MAGNET_BOARD) == 0) {
    return 1;
  }

  if (ETMCanMasterCheckSlaveConfigured(ETM_CAN_ADDR_ION_PUMP_BOARD) == 0) {
    return 1;
  }

  if (ETMCanMasterCheckSlaveConfigured(ETM_CAN_ADDR_GUN_DRIVER_BOARD) == 0) {
    return 1;
  }
  
  if (ETMCanMasterReturnSlaveStatusBit(HEATER_MAGNET_HEATER_OK_BIT, 0xFFFF) == 0) {
    // There is a problem with the magnetron heater
    return 1;
  }

  // DPARKER FIX THIS LOGIC

  
  return 0;
}



// DPARKER
unsigned int CheckFaultLatching(void) {
  if (ETMCanMasterStatusReadFaultBit(_FAULT_REPEATED_DRIVE_UP_FAULT) ||
      ETMCanMasterStatusReadFaultBit(_FAULT_REPEATED_HV_ON_FAULT)) {
    return 1;
  }
  
  if (ETMCanMasterStatusReadFaultBit(_FAULT_X_RAY_MISMATCH) ||
      ETMCanMasterStatusReadFaultBit(_FAULT_X_RAY_ON_BEAM_DISABLED) ||
      ETMCanMasterStatusReadFaultBit(_FAULT_X_RAY_ON_WRONG_STATE)) {
    return 1;
  }
    
  if (ETMCanMasterReturnSlaveStatusBit(MAGNETRON_CURRENT_FALSE_TRIGGER, 0)) {
    // If a false trigger is detected we must hold the fault
    return 1;
  }
  
  return 0;
}


unsigned int CheckStandbyFault(void) {
  if (ETMCanMasterStatusReadFaultRegister()) {
    return 1;
  }
  
  if (ETMCanMasterCheckAllBoardsConfigured() == 0) {
    return 1;
  }
  // All Boards are Connected and Configured

  if (ETMCanMasterCheckSlaveFault()) {
    // There was a fault on at least one slave
    return 1;
  }
  
  if (global_data_A37780.drive_up_timer > DRIVE_UP_TIMEOUT) {
    ETMCanMasterStatusUpdateLoggedBit(_STATUS_DRIVE_UP_TIMEOUT, 1);
    SendToEventLog(LOG_ID_DRIVE_UP_TIMEOUT);  // DPARKER MAKE LOGGING PART OF THE STATUS UPDATE FUNCTION
    return 1;
  }

  return 0;
}


unsigned int CheckCoolingFault(void) {
  // Check to see if cooling is present
  if (ETMCanMasterCheckSlaveConfigured(ETM_CAN_ADDR_COOLING_INTERFACE_BOARD) == 0) {
    return 1;
  }

  if (ETMCanMasterReturnSlaveStatusBit(COOLING_INTERFACE_FLOW_OK_BIT, 1) == 0) {
    return 1;
  }

  return 0;
}


unsigned int CheckGunHeaterOffFault(void) {
  // Check to see if there is an active over current condition in the ion pump

  if (ETMCanMasterCheckSlaveConfigured(ETM_CAN_ADDR_ION_PUMP_BOARD) == 0) {
    return 1;
  }
  if (ETMCanMasterReturnSlaveStatusBit(ION_PUMP_OVER_CURRENT_ACTIVE_BIT, 0) == 0xFFFF) {
    global_data_A37780.gun_heater_holdoff_timer = 0;
    return 1;
  }

  // DPARKER - I Don't understand these next two, what's going on???
  if (global_data_A37780.gun_heater_holdoff_timer < GUN_HEATER_HOLDOFF_AT_STARTUP) {
    return 1;
  }
  
  if (global_data_A37780.thyratron_warmup_remaining > global_data_A37780.gun_warmup_remaining) {
    if (global_data_A37780.gun_heater_holdoff_timer < (GUN_HEATER_HOLDOFF_AT_STARTUP + GUN_HEATER_ADDITONAL_HOLDOFF_COLD)) {
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
  if (ETMCanMasterStatusReadFaultRegister()) {
    return 1;
  }
  
  if (CheckStandbyFault()) {
    return 1;
  }

  if (ETMCanMasterCheckAllSlavesReady() == 0) {
    return 1;
  }

  return 0;
}


void UpdateDebugData(void) {
  /*
  debug_data_ecb.debug_reg[0x0] = global_data_A37780.high_voltage_on_fault_counter; 
  debug_data_ecb.debug_reg[0x1] = global_data_A37780.drive_up_fault_counter;
  debug_data_ecb.debug_reg[0x3] = test_ref_det_recieved; 
  debug_data_ecb.debug_reg[0x4] = test_ref_det_good_message;
  debug_data_ecb.debug_reg[0x5] = global_data_A37780.most_recent_ref_detector_reading; 
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
  
  /*
  debug_data_ecb.debug_reg[0x0] = ecb_data.dose_level_0.hvps_set_point;
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
  debug_data_ecb.debug_reg[0xF] = global_data_A37780.system_serial_number;
  */

  debug_data_ecb.ram_monitor_c = *global_data_A37780.ram_ptr_c;
  debug_data_ecb.ram_monitor_b = *global_data_A37780.ram_ptr_b;
  debug_data_ecb.ram_monitor_a = *global_data_A37780.ram_ptr_a;


  //debug_data_ecb.debug_reg[0x2] = off_time;


  debug_data_ecb.debug_reg[0x8] = (unsigned int)global_data_A37780.ram_ptr_a;
  debug_data_ecb.debug_reg[0x9] = global_data_A37780.adc_reading_at_turn_on;
  
  debug_data_ecb.debug_reg[0xA] = ecb_data.control_state;
  debug_data_ecb.debug_reg[0xB] = ecb_data.safety_self_test;
  debug_data_ecb.debug_reg[0xC] = 2;
  debug_data_ecb.debug_reg[0xD] = 2;

  debug_data_ecb.debug_reg[0xE] = ecb_data.safety_self_test;
  debug_data_ecb.debug_reg[0xF] = *(unsigned int*)&ecb_data.io_expander_inputs;

  // Update all of the discrete input mirror
  if (PIN_IN_PIC_INPUT_1) {
    ecb_data.cpu_inputs &= ~0x0001;
  } else {
    ecb_data.cpu_inputs |= 0x0001;
  }

  if (PIN_IN_PIC_INPUT_2) {
    ecb_data.cpu_inputs &= ~0x0002;
  } else {
    ecb_data.cpu_inputs |= 0x0002;
  }

  if (PIN_IN_PIC_INPUT_3) {
    ecb_data.cpu_inputs &= ~0x0004;
  } else {
    ecb_data.cpu_inputs |= 0x0004;
  }

  if (PIN_IN_PIC_INPUT_4) {
    ecb_data.cpu_inputs &= ~0x0008;
  } else {
    ecb_data.cpu_inputs |= 0x0008;
  }

  if (PIN_IN_PIC_INPUT_5) {
    ecb_data.cpu_inputs &= ~0x0010;
  } else {
    ecb_data.cpu_inputs |= 0x0010;
  }

  if (PIN_IN_PIC_INPUT_6) {
    ecb_data.cpu_inputs &= ~0x0020;
  } else {
    ecb_data.cpu_inputs |= 0x0020;
  }

  if (PIN_IN_PIC_INPUT_7) {
    ecb_data.cpu_inputs &= ~0x0040;
  } else {
    ecb_data.cpu_inputs |= 0x0040;
  }
  
  if (PIN_IN_HV_ON_CMD) {
    ecb_data.cpu_inputs &= ~0x0080;
  } else {
    ecb_data.cpu_inputs |= 0x0080;
  }

  if (PIN_IN_SPARE_READY_1) {
    ecb_data.cpu_inputs &= ~0x0100;
  } else {
    ecb_data.cpu_inputs |= 0x0100;
  }

  if (PIN_IN_TRIGGER_1) {
    ecb_data.cpu_inputs &= ~0x0200;
  } else {
    ecb_data.cpu_inputs |= 0x0200;
  }

  if (PIN_IN_TRIGGER_2) {
    ecb_data.cpu_inputs &= ~0x0400;
  } else {
    ecb_data.cpu_inputs |= 0x0400;
  }


  if (PIN_OUT_PIC_DIGITAL_OUT_1) {
    ecb_data.cpu_outputs &= ~0x0001;
  } else {
    ecb_data.cpu_outputs |= 0x0001;
  }

  if (PIN_OUT_PIC_DIGITAL_OUT_2) {
    ecb_data.cpu_outputs &= ~0x0002;
  } else {
    ecb_data.cpu_outputs |= 0x0002;
  }

  if (PIN_OUT_PIC_DIGITAL_OUT_3) {
    ecb_data.cpu_outputs &= ~0x0004;
  } else {
    ecb_data.cpu_outputs |= 0x0004;
  }

  if (PIN_OUT_PIC_DIGITAL_OUT_4) {
    ecb_data.cpu_outputs &= ~0x0008;
  } else {
    ecb_data.cpu_outputs |= 0x0008;
  }

  if (PIN_OUT_PIC_DIGITAL_OUT_5) {
    ecb_data.cpu_outputs &= ~0x0010;
  } else {
    ecb_data.cpu_outputs |= 0x0010;
  }

  if (PIN_OUT_PIC_DIGITAL_OUT_6) {
    ecb_data.cpu_outputs &= ~0x0020;
  } else {
    ecb_data.cpu_outputs |= 0x0020;
  }

  if (PIN_OUT_PIC_DIGITAL_OUT_7) {
    ecb_data.cpu_outputs &= ~0x0040;
  } else {
    ecb_data.cpu_outputs |= 0x0040;
  }

  if (PIN_OUT_PIC_DIGITAL_OUT_8) {
    ecb_data.cpu_outputs &= ~0x0080;
  } else {
    ecb_data.cpu_outputs |= 0x0080;
  }

  if (PIN_OUT_PIC_DIGITAL_OUT_9) {
    ecb_data.cpu_outputs &= ~0x0100;
  } else {
    ecb_data.cpu_outputs |= 0x0100;
  }

  // BALANCED OUT 1 IS NORMALLY CONTROLLED BY THE OUTPUT COMPARE MODULE
  
  if (PIN_OUT_BALANCED_OUT_2) {
    ecb_data.cpu_outputs &= ~0x0200;
  } else {
    ecb_data.cpu_outputs |= 0x0200;
  }

  if (PIN_OUT_UART_1_RX_SOURCE_SELECT) {
    ecb_data.cpu_outputs &= ~0x0400;
  } else {
    ecb_data.cpu_outputs |= 0x0400;
  }

  if (PIN_OUT_HV_CONTACTOR_ENABLE) {
    ecb_data.cpu_outputs &= ~0x0800;
  } else {
    ecb_data.cpu_outputs |= 0x0800;
  }

  if (PIN_OUT_AC_CONTACTOR_ENABLE) {
    ecb_data.cpu_outputs &= ~0x1000;
  } else {
    ecb_data.cpu_outputs |= 0x1000;
  }

  if (PIN_OUT_GUN_CONTACTOR_ENABLE) {
    ecb_data.cpu_outputs &= ~0x2000;
  } else {
    ecb_data.cpu_outputs |= 0x2000;
  }

  if (PIN_OUT_PIC_EXT_24V_ENABLE) {
    ecb_data.cpu_outputs &= ~0x4000;
  } else {
    ecb_data.cpu_outputs |= 0x4000;
  }

  if (PIN_OUT_PIC_LAMP_OUT_X_RAY_ON) {
    ecb_data.cpu_outputs &= ~0x8000;
  } else {
    ecb_data.cpu_outputs |= 0x8000;
  }
  
}


void DoA37780(void) {
  static unsigned long ten_millisecond_holding_var;
  static unsigned long one_second_holding_var;
  static unsigned char spi_device_alternate;
  unsigned int arc_difference;

  /* 
     DPARKER,do we need to set these???
     etm_can_master_sync_message.sync_1_ecb_state_for_fault_logic = ecb_data.control_state;
     etm_can_master_sync_message.sync_2 = 0x0123;
     etm_can_master_sync_message.sync_3 = 0x4567;
  */


  ETMCanMasterDoCan();
  ETMLinacModbusUpdate();
  ExecuteEthernetCommand();

  // Check the state of X-Ray On
  if (DISCRETE_INPUT_X_RAY_ON == ILL_X_RAY_ON_XRAY_ENABLED) {
    global_data_A37780.customer_x_ray_on = 1;
  } else {
    global_data_A37780.customer_x_ray_on = 0;
  }
  

  if (ETMTickRunOnceEveryNMilliseconds(1000, &one_second_holding_var)) {

    // Update the thyratron warmup counters
    // DPARKER TESTING
    //if (ETMCanMasterStatusReadFaultBit(_FAULT_PFN_FAN_FAULT) == 0) {
    if (ecb_data.control_state >= 0x18) {
      if (global_data_A37780.thyratron_warmup_remaining > 0) {
	global_data_A37780.thyratron_warmup_remaining--;
      }
    } else {
      global_data_A37780.thyratron_warmup_remaining += 2;
    }
    if (global_data_A37780.thyratron_warmup_remaining >= THYRATRON_WARM_UP_TIME) {
      global_data_A37780.thyratron_warmup_remaining = THYRATRON_WARM_UP_TIME;
    }

    // Update the Magnetron Heater warmup
    if (ETMCanMasterCheckSlaveConfigured(ETM_CAN_ADDR_HEATER_MAGNET_BOARD) &&
	ETMCanMasterReturnSlaveStatusBit(HEATER_MAGNET_HEATER_OK_BIT, 1)) {
      if (global_data_A37780.magnetron_warmup_remaining > 0) {
	global_data_A37780.magnetron_warmup_remaining--;
      }
    } else {
      global_data_A37780.magnetron_warmup_remaining += 2;
    }
    if (global_data_A37780.magnetron_warmup_remaining >= MAGNETRON_HEATER_WARM_UP_TIME) {
      global_data_A37780.magnetron_warmup_remaining = MAGNETRON_HEATER_WARM_UP_TIME;
    }

    // Update the Gun Heater Warmup
    if (ETMCanMasterCheckSlaveConfigured(ETM_CAN_ADDR_GUN_DRIVER_BOARD) &&
	ETMCanMasterReturnSlaveStatusBit(GUN_DRIVER_HEATER_RAMP_COMPLETE, 1)) {
      // The gun heater is on
      if (global_data_A37780.gun_warmup_remaining > 0) {
	global_data_A37780.gun_warmup_remaining--;
      }
    } else {
      global_data_A37780.gun_warmup_remaining += 2;
    }
    if (global_data_A37780.gun_warmup_remaining >= GUN_DRIVER_HEATER_WARM_UP_TIME) {
      global_data_A37780.gun_warmup_remaining = GUN_DRIVER_HEATER_WARM_UP_TIME;
    }

    // Write System timers, Arc Counter, Pulse Counter, and warmup timers to EEPROM
    //ecb_data.system_counters.last_warmup_seconds = global_data_A37780.time_seconds_now; // DPARKER testing
    ecb_data.system_counters.last_warmup_seconds = sync_time_10ms_units / 100;
    if (global_data_A37780.gun_warmup_remaining >= 0x3FF) {
      global_data_A37780.gun_warmup_remaining = 0x3FF;
    }
    if (global_data_A37780.magnetron_warmup_remaining >= 0x3FF) {
      global_data_A37780.magnetron_warmup_remaining = 0x3FF;
    }
    if (global_data_A37780.thyratron_warmup_remaining >= 0xFFF) {
      global_data_A37780.thyratron_warmup_remaining = 0xFFF;
    }

    ecb_data.system_counters.warmup_status = global_data_A37780.thyratron_warmup_remaining;
    ecb_data.system_counters.warmup_status <<= 10;
    ecb_data.system_counters.warmup_status += global_data_A37780.magnetron_warmup_remaining;
    ecb_data.system_counters.warmup_status <<= 10;
    ecb_data.system_counters.warmup_status += global_data_A37780.gun_warmup_remaining;

    global_data_A37780.current_arc_reading = local_data_mirror[ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD].log_data[10];
    if (global_data_A37780.current_arc_reading == 0) {
      global_data_A37780.previous_arc_reading = 0;
    }
    if (ecb_data.control_state <= STATE_WAITING_FOR_INITIALIZATION) {
      global_data_A37780.previous_arc_reading = global_data_A37780.current_arc_reading;
    }
    
    arc_difference = global_data_A37780.current_arc_reading - global_data_A37780.previous_arc_reading;
    if (arc_difference < 1000) {
      // Should not be possible to have more than 1000 arcs between messages, the system reall should have shut down
      // The system should have shut down long before than
      ecb_data.system_counters.arc_counter += arc_difference;
    }
    global_data_A37780.previous_arc_reading = global_data_A37780.current_arc_reading;
    
    
    // Once a second we write the timers to the EEProm OR Read the data from the external IO expander
    if (spi_device_alternate == 0) {
      spi_device_alternate = 1;
      if (global_data_A37780.eeprom_failure == 0) {
	// Do not overwrite the values if we were unable to read them properly at boot
	ETMEEPromWritePageFast(EEPROM_PAGE_ECB_COUNTER_AND_TIMERS, (unsigned int*)&ecb_data.system_counters);
      }
    } else {
      spi_device_alternate = 0;
      MCP23S18UpdateInputs();
    }
        
    // Update warmup done 
    if ((global_data_A37780.thyratron_warmup_remaining) ||
	(global_data_A37780.magnetron_warmup_remaining) ||
	(global_data_A37780.gun_warmup_remaining)) {
      global_data_A37780.warmup_done = 0;
    } else {
      global_data_A37780.warmup_done = 1;
    }

    // Update the system power counters
    if (ecb_data.control_state >= STATE_WAITING_FOR_INITIALIZATION) { 
      ecb_data.system_counters.powered_seconds++;
    }
    if (ecb_data.control_state == STATE_READY) {
      ecb_data.system_counters.hv_on_seconds++;
    }
    if (ecb_data.control_state == STATE_XRAY_ON) {
      ecb_data.system_counters.hv_on_seconds++;
      ecb_data.system_counters.xray_on_seconds++;
    }
  } // End of 1 Second Tasks

  if (ETMTickRunOnceEveryNMilliseconds(10, &ten_millisecond_holding_var)) {
    sync_time_10ms_units += 1;

    if (ecb_data.io_expander_inputs.a5_estop_2_open !=  ecb_data.io_expander_inputs.a4_estop_1_open) {
      // at least one of the E-Stop relays is closed
      // if this stays true for too long then we have a fault
      ETMDigitalUpdateInput(&global_data_A37780.estop_mismatch_input, 1);
    } else {
      ETMDigitalUpdateInput(&global_data_A37780.estop_mismatch_input, 0);
    }
    
    if (ecb_data.io_expander_inputs.a7_keylock_open) {
      DISCRETE_OUTPUT_KEYLOCK    = OLL_DISCRETE_OUTPUT_LOW;
    } else {
      DISCRETE_OUTPUT_KEYLOCK    = OLL_DISCRETE_OUTPUT_HIGH;
    }

    
    UpdateDiscreteInputs(); 
    //UpdateEthernetInputs(); // Monitor Ethernet control commands as needed
    LookForControlFaults();
    UpdateControlStatusBits();
    
    // Update the cooling fault sync bit
    if (CheckCoolingFault()) {
      ETMCanMasterSyncSet(SYNC_BIT_COOLING_FAULT, 1);
    } else {
      ETMCanMasterSyncSet(SYNC_BIT_COOLING_FAULT, 0);
    }
    
    // Update the Gun Driver Heater Enable sync bit
    // DPARKER need to update the libraries to use the Gun Heater Disable Bit
    if (CheckGunHeaterOffFault()) {
      ETMCanMasterSyncSet(SYNC_BIT_GUN_HTR_DISABLE, 1);
    } else {
      ETMCanMasterSyncSet(SYNC_BIT_GUN_HTR_DISABLE, 0);
    }

    UpdateDebugData();  // Load the customized debugging data into the debugging registers
    
    if (ecb_data.control_state == STATE_DRIVE_UP) {
      global_data_A37780.drive_up_timer++;
    } else {
      global_data_A37780.drive_up_timer = 0;
    }
    global_data_A37780.minimum_drive_up_timer++;
    global_data_A37780.startup_counter++;
    global_data_A37780.reset_hold_timer++;
    global_data_A37780.wait_for_power_on_timer++;
    
    // Update the heater current based on Output Power
    UpdateHeaterScale();

    if (global_data_A37780.gun_heater_holdoff_timer <= (GUN_HEATER_HOLDOFF_AT_STARTUP + GUN_HEATER_ADDITONAL_HOLDOFF_COLD)) {
      global_data_A37780.gun_heater_holdoff_timer++;
    }

    if (global_data_A37780.high_mode_active) {
      // The high mode bit is set.  Assume that we are interleaved mode
      global_data_A37780.single_dual_energy_mode_selection = OPERATION_MODE_INTERLEAVE_ENERGY;
    } else {
      // in single energy mode
      global_data_A37780.single_dual_energy_mode_selection = OPERATION_MODE_SINGLE_ENERGY;
    }

    if (global_data_A37780.low_mode_active) {
      // The low mode bit is set.  Assume we are in cargo mode
      // Set the cargo mode bit
      global_data_A37780.dose_level |= 0b10;
    }  else {
      // in cab mode
      // clear the cargo mode bit
      global_data_A37780.dose_level &= 0b01;
    }

  } // End of 10ms Tasks
}




void UpdateDiscreteInputs(void) {

  // ------------- UPDATE THE X_RAY_ON INPUT AND FAULTS RELATED TO IT ---------------------- //
  if (DISCRETE_INPUT_X_RAY_ON == DISCRETE_INPUT_X_RAY_OFF) {
    ETMDigitalUpdateInput(&global_data_A37780.x_ray_on_mismatch_input, 1);
  } else {
    ETMDigitalUpdateInput(&global_data_A37780.x_ray_on_mismatch_input, 0);
  }
  ETMCanMasterStatusUpdateFaultBit(_FAULT_X_RAY_MISMATCH, ETMDigitalFilteredOutput(&global_data_A37780.x_ray_on_mismatch_input));
  ETMCanMasterStatusUpdateNotLoggedBit(_STATUS_NOT_LOGGED_X_RAY_ON, DISCRETE_INPUT_X_RAY_ON);
  ETMCanMasterStatusUpdateNotLoggedBit(_STATUS_NOT_LOGGED_X_RAY_OFF, DISCRETE_INPUT_X_RAY_OFF);



  
  
  // DPARKER this need to be updated more than every 10mS
  if (DISCRETE_INPUT_X_RAY_ON == ILL_X_RAY_ON_XRAY_ENABLED) {
    global_data_A37780.customer_x_ray_on = 1;
  } else {
    global_data_A37780.customer_x_ray_on = 0;
  }

  // -------------- Update the pfn fan fault input -------------------------
  // THis should probably be filtered - DPARKER
  if (DISCRETE_INPUT_PFN_FAN_FAULT == ILL_PFN_FAN_FAULT) {
    ETMDigitalUpdateInput(&global_data_A37780.pfn_fan_fault_input, 1);
  } else {
    ETMDigitalUpdateInput(&global_data_A37780.pfn_fan_fault_input, 0);
  }
  ETMCanMasterStatusUpdateFaultBit(_FAULT_PFN_FAN_FAULT, ETMDigitalFilteredOutput(&global_data_A37780.pfn_fan_fault_input));


  // -------------- Update the Enable Input -----------------
  if (DISCRETE_INPUT_SYSTEM_ENABLE == ILL_SYSTEM_ENABLE) {
    ETMDigitalUpdateInput(&global_data_A37780.system_enable_input, 1);
  } else {
    ETMDigitalUpdateInput(&global_data_A37780.system_enable_input, 0);
  }

  if (ETMDigitalFilteredOutput(&global_data_A37780.system_enable_input)) {
    global_data_A37780.system_enable_on = 1;
  } else {
    global_data_A37780.system_enable_on = 0;
  }
  
  // -------------------- Update Low Mode Input --------------------
  if (DISCRETE_INPUT_MODE_LOW == ILL_MODE_ACTIVE) {
    global_data_A37780.low_mode_active = 1;
  } else {
    global_data_A37780.low_mode_active = 0;
  }

  // -------------------- Update High Mode Input --------------------
  if (DISCRETE_INPUT_MODE_HIGH == ILL_MODE_ACTIVE) {
    global_data_A37780.high_mode_active = 1;
  } else {
    global_data_A37780.high_mode_active = 0;
  }

  // --------------------- Update Beam Enable Input -----------------
  if (BEAM_ENABLE_INPUT == ILL_BEAM_ENABLE) {
    global_data_A37780.beam_enable_active = 1;
    DISCRETE_OUTPUT_BEAM_EN = OLL_DISCRETE_OUTPUT_HIGH;
  } else {
    global_data_A37780.beam_enable_active = 0;
    DISCRETE_OUTPUT_BEAM_EN = OLL_DISCRETE_OUTPUT_LOW;
  }

}

void LookForControlFaults(void) {

  // ------------- Look for X-Ray on Without Beam enable -----------------
  if (global_data_A37780.customer_x_ray_on && (global_data_A37780.beam_enable_active == 0)) {
    ETMDigitalUpdateInput(&global_data_A37780.x_ray_on_without_beam_enable_input, 1);
  } else {
    ETMDigitalUpdateInput(&global_data_A37780.x_ray_on_without_beam_enable_input, 0);
  }
  ETMCanMasterStatusUpdateFaultBit(_FAULT_X_RAY_ON_BEAM_DISABLED, ETMDigitalFilteredOutput(&global_data_A37780.x_ray_on_without_beam_enable_input));


  // ---------- MAKE SURE X_RAY_ON is not active when it shouldn't be -------------------
  if (global_data_A37780.customer_x_ray_on == 0) {
    // X_Rays off so this is fine
    ETMDigitalUpdateInput(&global_data_A37780.x_ray_on_wrong_state_input, 0);
  }
  if (global_data_A37780.customer_x_ray_on == 1) {
    if ((ecb_data.control_state == STATE_DRIVE_UP) ||
	(ecb_data.control_state == STATE_READY) ||
	     (ecb_data.control_state == STATE_XRAY_ON) ||
	(ecb_data.control_state == STATE_FAULT_HOLD)) {
      // This is a valid state for X-Rays to be requested
      ETMDigitalUpdateInput(&global_data_A37780.x_ray_on_wrong_state_input, 0);
    } else {
      // Not a valid state for X-Ray on
      ETMDigitalUpdateInput(&global_data_A37780.x_ray_on_wrong_state_input, 1);
    }
  }    
  ETMCanMasterStatusUpdateFaultBit(_FAULT_X_RAY_ON_WRONG_STATE, ETMDigitalFilteredOutput(&global_data_A37780.x_ray_on_wrong_state_input));

  if (global_data_A37780.drive_up_fault_counter > MAX_DRIVE_UP_FAULTS) {
    ETMCanMasterStatusUpdateFaultBit(_FAULT_REPEATED_DRIVE_UP_FAULT, 1);
  }
  
  if (global_data_A37780.high_voltage_on_fault_counter > MAX_HV_ON_FAULTS) {
    ETMCanMasterStatusUpdateFaultBit(_FAULT_REPEATED_HV_ON_FAULT, 1);
  }

  
}


void UpdateControlStatusBits(void) {
  ETMCanMasterStatusUpdateLoggedBit(_STATUS_X_RAY_ON, global_data_A37780.customer_x_ray_on);
  ETMCanMasterStatusUpdateLoggedBit(_STATUS_SYSTEM_ENABLE, global_data_A37780.system_enable_on);  
  ETMCanMasterStatusUpdateLoggedBit(_STATUS_LOW_MODE_INPUT, global_data_A37780.low_mode_active);
  ETMCanMasterStatusUpdateLoggedBit(_STATUS_HIGH_MODE_INPUT, global_data_A37780.high_mode_active);
  ETMCanMasterStatusUpdateLoggedBit(_STATUS_HV_ON_CMD, global_data_A37780.beam_enable_active);
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


void DoSafetyRelaySelfTest(void) {
  /*
	* Need to confirm that the safety relays are working
	* Confirm that 24V external out is "off"
	* Check that ESTOP-1 and ESTOP-2 Status are in the "off" state
	* Check that the AC Contactor, HVPS Contactor, Gun Contactor are in the open state
	* Enable the external 24v supply
	* Confirm that it turns on
  */

#define DELAY_100_MS 2000000
  
  __delay32(DELAY_100_MS);
  ClrWdt();
  MCP23S18UpdateInputs();

  
  ecb_data.safety_self_test = 0;


  if (ecb_data.io_expander_inputs.a1_24v_monitor_flt == 0) {
    // the ext 24V supply is on (it should not be)
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_24V_SUPPLY_OFF_FLT;
  }

  if (ecb_data.io_expander_inputs.a4_estop_1_open == 0) {
    // estop one is closed (it should not be)
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_ESTOP_1_CLOSED;
  }

  if (ecb_data.io_expander_inputs.a5_estop_2_open == 0) {
    // estop two is closed (it should not be)
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_ESTOP_2_CLOSED;
  }

  if (ecb_data.io_expander_inputs.a2_ac_contactor_open == 0) {
    // AC contactor is closed (it should not be)
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_AC_CONTACT_CLOSED;
  }

  if (ecb_data.io_expander_inputs.a3_hv_contactor_open == 0) {
    // HV contactor is closed (it should not be)
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_HV_CONTACT_CLOSED;
  }

  if (ecb_data.io_expander_inputs.b4_gun_contactor_open == 0) {
    // Gun Contactor is closed (it should not be)
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_GUN_CONTACT_CLOSED;
  }

  if (ecb_data.io_expander_inputs.a6_panel_switch_open == 0) {
    // The panel switch must be reporting open at this point
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_PANEL_SW;
  }

  if (ecb_data.io_expander_inputs.a7_keylock_open == 0) {
    // The keylock must be reporting open at this point

    ecb_data.safety_self_test |= SAFETY_SELF_TEST_KEYLOCK_SW;
  }
  
  KEYLOCK_PANEL_SWITCH_EN = OLL_KEYLOCK_PANEL_SWITCH_POWER_ENABLED;
  
  __delay32(DELAY_100_MS);
  ClrWdt();
  MCP23S18UpdateInputs();

  if (ecb_data.io_expander_inputs.a1_24v_monitor_flt) {
    // the ext 24V supply is off
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_24V_SUPPLY_ON_FLT;
  }
  
}


void SafetyRelayTestPowerOn(void) {
  // Confirm all of the relays are in the correct state for power on.
  if (ecb_data.io_expander_inputs.a1_24v_monitor_flt == 1) {
    // the ext 24V supply is off (it should be on)
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_24V_SUPPLY_ON_FLT;
  }
  
  if (ecb_data.io_expander_inputs.a4_estop_1_open == 1) {
    // estop one is open
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_ESTOP_1_OPEN;
  }
  
  if (ecb_data.io_expander_inputs.a5_estop_2_open == 1) {
    // estop two is open
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_ESTOP_2_OPEN;
  }
  
  if (ecb_data.io_expander_inputs.a2_ac_contactor_open == 1) {
    // AC contactor is open
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_AC_CONTACT_OPEN;
  }
}


void SafetyRelayTestHVon(void) {
  if (ecb_data.io_expander_inputs.a3_hv_contactor_open == 1) {
    // HV contactor is open
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_HV_CONTACT_OPEN;
  }
  
  if (ecb_data.io_expander_inputs.b4_gun_contactor_open == 1) {
    // Gun Contactor is open
    ecb_data.safety_self_test |= SAFETY_SELF_TEST_GUN_CONTACT_OPEN;
  }
}



void UpdateHeaterScale() {
  // DPAKER UPDATE THIS FUNCTION

  /*
  unsigned long long power_calc;
  unsigned int temp16;

  // DPARKER - UPDATE THIS TO USE THE ENERGY OF THE SELECTED MODE
  // Load the energy per pulse into temp32
  // Use the higher of High/Low Energy set point



  
  if (etm_can_master_next_pulse_level) {
    power_calc = CalculatePulseEnergyMilliJoules(ecb_data.dose_level_0.hvps_set_point);
  } else {
    power_calc = CalculatePulseEnergyMilliJoules(ecb_data.dose_level_1.hvps_set_point);
  }

  debug_data_ecb.debug_reg[0x5] = power_calc;
  debug_data_ecb.debug_reg[0x6] = ETMCanMasterGetPulsePRF();
  
  // Multiply the Energy per Pulse times the PRF (in deci-Hz)
  power_calc *= ETMCanMasterGetPulsePRF();
  if (ecb_data.control_state != STATE_XRAY_ON) {
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
  */

}


void InitializeA37780(void) {
  /*
    Initialization order
    * Pins
    * ADC
    * ADC Sample Off Time
    * ETM Tick
    * I/O Expader (includes SPI)
    * EEProm (SPI)
    * System Configuration (from EEProm)
    * Warmup timers (require EEProm)
    * Can Master - Reuquires the power cycle info from Warmup timers and EEProm
    * T2 - Trigger timing
    * Digital Input Objects
    * TCP Modubus

  */
  

  global_data_A37780.ram_ptr_a = 0;
  global_data_A37780.ram_ptr_b = 0;
  global_data_A37780.ram_ptr_c = 0;
  
   
  global_data_A37780.access_mode = ACCESS_MODE_DEFAULT;
  // DPARKER CHange acces mode back to default
  global_data_A37780.access_mode = ACCESS_MODE_ETM;
  
  global_data_A37780.service_passcode = ACCESS_MODE_SERVICE_PW_FIXED;
  global_data_A37780.etm_passcode = ACCESS_MODE_ETM_PW_FIXED;
  

  // Initialize all I/O Registers
  TRISA = A37780_TRISA_VALUE;
  TRISB = A37780_TRISB_VALUE;
  TRISC = A37780_TRISC_VALUE;
  TRISD = A37780_TRISD_VALUE;
  TRISF = A37780_TRISF_VALUE;
  TRISG = A37780_TRISG_VALUE;

  // DPARKER Turn on UART1 TX fiber as a test light source
  _LATF3 = 1;
  _TRISF3 = 0;
  
  
  // ---- Check the ADC to figure out how long the power was off for ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters
  
  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCSSL = ADCSSL_SETTING;
  
  _ADIF = 0;
  _ADON = 1;

  while (!_ADIF) {
  }

  global_data_A37780.adc_reading_at_turn_on = ADCBUF0 + ADCBUF1 + ADCBUF2 + ADCBUF3 + ADCBUF4 + ADCBUF5 + ADCBUF6 + ADCBUF7 +
    ADCBUF8 + ADCBUF9 + ADCBUFA + ADCBUFB + ADCBUFC + ADCBUFD + ADCBUFE + ADCBUFF;
  
  _ADON = 0;    


  
  ETMTickInitialize(FCY_CLK, ETM_TICK_USE_TIMER_1);


  MCP23S18Setup(_PIN_RC2,
		ETM_SPI_PORT_2,
		FCY_CLK,
		SPI_CLK_1_MBIT);

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
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_COUNTER_AND_TIMERS, (unsigned int*)&ecb_data.system_counters) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_COUNTER_AND_TIMERS, (unsigned int*)&ecb_data.system_counters) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_COUNTER_AND_TIMERS, (unsigned int*)&ecb_data.system_counters) == 0) {
	// We need to create an error
	ETMCanMasterStatusUpdateFaultBit(_FAULT_EEPROM_FAILURE, 1);
	global_data_A37780.eeprom_failure |= 0b0000000010000000;
      }
    }
  }

  global_data_A37780.gun_warmup_remaining = (ecb_data.system_counters.warmup_status & 0x03FF);
  ecb_data.system_counters.warmup_status >>= 10;
  global_data_A37780.magnetron_warmup_remaining = (ecb_data.system_counters.warmup_status & 0x3FF);
  ecb_data.system_counters.warmup_status >>= 10;
  global_data_A37780.thyratron_warmup_remaining = (ecb_data.system_counters.warmup_status & 0xFFF);
  
  ClrWdt();

  // what to do here
  ReadSystemConfigurationFromEEProm();

  
   // DPARKER Rework this, we can't set values in the can module until it is initialize
  /*
  _STATUS_LAST_RESET_WAS_POWER_CYCLE = 1;
  
  if ((persistent_data_reset_check_a == PERSISTENT_RESET_CHECK_SYSTEM_RESART) &
      (persistent_data_reset_check_b == PERSISTENT_RESET_CHECK_SYSTEM_RESART_XOR)) {
    _STATUS_LAST_RESET_WAS_POWER_CYCLE = 0;
  }
  
  if ((persistent_data_reset_check_a == PERSISTENT_RESET_CHECK_OTHER_RESET) &
      (persistent_data_reset_check_b == PERSISTENT_RESET_CHECK_OTHER_RESET_XOR)) {
    _STATUS_LAST_RESET_WAS_POWER_CYCLE = 0;
  }
  */    

  CalculateHeaterWarmupTimers();     // Calculate all of the warmup counters based on previous warmup counters


  persistent_data_reset_check_a = 0;
  persistent_data_reset_check_b = 0;
  
  if (global_data_A37780.eeprom_failure == 0) {
    // Do not overwrite the values if we were unable to read them properly at boot
    if (ETMEEPromWritePageFast(EEPROM_PAGE_ECB_COUNTER_AND_TIMERS, (unsigned int*)&ecb_data.system_counters)) {
      persistent_data_reset_check_a = PERSISTENT_RESET_CHECK_OTHER_RESET;
      persistent_data_reset_check_b = PERSISTENT_RESET_CHECK_OTHER_RESET_XOR;
    }
  }
#define BOARDS_TO_IGNORE 0b1111111111111111
  
  ETMCanMasterInitialize(CAN_PORT_1, FCY_CLK, _PIN_RC15, 4, BOARDS_TO_IGNORE, SYSTEM_CONFIGURATION_A37854_000);

  // DPARKER - FIGURE OUT THE CONFIGURATION
  //ETMCanMasterLoadConfiguration(37780, SOFTWARE_DASH_NUMBER, eeprom_read[0], FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV, eeprom_read[1]);
  //global_data_A37780.system_serial_number = eeprom_read[2];



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


  T2CON = T2CON_VALUE;
  PR2   = 0xFFFF;
  TMR2  = 0;


#define X_RAY_ON_BEAM_DISABLED_MAX_TIME 10 // 100mS
#define X_RAY_ON_OFF_MISMATCH_MAX_TIME  10 // 100mS
#define X_RAY_ON_WRONG_STATE_MAX_TIME   10 // 100mS
#define PFN_FAN_FAULT_MAX_TIME          50 // 500mS 


  ETMDigitalInitializeInput(&global_data_A37780.x_ray_on_mismatch_input, 0, X_RAY_ON_OFF_MISMATCH_MAX_TIME);
  ETMDigitalInitializeInput(&global_data_A37780.x_ray_on_without_beam_enable_input, 0, X_RAY_ON_BEAM_DISABLED_MAX_TIME); 
  ETMDigitalInitializeInput(&global_data_A37780.x_ray_on_wrong_state_input, 0, X_RAY_ON_WRONG_STATE_MAX_TIME);
  ETMDigitalInitializeInput(&global_data_A37780.pfn_fan_fault_input, 0, PFN_FAN_FAULT_MAX_TIME);

  ETMLinacModbusInitialize();
}
 

#define ADC_READING_OFF_TIME_10_SEC   53656
#define ADC_READING_OFF_TIME_15_SEC   48550
#define ADC_READING_OFF_TIME_20_SEC   43930
#define ADC_READING_OFF_TIME_25_SEC   39749
#define ADC_READING_OFF_TIME_30_SEC   35966
#define ADC_READING_OFF_TIME_40_SEC   29447
#define ADC_READING_OFF_TIME_50_SEC   24109
#define ADC_READING_OFF_TIME_60_SEC   19739
#define ADC_READING_OFF_TIME_75_SEC   16955
#define ADC_READING_OFF_TIME_90_SEC   14594
#define ADC_READING_OFF_TIME_105_SEC  12561
#define ADC_READING_OFF_TIME_120_SEC  10811
#define ADC_READING_OFF_TIME_135_SEC  9305
#define ADC_READING_OFF_TIME_150_SEC  8009
#define ADC_READING_OFF_TIME_165_SEC  6893
#define ADC_READING_OFF_TIME_180_SEC  5933


void CalculateHeaterWarmupTimers(void) {
  unsigned int off_time;

  // Calculate the off time
  if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_10_SEC) {
    off_time = 10;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_15_SEC) {
    off_time = 15;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_20_SEC) {
    off_time = 20;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_25_SEC) {
    off_time = 25;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_30_SEC) {
    off_time = 30;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_40_SEC) {
    off_time = 40;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_50_SEC) {
    off_time = 50;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_60_SEC) {
    off_time = 60;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_75_SEC) {
    off_time = 75;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_90_SEC) {
    off_time = 90;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_105_SEC) {
    off_time = 105;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_120_SEC) {
    off_time = 120;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_135_SEC) {
    off_time = 135;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_150_SEC) {
    off_time = 150;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_165_SEC) {
    off_time = 165;
  } else if (global_data_A37780.adc_reading_at_turn_on > ADC_READING_OFF_TIME_180_SEC) {
    off_time = 180;
  } else {
    off_time = 900;
  }

  debug_data_ecb.debug_reg[0x2] = off_time;
  
  global_data_A37780.thyratron_warmup_remaining += off_time*2;
  global_data_A37780.magnetron_warmup_remaining += off_time*2;
  global_data_A37780.gun_warmup_remaining += off_time*2;

}


void ReadSystemConfigurationFromEEProm(void) {
  unsigned int data_restore;
  
  // Read DOSE settings Zero, this is "HIGH DOSE" for MagneTX
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_0, (unsigned int*)&ecb_data.dose_level_0) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_0, (unsigned int*)&ecb_data.dose_level_0) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_0, (unsigned int*)&ecb_data.dose_level_0) == 0) {
	ETMCanMasterStatusUpdateFaultBit(_FAULT_EEPROM_FAILURE, 1);
	global_data_A37780.eeprom_failure |= 0b0000000000000001;
      }
    }
  }
 
  // Read DOSE settings One, this is "LOW DOSE" for MagneTX
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_1, (unsigned int*)&ecb_data.dose_level_1) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_1, (unsigned int*)&ecb_data.dose_level_1) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_1, (unsigned int*)&ecb_data.dose_level_1) == 0) {
	ETMCanMasterStatusUpdateFaultBit(_FAULT_EEPROM_FAILURE, 1);
	global_data_A37780.eeprom_failure |= 0b0000000000000010;
      }
    }
  }

  // Read DOSE settings two, this is "HIGH DOSE" for MagneTX
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_2, (unsigned int*)&ecb_data.dose_level_2) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_2, (unsigned int*)&ecb_data.dose_level_2) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_2, (unsigned int*)&ecb_data.dose_level_2) == 0) {
	ETMCanMasterStatusUpdateFaultBit(_FAULT_EEPROM_FAILURE, 1);
	global_data_A37780.eeprom_failure |= 0b0000000000000100;
      }
    }
  }
 
  // Read DOSE settings three, this is "LOW DOSE" for MagneTX
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_3, (unsigned int*)&ecb_data.dose_level_3) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_3, (unsigned int*)&ecb_data.dose_level_3) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_3, (unsigned int*)&ecb_data.dose_level_3) == 0) {
	ETMCanMasterStatusUpdateFaultBit(_FAULT_EEPROM_FAILURE, 1);
	global_data_A37780.eeprom_failure |= 0b0000000000001000;
      }
    }
  }
  
  // Read DOSE settings that apply to ALL ENERGY levels
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, (unsigned int*)&ecb_data.dose_level_all) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, (unsigned int*)&ecb_data.dose_level_all) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_SETTING_ALL, (unsigned int*)&ecb_data.dose_level_all) == 0) {
	ETMCanMasterStatusUpdateFaultBit(_FAULT_EEPROM_FAILURE, 1);
	global_data_A37780.eeprom_failure |= 0b0000000000010000;
      }
    }
  }

  // Read DOSE compensation Settings Group A
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_COMPENSATION_A, (unsigned int*)&ecb_data.dose_compensation_group_a) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_COMPENSATION_A, (unsigned int*)&ecb_data.dose_compensation_group_a) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_COMPENSATION_A, (unsigned int*)&ecb_data.dose_compensation_group_a) == 0) {
	ETMCanMasterStatusUpdateFaultBit(_FAULT_EEPROM_FAILURE, 1);
	global_data_A37780.eeprom_failure |= 0b0000000000100000;
      }
    }
  }

  // Read DOSE compensation Settings Group B
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_COMPENSATION_B, (unsigned int*)&ecb_data.dose_compensation_group_b) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_COMPENSATION_B, (unsigned int*)&ecb_data.dose_compensation_group_b) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_DOSE_COMPENSATION_B, (unsigned int*)&ecb_data.dose_compensation_group_b) == 0) {
	ETMCanMasterStatusUpdateFaultBit(_FAULT_EEPROM_FAILURE, 1);
	global_data_A37780.eeprom_failure |= 0b0000000001000000;
      }
    }
  }

  data_restore = ecb_data.control_state;
  
  // Read the Config data 
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, (unsigned int*)&ecb_data.config) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, (unsigned int*)&ecb_data.config) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, (unsigned int*)&ecb_data.config) == 0) {
	ETMCanMasterStatusUpdateFaultBit(_FAULT_EEPROM_FAILURE, 1);
	global_data_A37780.eeprom_failure |= 0b0000000010000000;
      }
    }
  }


  // Read the Config data 
  if (ETMEEPromReadPage(EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS, (unsigned int*)&ecb_data.aux_set_points) == 0) {
    if (ETMEEPromReadPage(EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS, (unsigned int*)&ecb_data.aux_set_points) == 0) {
      if (ETMEEPromReadPage(EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS, (unsigned int*)&ecb_data.aux_set_points) == 0) {
	ETMCanMasterStatusUpdateFaultBit(_FAULT_EEPROM_FAILURE, 1);
	global_data_A37780.eeprom_failure |= 0b0000000100000000;
      }
    }
  }
  

  
  // DPARKER fix this
  ecb_data.config.firmware_agile_rev = 1;
  ecb_data.config.firmware_branch = 2;
  ecb_data.config.firmware_branch_rev = 3;
  ecb_data.control_state = data_restore;
}


void FlashLeds(void) {
  switch (((global_data_A37780.startup_counter >> 4) & 0b11)) {
    
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


void LoadDefaultSystemCalibrationToEEProm(void) {
  unsigned int eeprom_data[16];

  // Update dose setting Zero - High Dose data

  eeprom_data[0]  = DEFAULT_HV_LAMBDA_SET_POINT;
  eeprom_data[1]  = DEFAULT_ELECTROMAGNET_CURRENT;
  eeprom_data[2]  = DEFAULT_GUN_DRIVER_PULSE_TOP;
  eeprom_data[3]  = DEFAULT_GUN_DRIVER_CATHODE_VOLTAGE;
  eeprom_data[4]  = DEFAULT_SPARE_TRIGGER;
  eeprom_data[5]  = DEFAULT_PULSE_SYNC_AFC_SAMPLE_DELAY;
  eeprom_data[6]  = DEFAULT_GUN_START_MIN_DOSE;
  eeprom_data[7]  = DEFAULT_GUN_START_MAX_DOSE;
  eeprom_data[8]  = DEFAULT_GUN_STOP_MIN_DOSE;
  eeprom_data[9]  = DEFAULT_GUN_STOP_MAX_DOSE;
  eeprom_data[10] = DEFAULT_AFC_HOME_POSITION;
  eeprom_data[11] = DEFAULT_PRF;
  eeprom_data[12] = 0;
  eeprom_data[13] = 0;
  eeprom_data[14] = 0;
  
  global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
  
  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_0, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_0, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_0, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
      }
    }
  }


  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_1, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_1, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_1, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
      }
    }
  }

  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_2, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_2, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_2, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
      }
    }
  }


  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_3, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_3, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_SETTING_3, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
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
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
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
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
      }
    }
  }

  eeprom_data[0]  = 0x0;
  eeprom_data[1]  = 37780;
  eeprom_data[2]  = 0;
  eeprom_data[3]  = 0x2041;
  eeprom_data[4]  = 0;
  eeprom_data[5]  = 100;
  eeprom_data[6]  = 0;
  eeprom_data[7]  = 0;
  eeprom_data[8]  = 0;
  eeprom_data[9]  = 0x2048;
  eeprom_data[10]  = 0;
  eeprom_data[11] = 7000;
  eeprom_data[12] = 0;
  eeprom_data[13] = 0x4450;
  eeprom_data[14] = 0;
  eeprom_data[15] = 0;
  

  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
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

  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_COMPENSATION_A, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_COMPENSATION_A, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_COMPENSATION_A, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
      }
    }
  }
  
  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_COMPENSATION_B, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_COMPENSATION_B, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_DOSE_COMPENSATION_B, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
      }
    }
  }

  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS, &eeprom_data[0]) == 0) {
    if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS, &eeprom_data[0]) == 0) {
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS, &eeprom_data[0]) == 0) {
	// Unable to write the data
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
      }
    }
  }
  
}



#define REGISTER_HVPS_SET_POINT_DOSE_0 0x0400
#define REGISTER_ELECTROMAGNET_CURRENT_DOSE_0 0x0401
#define REGISTER_GUN_DRIVER_PULSE_TOP_VOLTAGE_DOSE_0 0x0402
#define REGISTER_GUN_DRIVER_CATHODE_VOLTAGE_DOSE_0 0x0403
#define REGISTER_TRIGGER_SPARE_DOSE_0 0x0404
#define REGISTER_TRIGGER_AFC_DOSE_0 0x0405
#define REGISTER_TRIGGER_GRID_START_MIN_DOSE_0 0x0406
#define REGISTER_TRIGGER_GRID_START_MAX_DOSE_0 0x0407
#define REGISTER_TRIGGER_GRID_STOP_MIN_DOSE_0 0x0408
#define REGISTER_TRIGGER_GRID_STOP_MAX_DOSE_0 0x0409
#define REGISTER_AFC_HOME_POSITION_DOSE_0 0x40A
#define REGISTER_SELF_TRIGGER_PRF_DOSE_0 0x40B

#define REGISTER_HVPS_SET_POINT_DOSE_1 0x0410
#define REGISTER_ELECTROMAGNET_CURRENT_DOSE_1 0x0411
#define REGISTER_GUN_DRIVER_PULSE_TOP_VOLTAGE_DOSE_1 0x0412
#define REGISTER_GUN_DRIVER_CATHODE_VOLTAGE_DOSE_1 0x0413
#define REGISTER_TRIGGER_SPARE_DOSE_1 0x0414
#define REGISTER_TRIGGER_AFC_DOSE_1 0x0415
#define REGISTER_TRIGGER_GRID_START_MIN_DOSE_1 0x0416
#define REGISTER_TRIGGER_GRID_START_MAX_DOSE_1 0x0417
#define REGISTER_TRIGGER_GRID_STOP_MIN_DOSE_1 0x0418
#define REGISTER_TRIGGER_GRID_STOP_MAX_DOSE_1 0x0419
#define REGISTER_AFC_HOME_POSITION_DOSE_1 0x41A
#define REGISTER_SELF_TRIGGER_PRF_DOSE_1 0x41B

#define REGISTER_HVPS_SET_POINT_DOSE_2 0x0420
#define REGISTER_ELECTROMAGNET_CURRENT_DOSE_2 0x0421
#define REGISTER_GUN_DRIVER_PULSE_TOP_VOLTAGE_DOSE_2 0x0422
#define REGISTER_GUN_DRIVER_CATHODE_VOLTAGE_DOSE_2 0x0423
#define REGISTER_TRIGGER_SPARE_DOSE_2 0x0424
#define REGISTER_TRIGGER_AFC_DOSE_2 0x0425
#define REGISTER_TRIGGER_GRID_START_MIN_DOSE_2 0x0426
#define REGISTER_TRIGGER_GRID_START_MAX_DOSE_2 0x0427
#define REGISTER_TRIGGER_GRID_STOP_MIN_DOSE_2 0x0428
#define REGISTER_TRIGGER_GRID_STOP_MAX_DOSE_2 0x0429
#define REGISTER_AFC_HOME_POSITION_DOSE_2 0x42A
#define REGISTER_SELF_TRIGGER_PRF_DOSE_2 0x42B

#define REGISTER_HVPS_SET_POINT_DOSE_3 0x0430
#define REGISTER_ELECTROMAGNET_CURRENT_DOSE_3 0x0431
#define REGISTER_GUN_DRIVER_PULSE_TOP_VOLTAGE_DOSE_3 0x0432
#define REGISTER_GUN_DRIVER_CATHODE_VOLTAGE_DOSE_3 0x0433
#define REGISTER_TRIGGER_SPARE_DOSE_3 0x0434
#define REGISTER_TRIGGER_AFC_DOSE_3 0x0435
#define REGISTER_TRIGGER_GRID_START_MIN_DOSE_3 0x0436
#define REGISTER_TRIGGER_GRID_START_MAX_DOSE_3 0x0437
#define REGISTER_TRIGGER_GRID_STOP_MIN_DOSE_3 0x0438
#define REGISTER_TRIGGER_GRID_STOP_MAX_DOSE_3 0x0439
#define REGISTER_AFC_HOME_POSITION_DOSE_3 0x43A
#define REGISTER_SELF_TRIGGER_PRF_DOSE_3 0x43B

#define REGISTER_MAGNETRON_HEATER_CURRENT_DOSE_ALL 0x0500
#define REGISTER_GUN_DRIVER_HEATER_VOLTAGE_DOSE_ALL 0x0501
#define REGISTER_TRIGGER_HVPS_START_DOSE_ALL 0x0502
#define REGISTER_TRIGGER_HVPS_STOP_DOSE_ALL 0x0503
#define REGISTER_TRIGGER_PFN_DOSE_ALL 0x0504
#define REGISTER_TRIGGER_MAGNETRON_AND_TARGET_CURRENT_START_DOSE_ALL 0x0505
#define REGISTER_TRIGGER_MAGNETRON_AND_TARGET_CURRENT_STOP_DOSE_ALL 0x0506
#define REGISTER_X_RAY_ON_TIME_DOSE_ALL 0x0507
#define REGISTER_GUN_BIAS_VOLTAGE_DOSE_ALL 0x0508
#define REGISTER_AFC_AFT_CONTROL_VOLTAGE_DOSE_ALL 0x0509
#define REGISTER_AFC_LOCKED_TO_HOME_DOSE_ALL 0x50A

#define REGISTER_AUX_SETTING_0 0x0510
#define REGISTER_AUX_SETTING_1 0x0511
#define REGISTER_AUX_SETTING_2 0x0512
#define REGISTER_AUX_SETTING_3 0x0513
#define REGISTER_AUX_SETTING_4 0x0514
#define REGISTER_AUX_SETTING_5 0x0515
#define REGISTER_AUX_SETTING_6 0x0516
#define REGISTER_AUX_SETTING_7 0x0517

#define REGISTER_DOSE_COMP_0 0x0520
#define REGISTER_DOSE_COMP_1 0x0521
#define REGISTER_DOSE_COMP_2 0x0522
#define REGISTER_DOSE_COMP_3 0x0523
#define REGISTER_DOSE_COMP_4 0x0524
#define REGISTER_DOSE_COMP_5 0x0525
#define REGISTER_DOSE_COMP_6 0x0526
#define REGISTER_DOSE_COMP_7 0x0527
#define REGISTER_DOSE_COMP_8 0x0528
#define REGISTER_DOSE_COMP_9 0x0529
#define REGISTER_DOSE_COMP_10 0x52A
#define REGISTER_DOSE_COMP_11 0x52B
#define REGISTER_DOSE_COMP_12 0x52C
#define REGISTER_DOSE_COMP_13 0x52D
#define REGISTER_DOSE_COMP_14 0x52E

#define REGISTER_DOSE_COMP_15 0x0530
#define REGISTER_DOSE_COMP_16 0x0531
#define REGISTER_DOSE_COMP_17 0x0532
#define REGISTER_DOSE_COMP_18 0x0533
#define REGISTER_DOSE_COMP_19 0x0534
#define REGISTER_DOSE_COMP_20 0x0535
#define REGISTER_DOSE_COMP_21 0x0536
#define REGISTER_DOSE_COMP_22 0x0537
#define REGISTER_DOSE_COMP_23 0x0538
#define REGISTER_DOSE_COMP_24 0x0539
#define REGISTER_DOSE_COMP_25 0x53A
#define REGISTER_DOSE_COMP_26 0x53B
#define REGISTER_DOSE_COMP_27 0x53C
#define REGISTER_DOSE_COMP_28 0x53D
#define REGISTER_DOSE_COMP_29 0x53E

#define REGISTER_CMD_ECB_RESET_FAULTS 0x1000
#define REGISTER_SET_ACCESS_MODE_DEFAULT 0x1002
#define REGISTER_SET_ACCESS_MODE_SERVICE 0x1003
#define REGISTER_SET_ACCESS_MODE_ETM 0x1004
#define REGISTER_CLEAR_EEPROM_WRITE_STATUS 0x1005

#define REGISTER_CMD_AFC_SELECT_AFC_MODE 0x1100
#define REGISTER_CMD_AFC_SELECT_MANUAL_MODE 0x1101
#define REGISTER_CMD_AFC_MANUAL_TARGET_POSITION 0x1102
#define REGISTER_CMD_COOLANT_INTERFACE_SET_SF6_PULSES_IN_BOTTLE 0x1104
#define REGISTER_SYSTEM_SET_TIME 0x1105
#define REGISTER_SYSTEM_ENABLE_HIGH_SPEED_LOGGING 0x1106
#define REGISTER_SYSTEM_DISABLE_HIGH_SPEED_LOGGING 0x1107
#define REGISTER_SYSTEM_LOAD_FACTORY_DEFAULTS_AND_REBOOT 0x1108
#define REGISTER_SYSTEM_SAVE_CURRENT_SETTINGS_TO_CUSTOMER_SAVE 0x1109
#define REGISTER_SYSTEM_LOAD_CUSTOMER_SETTINGS_SAVE_AND_REBOOT 0x110A
#define REGISTER_REMOTE_IP_ADDRESS 0x110B
#define REGISTER_IP_ADDRESS 0x110C
#define REGISTER_SCOPE_DATA_SOURCE_GENERIC 0x1120
#define REGISTER_SCOPE_DATA_SOURCE_HV_VMON 0x1121
      

//#define REGISTER_DEBUG_TOGGLE_RESET_DEBUG 0x1200
#define REGISTER_DEBUG_RESET_MCU 0x1201
#define REGISTER_ETM_SYSTEM_SERIAL_NUMBER 0x1202
//#define REGISTER_DEBUG_GUN_DRIVER_RESET_FPGA 0x1203
#define REGISTER_ETM_ECB_RESET_ARC_AND_PULSE_COUNT 0x1204
#define REGISTER_ETM_ECB_RESET_SECONDS_POWERED_HV_ON_XRAY_ON 0x1205
#define REGISTER_ETM_ECB_LOAD_DEFAULT_SYSTEM_SETTINGS_AND_REBOOT 0x1206
#define REGISTER_ETM_SET_REVISION_AND_SERIAL_NUMBER 0x1207
#define REGISTER_ETM_SAVE_CURRENT_SETTINGS_TO_FACTORY_DEFAULT 0x1208
#define REGISTER_ETM_SLAVE_LOAD_DEFAULT_CALIBRATION 0x1209
#define REGISTER_DEBUG_SET_RAM_DEBUG 0x1210
#define REGISTER_DEBUG_SET_EEPROM_DEBUG 0x1211
#define REGISTER_ETM_SLAVE_SET_CALIBRATION_PAIR 0x1212
#define REGISTER_ETM_CLEAR_DEBUG 0x1213
#define REGISTER_ETM_IGNORE_FAULTS 0x1214

void ExecuteEthernetCommand(void) {
  ETMEthernetMessageFromGUI next_message;
  unsigned int eeprom_page_data[16];
  unsigned int page_to_read;

  // DPARKER PUT NEXT_MESSAGE BACK IN
  next_message = GetNextMessageFromGUI();
  if (next_message.index == 0xFFFF) {
    // there was no message
    return;
  }
  
  // This message needs to be processsed by the ethernet control board


  SendToEventLog(next_message.index);
  
  switch (next_message.index) {
    
  case REGISTER_CMD_ECB_RESET_FAULTS:
    global_data_A37780.reset_requested = 1;
    ETMCanMasterStatusFaultResetAll();
    global_data_A37780.drive_up_fault_counter = 0;
    global_data_A37780.high_voltage_on_fault_counter = 0;
    break;

    /*
      COMMAND REMOVED - WE NOT LONGER SEND THIS COMMAND
  case REGISTER_CMD_COOLANT_INTERFACE_ALLOW_25_MORE_SF6_PULSES:
    ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_COOLING_INTERFACE_BOARD << 2)),
			ETM_CAN_REGISTER_COOLING_CMD_SF6_PULSE_LIMIT_OVERRIDE,
			0,
			0,
			0);
    break;
    */
    
  case REGISTER_SET_ACCESS_MODE_DEFAULT:
    global_data_A37780.access_mode = ACCESS_MODE_DEFAULT;
    break;
    
  case REGISTER_SET_ACCESS_MODE_SERVICE:
    if (next_message.data_3 == global_data_A37780.service_passcode) {
      global_data_A37780.access_mode = ACCESS_MODE_SERVICE;
    }
    break;
    
  case REGISTER_SET_ACCESS_MODE_ETM:
    if (next_message.data_3 == global_data_A37780.etm_passcode) {
      global_data_A37780.access_mode = ACCESS_MODE_ETM;
    }
    break;


  case REGISTER_CLEAR_EEPROM_WRITE_STATUS:
    global_data_A37780.eeprom_write_status = EEPROM_WRITE_WAITING;
    break;
  }
  
  if (global_data_A37780.access_mode == ACCESS_MODE_ETM) {
    switch (next_message.index) {
      
    case REGISTER_ETM_SYSTEM_SERIAL_NUMBER:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
      ecb_data.config.system_serial_letter           = next_message.data_3;
      ecb_data.config.system_serial_number_high_word = next_message.data_2;
      ecb_data.config.system_serial_number_low_word  = next_message.data_1;
      if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, (unsigned int*)&ecb_data.config)) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
      }
      break;

    case REGISTER_ETM_ECB_RESET_ARC_AND_PULSE_COUNT:
      ecb_data.system_counters.arc_counter = 0;
      ecb_data.system_counters.pulse_counter = 0;
      break;

    case REGISTER_ETM_ECB_RESET_SECONDS_POWERED_HV_ON_XRAY_ON:
      ecb_data.system_counters.powered_seconds = 0;
      ecb_data.system_counters.hv_on_seconds = 0;
      ecb_data.system_counters.xray_on_seconds = 0;
      break;

    case REGISTER_ETM_ECB_LOAD_DEFAULT_SYSTEM_SETTINGS_AND_REBOOT:
      LoadDefaultSystemCalibrationToEEProm();
      __delay32(1000000);
      __asm__ ("Reset");
      break;

    case REGISTER_DEBUG_RESET_MCU:
      if ((ecb_data.control_state < STATE_DRIVE_UP) || (ecb_data.control_state > STATE_XRAY_ON)) {
	if (next_message.data_3 == ETM_CAN_ADDR_ETHERNET_BOARD) {
	  __asm__ ("Reset");
	} else {
	  ETMCanMasterSendSlaveResetMCU(next_message.data_3);
	}
      }
      break;

    case REGISTER_ETM_SET_REVISION_AND_SERIAL_NUMBER:
      if (next_message.data_3 == ETM_CAN_ADDR_ETHERNET_BOARD) {
	// Set the rev and S/N for the ECB
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
	ecb_data.config.ecb_serial_number_low_word  = next_message.data_2;
	ecb_data.config.ecb_agile_rev_ASCII_x2      = next_message.data_1;
	if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ECB_BOARD_CONFIGURATION, (unsigned int*)&ecb_data.config)) {
	  // The update was successful
	  global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	}
      } else {
	// Set the rev and S/N for the Slave
	ETMCanMasterSendSlaveRevAndSerialNumber(next_message.data_3, next_message.data_2, next_message.data_1);
      }
      break;

    case REGISTER_ETM_SLAVE_LOAD_DEFAULT_CALIBRATION:
      if (next_message.data_3 == ETM_CAN_ADDR_ETHERNET_BOARD) {
	// DO NOTHING
      } else {
	ETMCanMasterSendSlaveLoadDefaultEEpromData(next_message.data_3);
      }
      break;

    case REGISTER_ETM_SLAVE_SET_CALIBRATION_PAIR:
      if (next_message.data_3 == ETM_CAN_ADDR_ETHERNET_BOARD) {
	// DO NOTHING
      } else {
	ETMCanMasterSendSlaveCalibrationPair(next_message.data_3, next_message.data_2, next_message.data_1, next_message.data_0);
      }
      break;

    case REGISTER_ETM_CLEAR_DEBUG:
      ETMCanMasterClearECBDebug();
      //ETMCanMasterSendSlaveClearDebug();
      break;
      
    case REGISTER_ETM_IGNORE_FAULTS:
      ETMCanMasterSendSlaveIgnoreMessage(next_message.data_3, next_message.data_2, next_message.data_1, next_message.data_0);
      break;
      
    case REGISTER_ETM_SAVE_CURRENT_SETTINGS_TO_FACTORY_DEFAULT:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
      while (global_data_A37780.eeprom_write_status == EEPROM_WRITE_FAILURE) {
	CopyCurrentConfig(USE_FACTORY_DEFAULTS);
      }
      break;
      
    }
  }
  
  
  if ((global_data_A37780.access_mode == ACCESS_MODE_SERVICE) || (global_data_A37780.access_mode == ACCESS_MODE_ETM)) {
    
    switch (next_message.index) {

      // -------------------- DOSE 0 SETTINGS ----------------------- //
      
    case REGISTER_HVPS_SET_POINT_DOSE_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 0), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_0.hvps_set_point = next_message.data_3;
      }
      break;

    case REGISTER_ELECTROMAGNET_CURRENT_DOSE_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 1), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_0.electromagnet_set_point = next_message.data_3;
      }
      break;
    
    case REGISTER_GUN_DRIVER_PULSE_TOP_VOLTAGE_DOSE_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 2), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_0.gun_driver_pulse_top_voltage = next_message.data_3;
      }
      break;

    case REGISTER_GUN_DRIVER_CATHODE_VOLTAGE_DOSE_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 3), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_0.gun_driver_cathode_voltage = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_SPARE_DOSE_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 4), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_0.trigger_delay_spare = next_message.data_3;
      }
      break;
      
    case REGISTER_TRIGGER_AFC_DOSE_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 5), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_0.trigger_delay_afc = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_GRID_START_MIN_DOSE_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 6), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_0.trigger_grid_start_min_dose = next_message.data_3;
      }
      break;
        
    case REGISTER_TRIGGER_GRID_START_MAX_DOSE_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 7), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_0.trigger_grid_start_max_dose = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_GRID_STOP_MIN_DOSE_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 8), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_0.trigger_grid_stop_min_dose = next_message.data_3;
      }
      break;    
      
    case REGISTER_TRIGGER_GRID_STOP_MAX_DOSE_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 9), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_0.trigger_grid_stop_max_dose = next_message.data_3;
      }
      break;    

    case REGISTER_AFC_HOME_POSITION_DOSE_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 10), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_0.afc_home_poistion = next_message.data_3;
      }
      break;

    case REGISTER_SELF_TRIGGER_PRF_DOSE_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_0<<4) + 11), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_0.self_trigger_prf = next_message.data_3;
      }
      break;


      // -------------------- DOSE 1 SETTINGS ----------------------- //

    case REGISTER_HVPS_SET_POINT_DOSE_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 0), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_1.hvps_set_point = next_message.data_3;
      }
      break;

    case REGISTER_ELECTROMAGNET_CURRENT_DOSE_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 1), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_1.electromagnet_set_point = next_message.data_3;
      }
      break;
    
    case REGISTER_GUN_DRIVER_PULSE_TOP_VOLTAGE_DOSE_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 2), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_1.gun_driver_pulse_top_voltage = next_message.data_3;
      }
      break;

    case REGISTER_GUN_DRIVER_CATHODE_VOLTAGE_DOSE_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 3), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_1.gun_driver_cathode_voltage = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_SPARE_DOSE_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 4), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_1.trigger_delay_spare = next_message.data_3;
      }
      break;
      
    case REGISTER_TRIGGER_AFC_DOSE_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 5), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_1.trigger_delay_afc = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_GRID_START_MIN_DOSE_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 6), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_1.trigger_grid_start_min_dose = next_message.data_3;
      }
      break;
        
    case REGISTER_TRIGGER_GRID_START_MAX_DOSE_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 7), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_1.trigger_grid_start_max_dose = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_GRID_STOP_MIN_DOSE_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 8), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_1.trigger_grid_stop_min_dose = next_message.data_3;
      }
      break;    
      
    case REGISTER_TRIGGER_GRID_STOP_MAX_DOSE_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 9), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_1.trigger_grid_stop_max_dose = next_message.data_3;
      }
      break;    

    case REGISTER_AFC_HOME_POSITION_DOSE_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 10), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_1.afc_home_poistion = next_message.data_3;
      }
      break;

    case REGISTER_SELF_TRIGGER_PRF_DOSE_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_1<<4) + 11), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_1.self_trigger_prf = next_message.data_3;
      }
      break;


      // -------------------- DOSE 2 SETTINGS ----------------------- //

    case REGISTER_HVPS_SET_POINT_DOSE_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_2<<4) + 0), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_2.hvps_set_point = next_message.data_3;
      }
      break;

    case REGISTER_ELECTROMAGNET_CURRENT_DOSE_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_2<<4) + 1), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_2.electromagnet_set_point = next_message.data_3;
      }
      break;
    
    case REGISTER_GUN_DRIVER_PULSE_TOP_VOLTAGE_DOSE_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_2<<4) + 2), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_2.gun_driver_pulse_top_voltage = next_message.data_3;
      }
      break;

    case REGISTER_GUN_DRIVER_CATHODE_VOLTAGE_DOSE_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_2<<4) + 3), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_2.gun_driver_cathode_voltage = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_SPARE_DOSE_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_2<<4) + 4), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_2.trigger_delay_spare = next_message.data_3;
      }
      break;
      
    case REGISTER_TRIGGER_AFC_DOSE_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_2<<4) + 5), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_2.trigger_delay_afc = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_GRID_START_MIN_DOSE_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_2<<4) + 6), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_2.trigger_grid_start_min_dose = next_message.data_3;
      }
      break;
        
    case REGISTER_TRIGGER_GRID_START_MAX_DOSE_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_2<<4) + 7), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_2.trigger_grid_start_max_dose = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_GRID_STOP_MIN_DOSE_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_2<<4) + 8), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_2.trigger_grid_stop_min_dose = next_message.data_3;
      }
      break;    
      
    case REGISTER_TRIGGER_GRID_STOP_MAX_DOSE_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_2<<4) + 9), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_2.trigger_grid_stop_max_dose = next_message.data_3;
      }
      break;    

    case REGISTER_AFC_HOME_POSITION_DOSE_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_2<<4) + 10), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_2.afc_home_poistion = next_message.data_3;
      }
      break;

    case REGISTER_SELF_TRIGGER_PRF_DOSE_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_2<<4) + 11), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_2.self_trigger_prf = next_message.data_3;
      }
      break;

      // -------------------- DOSE 3 SETTINGS ----------------------- //
      
    case REGISTER_HVPS_SET_POINT_DOSE_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_3<<4) + 0), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_3.hvps_set_point = next_message.data_3;
      }
      break;

    case REGISTER_ELECTROMAGNET_CURRENT_DOSE_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_3<<4) + 1), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_3.electromagnet_set_point = next_message.data_3;
      }
      break;
    
    case REGISTER_GUN_DRIVER_PULSE_TOP_VOLTAGE_DOSE_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_3<<4) + 2), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_3.gun_driver_pulse_top_voltage = next_message.data_3;
      }
      break;

    case REGISTER_GUN_DRIVER_CATHODE_VOLTAGE_DOSE_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_3<<4) + 3), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_3.gun_driver_cathode_voltage = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_SPARE_DOSE_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_3<<4) + 4), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_3.trigger_delay_spare = next_message.data_3;
      }
      break;
      
    case REGISTER_TRIGGER_AFC_DOSE_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_3<<4) + 5), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_3.trigger_delay_afc = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_GRID_START_MIN_DOSE_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_3<<4) + 6), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_3.trigger_grid_start_min_dose = next_message.data_3;
      }
      break;
        
    case REGISTER_TRIGGER_GRID_START_MAX_DOSE_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_3<<4) + 7), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_3.trigger_grid_start_max_dose = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_GRID_STOP_MIN_DOSE_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_3<<4) + 8), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_3.trigger_grid_stop_min_dose = next_message.data_3;
      }
      break;    
      
    case REGISTER_TRIGGER_GRID_STOP_MAX_DOSE_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_3<<4) + 9), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_3.trigger_grid_stop_max_dose = next_message.data_3;
      }
      break;    

    case REGISTER_AFC_HOME_POSITION_DOSE_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_3<<4) + 10), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_3.afc_home_poistion = next_message.data_3;
      }
      break;

    case REGISTER_SELF_TRIGGER_PRF_DOSE_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_3<<4) + 11), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_3.self_trigger_prf = next_message.data_3;
      }
      break;
      
      // -------------------- DOSE ALL COMMANDS ----------------------- //
    
    
    case REGISTER_MAGNETRON_HEATER_CURRENT_DOSE_ALL:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 0), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_all.magnetron_heater_current_at_standby = next_message.data_3;
      }
      break;

    case REGISTER_GUN_DRIVER_HEATER_VOLTAGE_DOSE_ALL:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 1), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_all.gun_driver_heater_voltage = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_HVPS_START_DOSE_ALL:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 2), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_all.trigger_hvps_start = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_HVPS_STOP_DOSE_ALL:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 3), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_all.trigger_hvps_stop = next_message.data_3;
      }
      break;
    
    case REGISTER_TRIGGER_PFN_DOSE_ALL:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 4), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_all.trigger_pfn = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_MAGNETRON_AND_TARGET_CURRENT_START_DOSE_ALL:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 5), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_all.trigger_magnetron_and_target_current_start = next_message.data_3;
      }
      break;

    case REGISTER_TRIGGER_MAGNETRON_AND_TARGET_CURRENT_STOP_DOSE_ALL:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 6), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_all.trigger_magnetron_and_target_current_stop = next_message.data_3;
      }
      break;

    case REGISTER_X_RAY_ON_TIME_DOSE_ALL:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 7), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_all.x_ray_run_time_in_automated_mode = next_message.data_3;
      }
      break;

    case REGISTER_GUN_BIAS_VOLTAGE_DOSE_ALL:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 8), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_all.gun_driver_bias_voltage = next_message.data_3;
      }
      break;

    case REGISTER_AFC_AFT_CONTROL_VOLTAGE_DOSE_ALL:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 9), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_all.afc_aux_control_or_offset = next_message.data_3;
      }
      break;

    case REGISTER_CMD_AFC_MANUAL_TARGET_POSITION:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 10), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_all.afc_manual_target_position = next_message.data_3;
      }
      break;
      
    case REGISTER_AFC_LOCKED_TO_HOME_DOSE_ALL:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_SETTING_ALL<<4) + 11), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_level_all.afc_locked_mode = next_message.data_3;
      }
      break;
      
    case REGISTER_AUX_SETTING_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS<<4) + 0), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.aux_set_points.aux_set_point_0 = next_message.data_3;
      }
      break;

    case REGISTER_AUX_SETTING_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS<<4) + 1), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.aux_set_points.aux_set_point_1 = next_message.data_3;
      }
      break;

    case REGISTER_AUX_SETTING_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS<<4) + 2), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.aux_set_points.aux_set_point_2 = next_message.data_3;
      }
      break;

    case REGISTER_AUX_SETTING_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS<<4) + 3), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.aux_set_points.aux_set_point_3 = next_message.data_3;
      }
      break;

      case REGISTER_AUX_SETTING_4:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS<<4) + 4), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.aux_set_points.aux_set_point_4 = next_message.data_3;
      }
      break;

    case REGISTER_AUX_SETTING_5:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS<<4) + 5), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.aux_set_points.aux_set_point_5 = next_message.data_3;
      }
      break;

    case REGISTER_AUX_SETTING_6:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS<<4) + 6), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.aux_set_points.aux_set_point_6 = next_message.data_3;
      }
      break;

    case REGISTER_AUX_SETTING_7:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_SLAVE_AUX_SETTINGS<<4) + 7), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.aux_set_points.aux_set_point_7 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_0:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 0), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_0 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_1:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 1), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_1 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_2:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 2), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_2 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_3:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 3), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_3 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_4:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 4), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_4 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_5:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 5), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_5 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_6:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 6), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_6 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_7:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 7), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_7 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_8:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 8), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_8 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_9:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 9), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_9 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_10:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 10), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_10 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_11:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 11), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_11 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_12:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 12), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_12 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_13:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 13), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_13 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_14:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_A<<4) + 14), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_a.compensation_14 = next_message.data_3;
      }
      break;


    case REGISTER_DOSE_COMP_15:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 0), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_0 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_16:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 1), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_1 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_17:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 2), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_2 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_18:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 3), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_3 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_19:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 4), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_4 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_20:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 5), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_5 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_21:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 6), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_6 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_22:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 7), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_7 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_23:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 8), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_8 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_24:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 9), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_9 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_25:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 10), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_10 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_26:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 11), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_11 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_27:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 12), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_12 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_28:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 13), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_13 = next_message.data_3;
      }
      break;

    case REGISTER_DOSE_COMP_29:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;    
      if (ETMEEPromWriteWordWithConfirmation(((EEPROM_PAGE_ECB_DOSE_COMPENSATION_B<<4) + 14), next_message.data_3) == 0xFFFF) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
	ecb_data.dose_compensation_group_b.compensation_14 = next_message.data_3;
      }
      break;

    case REGISTER_REMOTE_IP_ADDRESS:
      //ETMEEPromWriteWord(next_message.index, next_message.data_3);
      //ETMEEPromWriteWord(next_message.index + 1, next_message.data_2);
      break;
      
    case REGISTER_IP_ADDRESS:
      //ETMEEPromWriteWord(next_message.index, next_message.data_3);
      //ETMEEPromWriteWord(next_message.index + 1, next_message.data_2);
      break;
      
    case REGISTER_CMD_AFC_SELECT_AFC_MODE:
      ETMCanMasterSendDiscreteCMD(DISCRETE_CMD_AFC_SELECT_AUTOMATIC_MODE);
      break;

    case REGISTER_CMD_AFC_SELECT_MANUAL_MODE:
      ETMCanMasterSendDiscreteCMD(DISCRETE_CMD_AFC_SELECT_MANUAL_MODE);
      break;

      /*
    case REGISTER_CMD_COOLANT_INTERFACE_ALLOW_SF6_PULSES_WHEN_PRESSURE_BELOW_LIMIT:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_COOLING_INTERFACE_BOARD << 2)),
			  ETM_CAN_REGISTER_COOLING_CMD_SF6_LEAK_LIMIT_OVERRIDE,
			  0,
			  0,
			  next_message.data_3);
      break;
      */
      
    case REGISTER_CMD_COOLANT_INTERFACE_SET_SF6_PULSES_IN_BOTTLE:
      ETMCanMasterSendDiscreteCMD(DISCRETE_CMD_COOLING_RESET_BOTTLE_COUNT);
      // DPARKER need a mechanism for setting the number of pulses in the bottle
      break;
      
    case REGISTER_SYSTEM_SAVE_CURRENT_SETTINGS_TO_CUSTOMER_SAVE:
      global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
      while (global_data_A37780.eeprom_write_status == EEPROM_WRITE_FAILURE) {
	CopyCurrentConfig(USE_CUSTOMER_BACKUP);
      }
      break;
      
    case REGISTER_SYSTEM_LOAD_FACTORY_DEFAULTS_AND_REBOOT:
      if ((ecb_data.control_state < STATE_DRIVE_UP) || (ecb_data.control_state > STATE_XRAY_ON)) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
	LoadConfig(USE_FACTORY_DEFAULTS);
	if (global_data_A37780.eeprom_write_status == EEPROM_WRITE_SUCCESSFUL) {
	  __delay32(1000000);
	  __asm__ ("Reset");
	}
      }
      break;

    case REGISTER_SYSTEM_LOAD_CUSTOMER_SETTINGS_SAVE_AND_REBOOT:
      if ((ecb_data.control_state < STATE_DRIVE_UP) || (ecb_data.control_state > STATE_XRAY_ON)) {
	global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;
	LoadConfig(USE_CUSTOMER_BACKUP);
	if (global_data_A37780.eeprom_write_status == EEPROM_WRITE_SUCCESSFUL) {
	  __delay32(1000000);
	  __asm__ ("Reset");
	}
      }
      break;

    case REGISTER_SYSTEM_ENABLE_HIGH_SPEED_LOGGING:
      // DPARKER - Clear the data in the pulse log so that we don't get old data in the log
      ETMCanMasterClearHighSpeedLogging();
      ETMCanMasterSyncSet(SYNC_BIT_ENABLE_PULSE_LOG, 1);
      break;
      
    case REGISTER_SYSTEM_DISABLE_HIGH_SPEED_LOGGING:
      ETMCanMasterSyncSet(SYNC_BIT_ENABLE_PULSE_LOG, 0);
      break;

    case REGISTER_DEBUG_SET_RAM_DEBUG:
      if (next_message.data_3 == ETM_CAN_ADDR_ETHERNET_BOARD) {
	// Debug Ram Loactions on the ECB
	next_message.data_2 <<= 1;
	next_message.data_1 <<= 1;
	next_message.data_0 <<= 1;
	if (next_message.data_2 < RAM_SIZE_WORDS) {
	  global_data_A37780.ram_ptr_a = (unsigned int*)next_message.data_2;
	}
	if (next_message.data_1 < RAM_SIZE_WORDS) {
	  global_data_A37780.ram_ptr_b = (unsigned int*)next_message.data_1;
	}
	if (next_message.data_0 < RAM_SIZE_WORDS) {
	  global_data_A37780.ram_ptr_c = (unsigned int*)next_message.data_0;
	}
      } else {
	// DEbug Ram Loactions on a slave
	ETMCanMasterSendSlaveRAMDebugLocations(next_message.data_3, next_message.data_2, next_message.data_1, next_message.data_0);}
      break;

    case REGISTER_SCOPE_DATA_SOURCE_GENERIC:
      ETMCanMasterSelectScopeDataSourceGeneric(next_message.data_3, next_message.data_2);
      break;

    case REGISTER_SCOPE_DATA_SOURCE_HV_VMON:
      ETMCanMasterSelectScopeDataSourceHVVmon(next_message.data_3);
      break;



      
    case REGISTER_DEBUG_SET_EEPROM_DEBUG:
      if (next_message.data_3 == ETM_CAN_ADDR_ETHERNET_BOARD) {
	// Debug Ram Loactions on the ECB
	page_to_read = next_message.data_2>>4;
	if (ETMEEPromPrivateReadSinglePage(page_to_read, eeprom_page_data) == 0xFFFF) {
	  debug_data_ecb.eeprom_read_result = eeprom_page_data[(next_message.data_2 & 0x000F)];
	} else {
	  debug_data_ecb.eeprom_read_result = 0xFFEF;
	}
      } else {
	ETMCanMasterSendSlaveEEPROMDebug(next_message.data_3, next_message.data_2);
      }
      break;
           
    }
  }
}





void CopyCurrentConfig(unsigned int destination) {
  unsigned int dose_setting_0_destination_page;
  unsigned int dose_setting_1_destination_page;
  unsigned int dose_setting_all_destination_page;
  unsigned int dose_setting_data[16];


  // DPARKER - NEED TO EXTEND THIS TO COPY ALL DOSE DATA
  // DOES NOT COPY ALL OF THE DOSE SETTINGS OR THE AUX SETTINGS

  
  global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;

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
  
  global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;

}


void LoadConfig(unsigned int source) {
  unsigned int dose_setting_0_source_page;
  unsigned int dose_setting_1_source_page;
  unsigned int dose_setting_all_source_page;
  unsigned int dose_setting_data[16];


  // DPARKER THIS NEEDS TO BE EXTENDED TO INCLUDE ALL THE DOSE LEVELS, AUX_REGISTERS, DOSE COMP, ECT
  
  global_data_A37780.eeprom_write_status = EEPROM_WRITE_FAILURE;

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


  global_data_A37780.eeprom_write_status = EEPROM_WRITE_SUCCESSFUL;
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




#define T2_CHARGE_TIME   42000 //2.1mS
#define T2_HOLDOFF_100US  2000


#define OCxCON_VALUE   0b0000000000000100 // TMR2, Single Output Pulse


// Internal Trigger - DPARKER WORK ON THIS.  IT MAY MAKE TESTING EASIER
/*
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
  unsigned int next_dose_level;
  if (mode_select_internal_trigger == 1) {
    T2CONbits.TON = 0;
    if (ecb_data.control_state == STATE_XRAY_ON) {
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

    // Figure out the next dose level
    next_dose_level = (global_data_A37780.dose_level^0x0001);
    if (global_data_A37780.single_dual_energy_mode_selection == OPERATION_MODE_SINGLE_ENERGY) {
      next_dose_level = global_data_A37780.dose_level;
    }
    global_data_A37780.dose_level = next_dose_level;
    SetDoseLevelTiming();
    
    while(!_T2IF) {} // Wait for Holdoff Period
  }
  _T5IF = 0;
}
*/  



// External Trigger
void __attribute__((interrupt, shadow, no_auto_psv)) _INT1Interrupt(void) {
  unsigned int next_dose_level;

  if (mode_select_internal_trigger == 0) {
    if (_INT1EP) {
      // This was a negative going edge transistion
      // Start Charging the power supply
      // Start the power supply timer
      // Send out the sync message
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
	if (ecb_data.control_state == STATE_XRAY_ON) {
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
	next_dose_level = (global_data_A37780.dose_level^0x0001);
	if (global_data_A37780.single_dual_energy_mode_selection == OPERATION_MODE_SINGLE_ENERGY) {
	  next_dose_level = global_data_A37780.dose_level;
	}
	global_data_A37780.dose_level = next_dose_level;
	SetDoseLevelTiming();

	while(!_T2IF) {} // Wait for Holdoff Period
	if (PIN_TRIGGER_IN != ILL_TRIGGER_ACTIVE) {
	  fault_data.trigger_width_too_short_count++;
	}
	_INT1IE = 0;
	_INT1EP = 1;  // Negative Transition
	_INT1IE = 1;
	ETMCanMasterSendSyncMessage(global_data_A37780.dose_level, global_data_A37780.pulse_counter++);
	ecb_data.system_counters.pulse_counter++;
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
  SetTriggerTiming(TRIGGER_PFN_TRIGGER, ecb_data.dose_level_all.trigger_pfn, TRIGGER_STOP_TIME);
  SetTriggerTiming(TRIGGER_MAGNETRON_I_SAMP, ecb_data.dose_level_all.trigger_magnetron_and_target_current_start, TRIGGER_STOP_TIME);
  SetTriggerTiming(TRIGGER_TARGET_I_SAMP, ecb_data.dose_level_all.trigger_magnetron_and_target_current_start, TRIGGER_STOP_TIME);
  SetTriggerTiming(TRIGGER_BALANCED_OUT_1, 0, TRIGGER_STOP_TIME);
}




void SetDoseLevelTiming(void) {
  
  switch (global_data_A37780.dose_level) {

  case DOSE_LEVEL_CARGO_HIGH:
    SetTriggerTiming(TRIGGER_GRID_TRIGGER, ecb_data.dose_level_1.trigger_grid_start_max_dose, ecb_data.dose_level_1.trigger_grid_stop_max_dose);
    SetTriggerTiming(TRIGGER_AFC_SAMPLE, ecb_data.dose_level_1.trigger_delay_afc, TRIGGER_STOP_TIME);
    break;

  case DOSE_LEVEL_CARGO_LOW:
    SetTriggerTiming(TRIGGER_GRID_TRIGGER, ecb_data.dose_level_0.trigger_grid_start_max_dose, ecb_data.dose_level_0.trigger_grid_stop_max_dose);
    SetTriggerTiming(TRIGGER_AFC_SAMPLE, ecb_data.dose_level_0.trigger_delay_afc, TRIGGER_STOP_TIME);
    break;

  case DOSE_LEVEL_CAB_HIGH:
    break;

  case DOSE_LEVEL_CAB_LOW:
    break;
  }
}



void GridDelayTrim(unsigned int start_trim, unsigned int stop_trim) {
  // Program the start_trim
  unsigned int n;
  
  PIN_OUT_DELAY_PGM_CLK = 0;


  PIN_OUT_GRID_START_PROG_EN = 1;
  for (n = 0; n < 8; n++) {
    // Shift out a bit
    if (start_trim & 0x0080) {
      PIN_OUT_DELAY_PGM_DO = 1;
    } else {
      PIN_OUT_DELAY_PGM_DO = 0;
    }
    PIN_OUT_DELAY_PGM_CLK = 1;
    start_trim <<= 1;
    PIN_OUT_DELAY_PGM_CLK = 0;
  }
  PIN_OUT_GRID_START_PROG_EN = 0;


  PIN_OUT_GRID_STOP_PROG_EN = 1;
  for (n = 0; n < 8; n++) {
    // Shift out a bit
    if (stop_trim & 0x0080) {
      PIN_OUT_DELAY_PGM_DO = 1;
    } else {
      PIN_OUT_DELAY_PGM_DO = 0;
    }
    PIN_OUT_DELAY_PGM_CLK = 1;
    stop_trim <<= 1;
    PIN_OUT_DELAY_PGM_CLK = 0;
  }
  PIN_OUT_GRID_STOP_PROG_EN = 0;
  

}




void SetTriggerTiming(unsigned int trigger_type, unsigned int start_time, unsigned int stop_time) {
  // DPARKER - error check start and stop time relative to each other and maximums to guarantee they happen

  switch (trigger_type) {

  case TRIGGER_GRID_TRIGGER:
    OC1R  = start_time;
    OC1RS = stop_time;

    // DPARKER - delay start by 100 units
    // DPARKER - delay stop by 200 units

    GridDelayTrim(100, 200);


    
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

#define OCxCON_VALUE_FREE_RUNNING 0b0000000000000101
#define OCxR_TEST_START_TIME      100
#define OCxR_TEST_STOP_TIME       200 // 40uS Pulse

void SendTestTriggers(void) {
  // Set up triggers to send out 40uS pulses at 100Hz
  T2CON = 0;

  OC1CON = OCxCON_VALUE_FREE_RUNNING;
  OC1R  = OCxR_TEST_START_TIME;
  OC1RS = OCxR_TEST_STOP_TIME;

  OC2CON = OCxCON_VALUE_FREE_RUNNING;  
  OC2R  = OCxR_TEST_START_TIME;
  OC2RS = OCxR_TEST_STOP_TIME;

  OC3CON = OCxCON_VALUE_FREE_RUNNING;  
  OC3R  = OCxR_TEST_START_TIME;
  OC3RS = OCxR_TEST_STOP_TIME;
  
  OC4CON = OCxCON_VALUE_FREE_RUNNING;  
  OC4R  = OCxR_TEST_START_TIME;
  OC4RS = OCxR_TEST_STOP_TIME;
  
  OC5CON = OCxCON_VALUE_FREE_RUNNING;  
  OC5R  = OCxR_TEST_START_TIME;
  OC5RS = OCxR_TEST_STOP_TIME;

  OC6CON = OCxCON_VALUE_FREE_RUNNING;  
  OC6R  = OCxR_TEST_START_TIME;
  OC6RS = OCxR_TEST_STOP_TIME;

  OC7CON = OCxCON_VALUE_FREE_RUNNING;  
  OC7R  = OCxR_TEST_START_TIME;
  OC7RS = OCxR_TEST_STOP_TIME;

  OC8CON = 0;  // This is not a fiber trigger, so don't pulse it

  TMR2  = 0;
  PR2   = 25000;  // 10 Milliseconds
  T2CON = T2CON_VALUE_TIMER_ON_SCALE_1_8;
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




void MCP23S18Setup(unsigned long pin_chip_select_not,
		   unsigned char spi_port,
		   unsigned long fcy_clk,
		   unsigned long spi_bit_rate) {

  mcp23S18_pin_chip_select_not = pin_chip_select_not;
  mcp23S18_spi_port            = spi_port;

  ETMSetPin(mcp23S18_pin_chip_select_not);
  ETMPinTrisOutput(mcp23S18_pin_chip_select_not);
  
  ConfigureSPI(mcp23S18_spi_port,
	       ETM_DEFAULT_SPI_CON_VALUE,
	       ETM_DEFAULT_SPI_CON2_VALUE,
	       ETM_DEFAULT_SPI_STAT_VALUE,
	       spi_bit_rate,
	       fcy_clk);

  // IOCON is set up how we want it at power on
}


//#define DISCRETE_INPUT_XOR_MASK 0b1100000001101000 // This flips the values to make sense based on the name
#define DISCRETE_INPUT_XOR_MASK 0b1100000001111000 // This flips the values to make sense based on the name
#define OP_CODE_ADDRESS_PORT_DATA_READ  0b0100000100010010

unsigned int MCP23S18UpdateInputs(void) {
  unsigned long return_value;
  unsigned int temp;
  // This assumes SPI Port is configured as 16bit

  ETMClearPin(mcp23S18_pin_chip_select_not);

  // Send out the op code and address
  if (SendAndReceiveSPI(OP_CODE_ADDRESS_PORT_DATA_READ, mcp23S18_spi_port) & 0xFFFF0000) {
    // There was an error sending out the op code and address
    ETMSetPin(mcp23S18_pin_chip_select_not);
    return 0;
  }

  return_value = SendAndReceiveSPI(0, mcp23S18_spi_port);
  // Read the port values
  if (return_value & 0xFFFF0000) {
    // There was an error reading the port data
    ETMSetPin(mcp23S18_pin_chip_select_not);
    return 0;
  }

  ETMSetPin(mcp23S18_pin_chip_select_not);
  
  temp = return_value;
  temp ^= DISCRETE_INPUT_XOR_MASK;
  *(unsigned int*)&ecb_data.io_expander_inputs = temp;
  
  return 0xFFFF;
}




void SetACContactor(unsigned int contactor_state) {
  if (contactor_state == CONTACTOR_CLOSED) {
    PIN_OUT_AC_CONTACTOR_ENABLE = 0;
  } else {
    PIN_OUT_AC_CONTACTOR_ENABLE = 1;
  }
}

void SetHVContactor(unsigned int contactor_state) {
  if (contactor_state == CONTACTOR_CLOSED) {
    PIN_OUT_HV_CONTACTOR_ENABLE = 0;
  } else {
    PIN_OUT_HV_CONTACTOR_ENABLE = 1;
  }
}
void SetGUNContactor(unsigned int contactor_state) {
  if (contactor_state == CONTACTOR_CLOSED) {
    PIN_OUT_GUN_CONTACTOR_ENABLE = 0;
  } else {
    PIN_OUT_GUN_CONTACTOR_ENABLE = 1;
  }
}


unsigned int CheckXRayOn(void) {
  if (global_data_A37780.customer_x_ray_on) {
    return 1;
  }
  return 0;
}

