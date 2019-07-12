#include <xc.h>
#include <timer.h>
#include "P1395_CAN_MASTER.h"
#include "ETM_IO_PORTS.H"
#include "ETM_SCALE.H"
#include "ETM_LINAC_MODBUS.h"
#include "ETM_TICK.h"

#define ETM_CAN_REGISTER_PULSE_SYNC_SET_1_MAGNETX_DOSE_0      0x3210
#define ETM_CAN_REGISTER_PULSE_SYNC_SET_1_MAGNETX_DOSE_1      0x3211
#define ETM_CAN_REGISTER_PULSE_SYNC_SET_1_MAGNETX_DOSE_ALL    0x3212


unsigned int can_master_millisecond_counter;
unsigned int previous_disable_xray_value;
unsigned int personality_loaded;


void UpdateSlaveEventLog(ETMCanStatusRegister* previous_status, ETMCanStatusRegister* current_status, unsigned int slave_select);

// DPARKER fix these
extern unsigned int SendCalibrationDataToGUI(unsigned int index, unsigned int scale, unsigned int offset);
unsigned int etm_can_active_debugging_board_id;

// ----------- Can Timers T4 & T5 Configuration ----------- //
#define T4_FREQUENCY_HZ          40  // This is 25mS rate
#define T5_FREQUENCY_HZ          4   // This is 250ms rate

// DPARKER remove the need for timers.h here
#define T4CON_VALUE              (T4_OFF & T4_IDLE_CON & T4_GATE_OFF & T4_PS_1_256 & T4_32BIT_MODE_OFF & T4_SOURCE_INT)
#define T5CON_VALUE              (T5_OFF & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_256 & T5_SOURCE_INT)


typedef struct {
  unsigned int  address;
  unsigned long led;
} TYPE_CAN_PARAMETERS;



ETMCanBoardDebuggingData debug_data_ecb;
ETMCanBoardDebuggingData debug_data_slave_mirror;
ETMCanSyncMessage    etm_can_master_sync_message;


typedef struct {
  unsigned int no_connect_count_ion_pump_board;
  unsigned int no_connect_count_magnetron_current_board;
  unsigned int no_connect_count_pulse_sync_board;
  unsigned int no_connect_count_hv_lambda_board;
  unsigned int no_connect_count_afc_board;
  unsigned int no_connect_count_cooling_interface_board;
  unsigned int no_connect_count_heater_magnet_board;
  unsigned int no_connect_count_gun_driver_board;
  
  unsigned int event_log_counter;
  
  unsigned int buffer_a_ready_to_send;
  unsigned int buffer_a_sent;
  
  unsigned int buffer_b_ready_to_send;
  unsigned int buffer_b_sent;
} TYPE_GLOBAL_DATA_CAN_MASTER;


TYPE_GLOBAL_DATA_CAN_MASTER global_data_can_master;

// --------- Global Buffers --------------- //
TYPE_EVENT_LOG              event_log;
ETMCanHighSpeedData         high_speed_data_buffer_a[HIGH_SPEED_DATA_BUFFER_SIZE];
ETMCanHighSpeedData         high_speed_data_buffer_b[HIGH_SPEED_DATA_BUFFER_SIZE];


// --------- Local Buffers ---------------- // 
ETMCanMessageBuffer         etm_can_master_rx_data_log_buffer;
ETMCanMessageBuffer         etm_can_master_rx_message_buffer;
ETMCanMessageBuffer         etm_can_master_tx_message_buffer;


// ------------- Global Variables ------------ //
unsigned int etm_can_master_next_pulse_level;
unsigned int etm_can_master_next_pulse_count;
unsigned int etm_can_master_next_pulse_prf;


// --------------------- Local Variables -------------------------- //
unsigned int master_high_speed_update_index;
unsigned int master_low_speed_update_index;
P1395BoardBits board_status_received;
P1395BoardBits board_com_ok;
//P1395BoardBits board_com_fault;
TYPE_CAN_PARAMETERS can_params;


// ---------- Pointers to CAN stucutres so that we can use CAN1 or CAN2
volatile unsigned int *CXEC_ptr;
volatile unsigned int *CXINTF_ptr;
volatile unsigned int *CXRX0CON_ptr;
volatile unsigned int *CXRX1CON_ptr;
volatile unsigned int *CXTX0CON_ptr;
volatile unsigned int *CXTX1CON_ptr;
volatile unsigned int *CXTX2CON_ptr;


// --------- Ram Structures that store the module status ---------- //
ETMCanBoardData local_data_ecb;
ETMCanBoardData mirror_hv_lambda;
ETMCanBoardData mirror_ion_pump;
ETMCanBoardData mirror_afc;
ETMCanBoardData mirror_cooling;
ETMCanBoardData mirror_htr_mag;
ETMCanBoardData mirror_gun_drv;
ETMCanBoardData mirror_pulse_mon;
ETMCanBoardData mirror_pulse_sync;


// ---------------- CAN Message Defines ---------------------- //
ETMCanSyncMessage                etm_can_master_sync_message;                // This is the sync message that the ECB sends out, only word zero ande one are used at this time




typedef struct {
  unsigned int reset_count;
  unsigned int can_timeout_count;
} PersistentData;

volatile PersistentData etm_can_persistent_data __attribute__ ((persistent));


void ETMCanMasterProcessMessage(void);

void ETMCanMasterCheckForTimeOut(void);



void ETMCanMasterTimedTransmit(void);
/*
  This schedules all of the Master commands (listed below)
  
*/




// These are the regularly scheduled commands that the ECB sends to sub boards
void ETMCanMasterSendSync();                          // This gets sent out 1 time every 50ms
void ETMCanMasterHVLambdaUpdateOutput(void);          // This gets sent out 1 time every 200ms
void ETMCanMasterHtrMagnetUpdateOutput(void);         // This gets sent out 1 time every 200ms 
void ETMCanMasterGunDriverUpdatePulseTop(void);       // This gets sent out 1 time every 200ms
void ETMCanMasterAFCUpdateHomeOffset(void);           // This gets sent out at 200ms / 6 = 1.2 Seconds
void ETMCanMasterGunDriverUpdateHeaterCathode(void);  // This gets sent out at 200ms / 6 = 1.2 Seconds
//void ETMCanMasterPulseSyncUpdateHighRegZero(void);    // This gets sent out at 200ms / 6 = 1.2 Seconds
//void ETMCanMasterPulseSyncUpdateHighRegOne(void);     // This gets sent out at 200ms / 6 = 1.2 Seconds
//void ETMCanMasterPulseSyncUpdateLowRegZero(void);     // This gets sent out at 200ms / 6 = 1.2 Seconds
//void ETMCanMasterPulseSyncUpdateLowRegOne(void);      // This gets sent out at 200ms / 6 = 1.2 Seconds

void ETMCanMasterPulseSyncUpdateMagneTXSetDose0(void);
void ETMCanMasterPulseSyncUpdateMagneTXSetDose1(void);
void ETMCanMasterPulseSyncUpdateMagneTXSetDoseAll(void);

// DPARKER how are the LEDs set on Pulse Sync Board?? Missing that command

void ETMCanMasterDataReturnFromSlave(ETMCanMessage* message_ptr);
/*
  This processes Return Commands (From slave board).
  This is used to return EEprom Data
*/


void ETMCanMasterUpdateSlaveStatus(ETMCanMessage* message_ptr);
/*
  This moves the data from the status message into the RAM copy on the master
  It also keeps track of which boards have sent a status message and clears TMR5 once all boards have reported a status message
*/

void ETMCanMasterProcessLogData(void);
/*
  This moves data from a log message into the RAM copy on the master
*/


void ETMCanMasterClearDebug(void);
/*
  This sets all the debug data to zero.
  This is usefull when debugging
  It is also required on power cycle because persistent variables will be set to random value
*/


void ETMCanMasterInitialize(unsigned int requested_can_port, unsigned long fcy, unsigned int etm_can_address, unsigned long can_operation_led, unsigned int can_interrupt_priority) {
  unsigned long timer_period_value;

  if (can_interrupt_priority > 7) {
    can_interrupt_priority = 7;
  }

  can_params.address = etm_can_address;
  can_params.led = can_operation_led;

  etm_can_persistent_data.reset_count++;
  
  _SYNC_CONTROL_WORD = 0;
  etm_can_master_sync_message.sync_1_ecb_state_for_fault_logic = 0;
  etm_can_master_sync_message.sync_2 = 0;
  etm_can_master_sync_message.sync_3 = 0;
  
  debug_data_ecb.reset_count = etm_can_persistent_data.reset_count;
  debug_data_ecb.can_timeout = etm_can_persistent_data.can_timeout_count;

  ETMCanBufferInitialize(&etm_can_master_rx_message_buffer);
  ETMCanBufferInitialize(&etm_can_master_tx_message_buffer);
  ETMCanBufferInitialize(&etm_can_master_rx_data_log_buffer);


  // Configure T4
  timer_period_value = fcy;
  timer_period_value >>= 8;
  timer_period_value /= T4_FREQUENCY_HZ;
  if (timer_period_value > 0xFFFF) {
    timer_period_value = 0xFFFF;
  }
  T4CON = T4CON_VALUE;
  PR4 = timer_period_value;  
  TMR4 = 0;
  _T4IF = 0;
  _T4IE = 0;
  T4CONbits.TON = 1;

  // Configure T5
  timer_period_value = fcy;
  timer_period_value >>= 8;
  timer_period_value /= T5_FREQUENCY_HZ;
  if (timer_period_value > 0xFFFF) {
    timer_period_value = 0xFFFF;
  }
  T5CON = T5CON_VALUE;
  PR5 = timer_period_value;
  TMR5 = 0;
  _T5IF = 0;
  _T5IE = 0;
  T5CONbits.TON = 1;

  ETMPinTrisOutput(can_params.led);
  

  if (requested_can_port != CAN_PORT_2) {
    // Use CAN1
    
    CXEC_ptr     = &C1EC;
    CXINTF_ptr   = &C1INTF;
    CXRX0CON_ptr = &C1RX0CON;
    CXRX1CON_ptr = &C1RX1CON;
    CXTX0CON_ptr = &C1TX0CON;
    CXTX1CON_ptr = &C1TX1CON;
    CXTX2CON_ptr = &C1TX2CON;
    
    
    _C1IE = 0;
    _C1IF = 0;
    _C1IP = can_interrupt_priority;
    
    C1INTF = 0;
    
    C1INTEbits.RX0IE = 1; // Enable RXB0 interrupt
    C1INTEbits.RX1IE = 1; // Enable RXB1 interrupt
    C1INTEbits.TX0IE = 1; // Enable TXB0 interrupt
    C1INTEbits.ERRIE = 1; // Enable Error interrupt

  
  
    // ---------------- Set up CAN Control Registers ---------------- //
  
    // Set Baud Rate
    C1CTRL = CXCTRL_CONFIG_MODE_VALUE;
    while(C1CTRLbits.OPMODE != 4);
  
    if (fcy == 25000000) {
      C1CFG1 = CXCFG1_25MHZ_FCY_VALUE;    
    } else if (fcy == 20000000) {
      C1CFG1 = CXCFG1_20MHZ_FCY_VALUE;    
    } else if (fcy == 10000000) {
      C1CFG1 = CXCFG1_10MHZ_FCY_VALUE;    
    } else {
      // If you got here we can't configure the can module
      // Try to set to operate in 10MHZ mode.
      // Can probably won't talk
      C1CFG1 = CXCFG1_10MHZ_FCY_VALUE;    
    }
    
    C1CFG2 = CXCFG2_VALUE;
    
    // Load Mask registers for RX0 and RX1
    C1RXM0SID = ETM_CAN_MASTER_RX0_MASK;
    C1RXM1SID = ETM_CAN_MASTER_RX1_MASK;
    
    // Load Filter registers
    C1RXF0SID = ETM_CAN_MASTER_MSG_FILTER_RF0;
    C1RXF1SID = ETM_CAN_MASTER_MSG_FILTER_RF1;
    C1RXF2SID = ETM_CAN_MASTER_MSG_FILTER_RF2;
    //C1RXF3SID = ETM_CAN_MSG_FILTER_OFF;
    //C1RXF4SID = ETM_CAN_MSG_FILTER_OFF;
    //C1RXF5SID = ETM_CAN_MSG_FILTER_OFF;

    // Set Transmitter Mode
    C1TX0CON = CXTXXCON_VALUE_LOW_PRIORITY;
    C1TX1CON = CXTXXCON_VALUE_MEDIUM_PRIORITY;
    C1TX2CON = CXTXXCON_VALUE_HIGH_PRIORITY;
    
    C1TX0DLC = CXTXXDLC_VALUE;
    C1TX1DLC = CXTXXDLC_VALUE;
    C1TX2DLC = CXTXXDLC_VALUE;

    // Set Receiver Mode
    C1RX0CON = CXRXXCON_VALUE;
    C1RX1CON = CXRXXCON_VALUE;
  
    // Switch to normal operation
    C1CTRL = CXCTRL_OPERATE_MODE_VALUE;
    while(C1CTRLbits.OPMODE != 0);
    
    //etm_can_ethernet_board_data.status_received_register = 0x0000;
    
    // Enable Can interrupt
    _C1IE = 1;
  } else {
    // Use CAN2
  }
}


void ETMCanMasterLoadConfiguration(unsigned long agile_id, unsigned int agile_dash, unsigned int agile_rev, unsigned int firmware_agile_rev, unsigned int firmware_branch, unsigned int firmware_branch_rev, unsigned int serial_number) {
  
  config_agile_number_low_word = (agile_id & 0xFFFF);
  agile_id >>= 16;
  config_agile_number_high_word = agile_id;
  config_agile_dash = agile_dash;
  config_agile_rev_ascii = agile_rev;

  config_firmware_agile_rev = firmware_agile_rev;
  config_firmware_branch = firmware_branch;
  config_firmware_branch_rev= firmware_branch_rev;
  config_serial_number = serial_number;
}

void ETMCanMasterSetSyncState(unsigned int state) {
  etm_can_master_sync_message.sync_1_ecb_state_for_fault_logic = state;
}


void ETMCanMasterDoCan(void) {
  ETMCanMasterProcessMessage();
  ETMCanMasterTimedTransmit();
  ETMCanMasterProcessLogData();
  ETMCanMasterCheckForTimeOut();
  if (_SYNC_CONTROL_CLEAR_DEBUG_DATA) {
    ETMCanMasterClearDebug();
  }

  if ((global_data_can_master.buffer_a_ready_to_send == 1) && (global_data_can_master.buffer_a_sent == 0)) {
    SendPulseData(SEND_BUFFER_A);
    global_data_can_master.buffer_a_sent = 1;
  }

  if ((global_data_can_master.buffer_b_ready_to_send == 1) && (global_data_can_master.buffer_b_sent == 0)) {
    SendPulseData(SEND_BUFFER_B);
    global_data_can_master.buffer_b_sent = 1;
  }

  // DPARKER put that in a function


  // Log Debugging Information
  // Record the RCON state
  debug_data_ecb.RCON_value = RCON;

  // Record the max TX counter
  if ((*CXEC_ptr & 0xFF00) > (debug_data_ecb.CXEC_reg_max & 0xFF00)) {
    debug_data_ecb.CXEC_reg_max &= 0x00FF;
    debug_data_ecb.CXEC_reg_max += (*CXEC_ptr & 0xFF00);
  }

  // Record the max RX counter
  if ((*CXEC_ptr & 0x00FF) > (debug_data_ecb.CXEC_reg_max & 0x00FF)) {
    debug_data_ecb.CXEC_reg_max &= 0xFF00;
    debug_data_ecb.CXEC_reg_max += (*CXEC_ptr & 0x00FF);
  }
}


void ETMCanMasterProcessMessage(void) {
  ETMCanMessage next_message;
  while (ETMCanBufferNotEmpty(&etm_can_master_rx_message_buffer)) {
    ETMCanReadMessageFromBuffer(&etm_can_master_rx_message_buffer, &next_message);
    //if ((next_message.identifier & ETM_CAN_MASTER_RX0_MASK) == ETM_CAN_MSG_RTN_RX) {
    if ((next_message.identifier & 0b1111111111000000) == ETM_CAN_MSG_RTN_RX) {
      ETMCanMasterDataReturnFromSlave(&next_message);
    } else if ((next_message.identifier & 0b1111111111000000) == ETM_CAN_MSG_STATUS_RX) {
      ETMCanMasterUpdateSlaveStatus(&next_message);
    } else {
      debug_data_ecb.can_unknown_msg_id++;
    } 
  }
  
  debug_data_ecb.can_tx_buf_overflow = etm_can_master_tx_message_buffer.message_overwrite_count;
  debug_data_ecb.can_rx_buf_overflow = etm_can_master_rx_message_buffer.message_overwrite_count;
  debug_data_ecb.can_rx_log_buf_overflow = etm_can_master_rx_data_log_buffer.message_overwrite_count;
}




void ETMCanMasterTimedTransmit(void) {
  /*
    One command is schedule to be sent every 25ms
    This loops through 8 times so each command is sent once every 200mS (5Hz)
    The sync command and Pulse Sync enable command are each sent twice for an effecive rate of 100ms (10Hz)
  */
  
  if ((previous_disable_xray_value == 0) && (_SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY)) {
    // We need to immediately send out a sync message
    ETMCanMasterSendSync();
  }
  
  
  if (_T4IF) {
    // should be true once every 25mS
    // each of the 8 cases will be true once every 200mS
    _T4IF = 0;

    if (!personality_loaded) {
      // Just send out a sync message
      ETMCanMasterSendSync();
    } else {
      master_high_speed_update_index++;
      master_high_speed_update_index &= 0x7;
      
      
      switch (master_high_speed_update_index) 
	{
	case 0x0:
	  // Send Sync Command (this is on TX1) - This also includes Pulse Sync Enable/Disable
	  ETMCanMasterSendSync();
	  break;
	  
	case 0x1:
	  // Send High/Low Energy Program voltage to Lambda Board
	  ETMCanMasterHVLambdaUpdateOutput();
	  break;
	  
	case 0x2:
	  // Send Sync Command (this is on TX1) - This also includes Pulse Sync Enable/Disable
	  ETMCanMasterSendSync();
	  break;
	  
	case 0x3:
	  // Send Heater/Magnet Current to Heater Magnet Board
	  ETMCanMasterHtrMagnetUpdateOutput();
	  break;
	  
	case 0x4:
	  // Send Sync Command (this is on TX1) - This also includes Pulse Sync Enable/Disable
	  ETMCanMasterSendSync();
	  break;
	  
	case 0x5:
	  // Send High/Low Energy Pulse top voltage to Gun Driver
	  ETMCanMasterGunDriverUpdatePulseTop();
	  break;
	  
	case 0x6:
	  // Send Sync Command (this is on TX1) - This also includes Pulse Sync Enable/Disable
	  ETMCanMasterSendSync();
	  break;
	  
	case 0x7:
	  // LOOP THROUGH SLOWER SPEED TRANSMITS
	  /*
	    THis is the following messages
	    Gun Driver Heater/Cathode
	    AFC Home/Offset
	    Pulse Sync Registers 1,2,3,4
	  */

	  master_low_speed_update_index++;
	  if (master_low_speed_update_index >= 6) {
	    master_low_speed_update_index = 0;
	  } 
	  
	  if (master_low_speed_update_index == 0) {
	    ETMCanMasterGunDriverUpdateHeaterCathode();  
	  }

	  if (master_low_speed_update_index == 1) {
	    ETMCanMasterAFCUpdateHomeOffset();	   
	  }

	  if (master_low_speed_update_index == 2) {
	    //ETMCanMasterPulseSyncUpdateHighRegZero();
	    ETMCanMasterPulseSyncUpdateMagneTXSetDose0();
	  }

	  if (master_low_speed_update_index == 3) {
	    //ETMCanMasterPulseSyncUpdateHighRegOne();
	    ETMCanMasterPulseSyncUpdateMagneTXSetDose1();
	  }

	  if (master_low_speed_update_index == 4) {
	    //ETMCanMasterPulseSyncUpdateLowRegZero();
	    ETMCanMasterPulseSyncUpdateMagneTXSetDoseAll();
	  }

	  if (master_low_speed_update_index == 5) {
	    //ETMCanMasterPulseSyncUpdateLowRegOne();	   
	  }
	  break;
	}
    }
  }
}




void ETMCanMasterSendSync(void) {
  ETMCanMessage sync_message;
  sync_message.identifier = ETM_CAN_MSG_SYNC_TX;
  sync_message.word0 = _SYNC_CONTROL_WORD;
  sync_message.word2 = etm_can_master_sync_message.sync_1_ecb_state_for_fault_logic;
  sync_message.word2 = etm_can_master_sync_message.sync_2;
  sync_message.word3 = etm_can_master_sync_message.sync_3;
  
  ETMCanTXMessage(&sync_message, CXTX1CON_ptr);
  debug_data_ecb.can_tx_1++;

  if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY) {
    previous_disable_xray_value = 1;
  } else {
    previous_disable_xray_value = 0;
  }
}

void ETMCanMasterHVLambdaUpdateOutput(void) {
  ETMCanMessage can_message;
  
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_HV_LAMBDA_BOARD << 2));
  can_message.word3 = ETM_CAN_REGISTER_HV_LAMBDA_SET_1_LAMBDA_SET_POINT;
  can_message.word2 = local_hvps_set_point_dose_1;
  can_message.word1 = local_hvps_set_point_dose_0;
  can_message.word0 = 0;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();  // DPARKER - Figure out how to build this into ETMCanAddMessageToBuffer()
}

void ETMCanMasterAFCUpdateHomeOffset(void) {
  ETMCanMessage can_message;
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_AFC_CONTROL_BOARD << 2));
  can_message.word3 = ETM_CAN_REGISTER_AFC_SET_1_HOME_POSITION_AND_OFFSET;
  can_message.word2 = local_afc_aft_control_voltage_dose_all;
  can_message.word1 = local_afc_aft_control_voltage_dose_all;
  can_message.word0 = local_afc_home_position_dose_0;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}

void ETMCanMasterHtrMagnetUpdateOutput(void) {
  ETMCanMessage can_message;
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_HEATER_MAGNET_BOARD << 2));
  can_message.word3 = ETM_CAN_REGISTER_HEATER_MAGNET_SET_1_CURRENT_SET_POINT;
  can_message.word2 = local_magnet_current_set_point_dose_1;
  can_message.word1 = local_heater_current_scaled_set_point;
  can_message.word0 = local_magnet_current_set_point_dose_0;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}

void ETMCanMasterGunDriverUpdatePulseTop(void) {
  ETMCanMessage can_message;
    can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_GUN_DRIVER_BOARD << 2));
  can_message.word3 = ETM_CAN_REGISTER_GUN_DRIVER_SET_1_GRID_TOP_SET_POINT;
  can_message.word2 = 0;
  can_message.word1 = local_gun_drv_top_v_dose_0;
  can_message.word0 = local_gun_drv_top_v_dose_1;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}

void ETMCanMasterGunDriverUpdateHeaterCathode(void) {
  ETMCanMessage can_message;
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_GUN_DRIVER_BOARD << 2));
  can_message.word3 = ETM_CAN_REGISTER_GUN_DRIVER_SET_1_HEATER_CATHODE_SET_POINT;
  can_message.word2 = 0;
  can_message.word1 = local_gun_drv_cathode_v_dose_0;
  can_message.word0 = local_gun_drv_heater_v_dose_all;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}


void ETMCanMasterPulseSyncUpdateMagneTXSetDose0(void) {
  ETMCanMessage can_message;
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_PULSE_SYNC_BOARD << 2));
  can_message.word3 = ETM_CAN_REGISTER_PULSE_SYNC_SET_1_MAGNETX_DOSE_0;
  can_message.word2 = local_pulse_sync_afc_trig_dose_0;
  can_message.word1 = local_pulse_sync_gun_trig_stop_max_dose_0;
  can_message.word0 = local_pulse_sync_gun_trig_start_max_dose_0;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}

void ETMCanMasterPulseSyncUpdateMagneTXSetDose1(void) {
  ETMCanMessage can_message;
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_PULSE_SYNC_BOARD << 2));
  can_message.word3 = ETM_CAN_REGISTER_PULSE_SYNC_SET_1_MAGNETX_DOSE_1;
  can_message.word2 = local_pulse_sync_afc_trig_dose_1;
  can_message.word1 = local_pulse_sync_gun_trig_stop_max_dose_1;
  can_message.word0 = local_pulse_sync_gun_trig_start_max_dose_1;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}
 
void ETMCanMasterPulseSyncUpdateMagneTXSetDoseAll(void) {
  ETMCanMessage can_message;
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_PULSE_SYNC_BOARD << 2));
  can_message.word3 = ETM_CAN_REGISTER_PULSE_SYNC_SET_1_MAGNETX_DOSE_ALL;
  can_message.word2 = local_pulse_sync_hvps_trig_start_dose_all;
  can_message.word1 = local_pulse_sync_pfn_trig_dose_all;
  can_message.word0 = local_pulse_sync_pulse_mon_trig_start_dose_all;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}



/*
void ETMCanMasterPulseSyncUpdateHighRegZero(void) {
  ETMCanMessage can_message;
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_PULSE_SYNC_BOARD << 2));
  can_message.word3 = ETM_CAN_REGISTER_PULSE_SYNC_SET_1_HIGH_ENERGY_TIMING_REG_0;
  can_message.word2 = *(unsigned int*)&psync_grid_start_high_intensity_3;
  can_message.word1 = *(unsigned int*)&psync_grid_start_high_intensity_1;
  can_message.word0 = *(unsigned int*)&psync_dose_delay_high;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}

void ETMCanMasterPulseSyncUpdateHighRegOne(void) {
  ETMCanMessage can_message;
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_PULSE_SYNC_BOARD << 2));
  can_message.word3 = ETM_CAN_REGISTER_PULSE_SYNC_SET_1_HIGH_ENERGY_TIMING_REG_1;
  can_message.word2 = *(unsigned int*)&psync_grid_stop_high_intensity_3;
  can_message.word1 = *(unsigned int*)&psync_grid_stop_high_intensity_1;
  can_message.word0 = *(unsigned int*)&psync_mag_delay_high;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}

void ETMCanMasterPulseSyncUpdateLowRegZero(void) {
  ETMCanMessage can_message;
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_PULSE_SYNC_BOARD << 2));
  can_message.word3 = ETM_CAN_REGISTER_PULSE_SYNC_SET_1_LOW_ENERGY_TIMING_REG_0;
  can_message.word2 = *(unsigned int*)&psync_grid_start_low_intensity_3;
  can_message.word1 = *(unsigned int*)&psync_grid_start_low_intensity_1;
  can_message.word0 = *(unsigned int*)&psync_dose_delay_low;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}

void ETMCanMasterPulseSyncUpdateLowRegOne(void) {
  ETMCanMessage can_message;
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_PULSE_SYNC_BOARD << 2));
  can_message.word3 = ETM_CAN_REGISTER_PULSE_SYNC_SET_1_LOW_ENERGY_TIMING_REG_1;
  can_message.word2 = *(unsigned int*)&psync_grid_stop_low_intensity_3;
  can_message.word1 = *(unsigned int*)&psync_grid_stop_low_intensity_1;
  can_message.word0 = *(unsigned int*)&psync_mag_delay_low;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}

*/



void ETMCanMasterDataReturnFromSlave(ETMCanMessage* message_ptr) {
  unsigned int index_word;
  index_word = message_ptr->word3 & 0x0FFF;
  
  if ((index_word >= 0x0100) & (index_word < 0x0200)) {
    // This is Calibration data that was read from the slave EEPROM
    SendCalibrationDataToGUI(message_ptr->word3, message_ptr->word1, message_ptr->word0);
  } else {
    // It was not a set value index 
    debug_data_ecb.can_invalid_index++;
  }
}



/*
  How to tell what boards are connected.

  When a status message is recieved, the the board_status_received register is updated.
  Every 250ms.  THe board_status_received register is checked.
  If a bit is not set & it is not ignored then a not connected error is generates


*/

void ETMCanMasterUpdateSlaveStatus(ETMCanMessage* message_ptr) {
  ETMCanStatusRegister status_message;
  unsigned int source_board;
  unsigned int message_bit;

  source_board = (message_ptr->identifier >> 2);
  source_board &= 0x000F;
  message_bit = 1 << source_board;
  
  status_message.control_notice_bits = *(ETMCanStatusRegisterControlAndNoticeBits*)&message_ptr->word0;
  status_message.fault_bits          = *(ETMCanStatusRegisterFaultBits*)&message_ptr->word1;
  status_message.warning_bits        = *(ETMCanStatusRegisterWarningBits*)&message_ptr->word2;
  status_message.not_logged_bits     = *(ETMCanStatusRegisterNotLoggedBits*)&message_ptr->word3;
  ClrWdt();

  switch (source_board) {
    /*
      Place all board specific status updates here
    */

  case ETM_CAN_ADDR_ION_PUMP_BOARD:
    UpdateSlaveEventLog(&mirror_ion_pump.status, &status_message, source_board);
    mirror_ion_pump.status = status_message;
    board_status_received.ion_pump_board = 1;
    break;

  case ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD:
    UpdateSlaveEventLog(&mirror_pulse_mon.status, &status_message, source_board);
    mirror_pulse_mon.status = status_message;
    board_status_received.magnetron_current_board = 1;
    break;

  case ETM_CAN_ADDR_PULSE_SYNC_BOARD:
    UpdateSlaveEventLog(&mirror_pulse_sync.status, &status_message, source_board);
    mirror_pulse_sync.status = status_message;
    board_status_received.pulse_sync_board = 1;
    break;

  case ETM_CAN_ADDR_HV_LAMBDA_BOARD:
    UpdateSlaveEventLog(&mirror_hv_lambda.status, &status_message, source_board);
    mirror_hv_lambda.status = status_message;
    board_status_received.hv_lambda_board = 1;
    break;

  case ETM_CAN_ADDR_AFC_CONTROL_BOARD:
    UpdateSlaveEventLog(&mirror_afc.status, &status_message, source_board);
    mirror_afc.status = status_message;
    board_status_received.afc_board = 1;
    break;
    
  case ETM_CAN_ADDR_COOLING_INTERFACE_BOARD:
    UpdateSlaveEventLog(&mirror_cooling.status, &status_message, source_board);
    mirror_cooling.status = status_message;
    board_status_received.cooling_interface_board = 1;
    break;

  case ETM_CAN_ADDR_HEATER_MAGNET_BOARD:
    UpdateSlaveEventLog(&mirror_htr_mag.status, &status_message, source_board);
    mirror_htr_mag.status = status_message;
    board_status_received.heater_magnet_board = 1;
    break;

  case ETM_CAN_ADDR_GUN_DRIVER_BOARD:
    UpdateSlaveEventLog(&mirror_gun_drv.status, &status_message, source_board);
    mirror_gun_drv.status = status_message;
    board_status_received.gun_driver_board = 1;
    break;
    
    
  default:
    debug_data_ecb.can_address_error++;
    break;
  }
  /*
#ifdef __IGNORE_ION_PUMP_MODULE
  board_status_received.ion_pump_board = 1;
#endif
  
#ifdef __IGNORE_AFC_MODULE
  board_status_received.afc_board = 1;
#endif
  
#ifdef __IGNORE_GUN_DRIVER_MODULE
  board_status_received.gun_driver_board = 1;
#endif

#ifdef __IGNORE_COOLING_INTERFACE_MODULE
  board_status_received.cooling_interface_board = 1;
#endif

#ifdef __IGNORE_HEATER_MAGNET_MODULE
  board_status_received.heater_magnet_board = 1;
#endif
    
#ifdef __IGNORE_HV_LAMBDA_MODULE
  board_status_received.hv_lambda_board = 1;
#endif

#ifdef __IGNORE_PULSE_CURRENT_MODULE
  board_status_received.magnetron_current_board = 1;
#endif
    
#ifdef __IGNORE_PULSE_SYNC_MODULE
  board_status_received.pulse_sync_board = 1;
#endif
    
    
  // Figure out if all the boards are connected
  all_boards_connected = 1;

  //#ifndef __IGNORE_ION_PUMP_MODULE
  if (!board_status_received.ion_pump_board) {
    all_boards_connected = 0;
  }
  //#endif
  
  
  //#ifndef __IGNORE_PULSE_CURRENT_MODULE
  if (!board_status_received.magnetron_current_board) {
    all_boards_connected = 0;
  }
  //#endif
  
  //#ifndef __IGNORE_PULSE_SYNC_MODULE
  if (!board_status_received.pulse_sync_board) {
    all_boards_connected = 0;
  }
  //#endif
  
  //#ifndef __IGNORE_HV_LAMBDA_MODULE
  if (!board_status_received.hv_lambda_board) {
    all_boards_connected = 0;
  }
  //#endif
  
  //#ifndef __IGNORE_AFC_MODULE
  if (!board_status_received.afc_board) {
    all_boards_connected = 0;
  }
  //#endif
  
  //#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  if (!board_status_received.cooling_interface_board) {
    all_boards_connected = 0;
  }
  //#endif
  
  //#ifndef __IGNORE_HEATER_MAGNET_MODULE
  if (!board_status_received.heater_magnet_board) {
    all_boards_connected = 0;
  }
  //#endif
  
  //#ifndef __IGNORE_GUN_DRIVER_MODULE
  if (!board_status_received.gun_driver_board) {
    all_boards_connected = 0;
  }
  //#endif
  
  if (all_boards_connected) {
    // Clear the status received register
    *(unsigned int*)&board_status_received = 0x0000; 
    *(unsigned int*)&board_com_fault = 0x0000;
    // Reset T5 to start the next timer cycle
    TMR5 = 0;
  } 
*/

}


void UpdateSlaveEventLog(ETMCanStatusRegister* previous_status, ETMCanStatusRegister* current_status, unsigned int slave_select) {
  unsigned int log_id;

  log_id = 0xC000;
  log_id += slave_select << 8;

  // First update the control_notice_bits
  if ((*(unsigned int*)&previous_status->control_notice_bits) != (*(unsigned int*)&current_status->control_notice_bits)) {
    //update based on changes
    if (previous_status->control_notice_bits.control_not_ready != current_status->control_notice_bits.control_not_ready) {
      if (current_status->control_notice_bits.control_not_ready) {
	SendToEventLog(log_id + 0x00);
      } else {
	SendToEventLog(log_id + 0x08);
      }
    }

    if (previous_status->control_notice_bits.control_not_configured != current_status->control_notice_bits.control_not_configured) {
      if (current_status->control_notice_bits.control_not_configured) {
	SendToEventLog(log_id + 0x01);
      } else {
	SendToEventLog(log_id + 0x09);
      }
    }

    if (previous_status->control_notice_bits.control_self_check_error != current_status->control_notice_bits.control_self_check_error) {
      if (current_status->control_notice_bits.control_self_check_error) {
	SendToEventLog(log_id + 0x02);
      } else {
	SendToEventLog(log_id + 0x0A);
      }
    }

    if (previous_status->control_notice_bits.control_3_unused != current_status->control_notice_bits.control_3_unused) {
      if (current_status->control_notice_bits.control_3_unused) {
	SendToEventLog(log_id + 0x03);
      } else {
	SendToEventLog(log_id + 0x0B);
      }
    }

    if (previous_status->control_notice_bits.control_4_unused != current_status->control_notice_bits.control_4_unused) {
      if (current_status->control_notice_bits.control_4_unused) {
	SendToEventLog(log_id + 0x04);
      } else {
	SendToEventLog(log_id + 0x0C);
      }
    }

    if (previous_status->control_notice_bits.control_5_unused != current_status->control_notice_bits.control_5_unused) {
      if (current_status->control_notice_bits.control_5_unused) {
	SendToEventLog(log_id + 0x05);
      } else {
	SendToEventLog(log_id + 0x0D);
      }
    }

    if (previous_status->control_notice_bits.control_6_unused != current_status->control_notice_bits.control_6_unused) {
      if (current_status->control_notice_bits.control_6_unused) {
	SendToEventLog(log_id + 0x06);
      } else {
	SendToEventLog(log_id + 0x0E);
      }
    }

    if (previous_status->control_notice_bits.control_7_unused != current_status->control_notice_bits.control_7_unused) {
      if (current_status->control_notice_bits.control_7_unused) {
	SendToEventLog(log_id + 0x07);
      } else {
	SendToEventLog(log_id + 0x0F);
      }
    }

    if (current_status->control_notice_bits.notice_0) {
      SendToEventLog(log_id + 0x10);
      current_status->control_notice_bits.notice_0 = 0;
    }

    if (current_status->control_notice_bits.notice_1) {
      SendToEventLog(log_id + 0x11);
      current_status->control_notice_bits.notice_1 = 0;
    }

    if (current_status->control_notice_bits.notice_2) {
      SendToEventLog(log_id + 0x12);
      current_status->control_notice_bits.notice_2 = 0;
    }

    if (current_status->control_notice_bits.notice_3) {
      SendToEventLog(log_id + 0x13);
      current_status->control_notice_bits.notice_3 = 0;
    }

    if (current_status->control_notice_bits.notice_4) {
      SendToEventLog(log_id + 0x14);
      current_status->control_notice_bits.notice_4 = 0;
    }

    if (current_status->control_notice_bits.notice_5) {
      SendToEventLog(log_id + 0x15);
      current_status->control_notice_bits.notice_5 = 0;
    }

    if (current_status->control_notice_bits.notice_6) {
      SendToEventLog(log_id + 0x16);
      current_status->control_notice_bits.notice_6 = 0;
    }

    if (current_status->control_notice_bits.notice_7) {
      SendToEventLog(log_id + 0x17);
      current_status->control_notice_bits.notice_7 = 0;
    }
  }
  
  
  if ((*(unsigned int*)&previous_status->fault_bits) != (*(unsigned int*)&current_status->fault_bits)) {
    
    if (previous_status->fault_bits.fault_0 != current_status->fault_bits.fault_0) {
      if (current_status->fault_bits.fault_0) {
	SendToEventLog(log_id + 0x20);
      }	else {
	SendToEventLog(log_id + 0x30);
      }
    }
    
    if (previous_status->fault_bits.fault_1 != current_status->fault_bits.fault_1) {
      if (current_status->fault_bits.fault_1) {
	SendToEventLog(log_id + 0x21);
      }	else {
	SendToEventLog(log_id + 0x31);
      }
    }

    if (previous_status->fault_bits.fault_2 != current_status->fault_bits.fault_2) {
      if (current_status->fault_bits.fault_2) {
	SendToEventLog(log_id + 0x22);
      }	else {
	SendToEventLog(log_id + 0x32);
      }
    }

    if (previous_status->fault_bits.fault_3 != current_status->fault_bits.fault_3) {
      if (current_status->fault_bits.fault_3) {
	SendToEventLog(log_id + 0x23);
      }	else {
	SendToEventLog(log_id + 0x33);
      }
    }

    if (previous_status->fault_bits.fault_4 != current_status->fault_bits.fault_4) {
      if (current_status->fault_bits.fault_4) {
	SendToEventLog(log_id + 0x24);
      }	else {
	SendToEventLog(log_id + 0x34);
      }
    }

    if (previous_status->fault_bits.fault_5 != current_status->fault_bits.fault_5) {
      if (current_status->fault_bits.fault_5) {
	SendToEventLog(log_id + 0x25);
      }	else {
	SendToEventLog(log_id + 0x35);
      }
    }

    if (previous_status->fault_bits.fault_6 != current_status->fault_bits.fault_6) {
      if (current_status->fault_bits.fault_6) {
	SendToEventLog(log_id + 0x26);
      }	else {
	SendToEventLog(log_id + 0x36);
      }
    }

    if (previous_status->fault_bits.fault_7 != current_status->fault_bits.fault_7) {
      if (current_status->fault_bits.fault_7) {
	SendToEventLog(log_id + 0x27);
      }	else {
	SendToEventLog(log_id + 0x37);
      }
    }

    if (previous_status->fault_bits.fault_8 != current_status->fault_bits.fault_8) {
      if (current_status->fault_bits.fault_8) {
	SendToEventLog(log_id + 0x28);
      }	else {
	SendToEventLog(log_id + 0x38);
      }
    }

    if (previous_status->fault_bits.fault_9 != current_status->fault_bits.fault_9) {
      if (current_status->fault_bits.fault_9) {
	SendToEventLog(log_id + 0x29);
      }	else {
	SendToEventLog(log_id + 0x39);
      }
    }

    if (previous_status->fault_bits.fault_A != current_status->fault_bits.fault_A) {
      if (current_status->fault_bits.fault_A) {
	SendToEventLog(log_id + 0x2A);
      }	else {
	SendToEventLog(log_id + 0x3A);
      }
    }

    if (previous_status->fault_bits.fault_B != current_status->fault_bits.fault_B) {
      if (current_status->fault_bits.fault_B) {
	SendToEventLog(log_id + 0x2B);
      }	else {
	SendToEventLog(log_id + 0x3B);
      }
    }

    if (previous_status->fault_bits.fault_C != current_status->fault_bits.fault_C) {
      if (current_status->fault_bits.fault_C) {
	SendToEventLog(log_id + 0x2C);
      }	else {
	SendToEventLog(log_id + 0x3C);
      }
    }

    if (previous_status->fault_bits.fault_D != current_status->fault_bits.fault_D) {
      if (current_status->fault_bits.fault_D) {
	SendToEventLog(log_id + 0x2D);
      }	else {
	SendToEventLog(log_id + 0x3D);
      }
    }

    if (previous_status->fault_bits.fault_E != current_status->fault_bits.fault_E) {
      if (current_status->fault_bits.fault_E) {
	SendToEventLog(log_id + 0x2E);
      }	else {
	SendToEventLog(log_id + 0x3E);
      }
    }

    if (previous_status->fault_bits.fault_F != current_status->fault_bits.fault_F) {
      if (current_status->fault_bits.fault_F) {
	SendToEventLog(log_id + 0x2F);
      }	else {
	SendToEventLog(log_id + 0x3F);
      }
    }
  }
  

  if ((*(unsigned int*)&previous_status->warning_bits) != (*(unsigned int*)&current_status->warning_bits)) {

    if (previous_status->warning_bits.warning_0 != current_status->warning_bits.warning_0) {
      if (current_status->warning_bits.warning_0) {
	SendToEventLog(log_id + 0x40);
      }	else {
	SendToEventLog(log_id + 0x50);
      }
    }

    if (previous_status->warning_bits.warning_1 != current_status->warning_bits.warning_1) {
      if (current_status->warning_bits.warning_1) {
	SendToEventLog(log_id + 0x41);
      }	else {
	SendToEventLog(log_id + 0x51);
      }
    }

    if (previous_status->warning_bits.warning_2 != current_status->warning_bits.warning_2) {
      if (current_status->warning_bits.warning_2) {
	SendToEventLog(log_id + 0x42);
      }	else {
	SendToEventLog(log_id + 0x52);
      }
    }

    if (previous_status->warning_bits.warning_3 != current_status->warning_bits.warning_3) {
      if (current_status->warning_bits.warning_3) {
	SendToEventLog(log_id + 0x43);
      }	else {
	SendToEventLog(log_id + 0x53);
      }
    }

    if (previous_status->warning_bits.warning_4 != current_status->warning_bits.warning_4) {
      if (current_status->warning_bits.warning_4) {
	SendToEventLog(log_id + 0x44);
      }	else {
	SendToEventLog(log_id + 0x54);
      }
    }

    if (previous_status->warning_bits.warning_5 != current_status->warning_bits.warning_5) {
      if (current_status->warning_bits.warning_5) {
	SendToEventLog(log_id + 0x45);
      }	else {
	SendToEventLog(log_id + 0x55);
      }
    }

    if (previous_status->warning_bits.warning_6 != current_status->warning_bits.warning_6) {
      if (current_status->warning_bits.warning_6) {
	SendToEventLog(log_id + 0x46);
      }	else {
	SendToEventLog(log_id + 0x56);
      }
    }

    if (previous_status->warning_bits.warning_7 != current_status->warning_bits.warning_7) {
      if (current_status->warning_bits.warning_7) {
	SendToEventLog(log_id + 0x47);
      }	else {
	SendToEventLog(log_id + 0x57);
      }
    }

    if (previous_status->warning_bits.warning_8 != current_status->warning_bits.warning_8) {
      if (current_status->warning_bits.warning_8) {
	SendToEventLog(log_id + 0x48);
      }	else {
	SendToEventLog(log_id + 0x58);
      }
    }

    if (previous_status->warning_bits.warning_9 != current_status->warning_bits.warning_9) {
      if (current_status->warning_bits.warning_9) {
	SendToEventLog(log_id + 0x49);
      }	else {
	SendToEventLog(log_id + 0x59);
      }
    }

    if (previous_status->warning_bits.warning_A != current_status->warning_bits.warning_A) {
      if (current_status->warning_bits.warning_A) {
	SendToEventLog(log_id + 0x4A);
      }	else {
	SendToEventLog(log_id + 0x5A);
      }
    }

    if (previous_status->warning_bits.warning_B != current_status->warning_bits.warning_B) {
      if (current_status->warning_bits.warning_B) {
	SendToEventLog(log_id + 0x4B);
      }	else {
	SendToEventLog(log_id + 0x5B);
      }
    }

if (previous_status->warning_bits.warning_C != current_status->warning_bits.warning_C) {
      if (current_status->warning_bits.warning_C) {
	SendToEventLog(log_id + 0x4C);
      }	else {
	SendToEventLog(log_id + 0x5C);
      }
    }

    if (previous_status->warning_bits.warning_D != current_status->warning_bits.warning_D) {
      if (current_status->warning_bits.warning_D) {
	SendToEventLog(log_id + 0x4D);
      }	else {
	SendToEventLog(log_id + 0x5D);
      }
    }

    if (previous_status->warning_bits.warning_E != current_status->warning_bits.warning_E) {
      if (current_status->warning_bits.warning_E) {
	SendToEventLog(log_id + 0x4E);
      }	else {
	SendToEventLog(log_id + 0x5E);
      }
    }

    if (previous_status->warning_bits.warning_F != current_status->warning_bits.warning_F) {
      if (current_status->warning_bits.warning_F) {
	SendToEventLog(log_id + 0x4F);
      }	else {
	SendToEventLog(log_id + 0x5F);
      }
    }
  }
}




void ETMCanMasterProcessLogData(void) {
  ETMCanMessage          next_message;
  unsigned int           data_log_index;
  unsigned int           board_id;
  unsigned int           log_id;

  ETMCanBoardData*       board_data_ptr;

  unsigned int           fast_log_buffer_index;
  ETMCanHighSpeedData*   ptr_high_speed_data;


  while (ETMCanBufferNotEmpty(&etm_can_master_rx_data_log_buffer)) {
    ETMCanReadMessageFromBuffer(&etm_can_master_rx_data_log_buffer, &next_message);
    data_log_index = next_message.identifier;
    data_log_index >>= 2;
    data_log_index &= 0x03FF;
    board_id = data_log_index & 0x000F;
    log_id = data_log_index & 0x03F0;



    if (log_id <= 0x03F) {
      // It is high speed logging data that must be handled manually
      // It is board specific logging data
      
      // Figure out where to store high speed logging data (this will only be used if it IS high speed data logging)
      // But I'm going to go ahead and calculate where to store it for all messages
      fast_log_buffer_index = next_message.word3 & 0x000F;
      if (next_message.word3 & 0x0010) {
	ptr_high_speed_data = &high_speed_data_buffer_a[fast_log_buffer_index];
      } else {
	ptr_high_speed_data = &high_speed_data_buffer_b[fast_log_buffer_index];
      }
      
      switch (data_log_index) 
	{
 	case ETM_CAN_DATA_LOG_REGISTER_HV_LAMBDA_FAST_LOG_0:
	  // Update the high speed data table
	  ptr_high_speed_data->hvlambda_vmon_at_eoc_period = next_message.word2;
	  ptr_high_speed_data->hvlambda_vmon_at_trigger = next_message.word1;
	  ptr_high_speed_data->hvlambda_vpeak_at_eoc_period = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_AFC_FAST_LOG_0:
	  ptr_high_speed_data->afc_readback_target_position = next_message.word1;
	  ptr_high_speed_data->afc_readback_filtered_error_reading = next_message.word2;

	  //ptr_high_speed_data->afc_readback_current_position = next_message.word2;
	  //ptr_high_speed_data->afc_readback_target_position = next_message.word1;
	  // unused word 0
	  break;
	  
	case ETM_CAN_DATA_LOG_REGISTER_AFC_FAST_LOG_1:
	  ptr_high_speed_data->afc_readback_current_position = next_message.word2;
	  ptr_high_speed_data->afc_readback_a_input = next_message.word1;
	  ptr_high_speed_data->afc_readback_b_input = next_message.word0;
	  
	  //ptr_high_speed_data->afc_readback_a_input = next_message.word2;
	  //ptr_high_speed_data->afc_readback_b_input = next_message.word1;
	  //ptr_high_speed_data->afc_readback_filtered_error_reading = next_message.word0;
	  break;
	  
	case ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_FAST_LOG_0:
	  ptr_high_speed_data->magmon_external_adc_reading = next_message.word2;
	  ptr_high_speed_data->magmon_internal_adc_reading = next_message.word1;
	  if (next_message.word0) {
	    ptr_high_speed_data->status_bits.arc_this_pulse = 1;
	  }
	  break;
	  
	case ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_FAST_LOG_0:
	  ptr_high_speed_data->psync_trigger_width_and_filtered_trigger_width = next_message.word2;
	  ptr_high_speed_data->psync_grid_width_and_delay = next_message.word1;
	  ptr_high_speed_data->psync_period = next_message.word0;
	  break;
	  
	default:
	  debug_data_ecb.can_unknown_msg_id++;
	  break;
	}
    } else if (log_id >= 0x100) {
      // It is debugging information, load into the common debugging register if that board is actively being debugged
      if (board_id == etm_can_active_debugging_board_id) {
	switch (log_id) 
	  {
	  case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_0:
	    debug_data_slave_mirror.debug_reg[0]            = next_message.word3;
	    debug_data_slave_mirror.debug_reg[1]            = next_message.word2;
	    debug_data_slave_mirror.debug_reg[2]            = next_message.word1;
	    debug_data_slave_mirror.debug_reg[3]            = next_message.word0;
	    break;
	    
	  case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_1:
	    debug_data_slave_mirror.debug_reg[4]            = next_message.word3;
	    debug_data_slave_mirror.debug_reg[5]            = next_message.word2;
	    debug_data_slave_mirror.debug_reg[6]            = next_message.word1;
	    debug_data_slave_mirror.debug_reg[7]            = next_message.word0;
	    break;

	  case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_2:
	    debug_data_slave_mirror.debug_reg[8]            = next_message.word3;
	    debug_data_slave_mirror.debug_reg[9]            = next_message.word2;
	    debug_data_slave_mirror.debug_reg[10]           = next_message.word1;
	    debug_data_slave_mirror.debug_reg[11]           = next_message.word0;
	    break;

	  case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_3:
	    debug_data_slave_mirror.debug_reg[12]           = next_message.word3;
	    debug_data_slave_mirror.debug_reg[13]           = next_message.word2;
	    debug_data_slave_mirror.debug_reg[14]           = next_message.word1;
	    debug_data_slave_mirror.debug_reg[15]           = next_message.word0;
	    break;

	  case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_0:
	    debug_data_slave_mirror.can_tx_0                = next_message.word3;
	    debug_data_slave_mirror.can_tx_1                = next_message.word2;
	    debug_data_slave_mirror.can_tx_2                = next_message.word1;
	    debug_data_slave_mirror.CXEC_reg_max            = next_message.word0;
	    break;

	  case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_1:
	    debug_data_slave_mirror.can_rx_0_filt_0         = next_message.word3;
	    debug_data_slave_mirror.can_rx_0_filt_1         = next_message.word2;
	    debug_data_slave_mirror.can_rx_1_filt_2         = next_message.word1;
	    debug_data_slave_mirror.CXINTF_max              = next_message.word0;
	    break;

	  case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_2:
	    debug_data_slave_mirror.can_unknown_msg_id      = next_message.word3;
	    debug_data_slave_mirror.can_invalid_index       = next_message.word2;
	    debug_data_slave_mirror.can_address_error       = next_message.word1;
	    debug_data_slave_mirror.can_error_flag          = next_message.word0;
	    break;

	  case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_3:
	    debug_data_slave_mirror.can_tx_buf_overflow     = next_message.word3;
	    debug_data_slave_mirror.can_rx_buf_overflow     = next_message.word2;
	    debug_data_slave_mirror.can_rx_log_buf_overflow = next_message.word1;
	    debug_data_slave_mirror.can_timeout             = next_message.word0;
	    break;

	  case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_SYSTEM_ERROR_0:
	    debug_data_slave_mirror.reset_count             = next_message.word3;
	    debug_data_slave_mirror.RCON_value              = next_message.word2;
	    debug_data_slave_mirror.reserved_1              = next_message.word1;
	    debug_data_slave_mirror.reserved_0              = next_message.word0;
	    break;

	  case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_SYSTEM_ERROR_1:
	    debug_data_slave_mirror.i2c_bus_error_count     = next_message.word3;
	    debug_data_slave_mirror.spi_bus_error_count     = next_message.word2;
	    debug_data_slave_mirror.scale_error_count       = next_message.word1;
	    debug_data_slave_mirror.self_test_results       = *(ETMCanSelfTestRegister*)&next_message.word0;
	    break;
	    
	  default:
	    debug_data_ecb.can_unknown_msg_id++;
	    break;
	  }
      }
    } else {
      // It is data that needs to be stored for a specific board. 

      // First figure out which board the data is from
      switch (board_id) 
	{
	case ETM_CAN_ADDR_ION_PUMP_BOARD:
	  board_data_ptr = &mirror_ion_pump;
	  break;
	  
	case ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD:
	  board_data_ptr = &mirror_pulse_mon;
	  break;

	case ETM_CAN_ADDR_PULSE_SYNC_BOARD:
	  board_data_ptr = &mirror_pulse_sync;
	  break;
	  
	case ETM_CAN_ADDR_HV_LAMBDA_BOARD:
	  board_data_ptr = &mirror_hv_lambda;
	  break;
	  
	case ETM_CAN_ADDR_AFC_CONTROL_BOARD:
	  board_data_ptr = &mirror_afc;
	  break;

	case ETM_CAN_ADDR_COOLING_INTERFACE_BOARD:
	  board_data_ptr = &mirror_cooling;
	  break;
	  
	case ETM_CAN_ADDR_HEATER_MAGNET_BOARD:
	  board_data_ptr = &mirror_htr_mag;
	  break;

	case ETM_CAN_ADDR_GUN_DRIVER_BOARD:
	  board_data_ptr = &mirror_gun_drv;
	  break;

	default:
	  debug_data_ecb.can_address_error++;
	  break;
	  
	}
      
      // Now figure out which data log it is
      switch (log_id)
	{
	case ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_0:
	  board_data_ptr->log_data[0]  = next_message.word0;
	  board_data_ptr->log_data[1]  = next_message.word1;
	  board_data_ptr->log_data[2]  = next_message.word2;
	  board_data_ptr->log_data[3]  = next_message.word3;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_1:
	  board_data_ptr->log_data[4]  = next_message.word0;
	  board_data_ptr->log_data[5]  = next_message.word1;
	  board_data_ptr->log_data[6]  = next_message.word2;
	  board_data_ptr->log_data[7]  = next_message.word3;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_2:
	  board_data_ptr->log_data[8]  = next_message.word0;
	  board_data_ptr->log_data[9]  = next_message.word1;
	  board_data_ptr->log_data[10] = next_message.word2;
	  board_data_ptr->log_data[11] = next_message.word3;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_3:
	  board_data_ptr->log_data[12] = next_message.word0;
	  board_data_ptr->log_data[13] = next_message.word1;
	  board_data_ptr->log_data[14] = next_message.word2;
	  board_data_ptr->log_data[15] = next_message.word3;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_4:
	  board_data_ptr->log_data[16] = next_message.word0;
	  board_data_ptr->log_data[17] = next_message.word1;
	  board_data_ptr->log_data[18] = next_message.word2;
	  board_data_ptr->log_data[19] = next_message.word3;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_5:
	  board_data_ptr->log_data[20] = next_message.word0;
	  board_data_ptr->log_data[21] = next_message.word1;
	  board_data_ptr->log_data[22] = next_message.word2;
	  board_data_ptr->log_data[23] = next_message.word3;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CONFIG_0:
	  board_data_ptr->config_data[0] = next_message.word0;
	  board_data_ptr->config_data[1] = next_message.word1;
	  board_data_ptr->config_data[2] = next_message.word2;
	  board_data_ptr->config_data[3] = next_message.word3;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CONFIG_1:
	  board_data_ptr->config_data[4] = next_message.word0;
	  board_data_ptr->config_data[5] = next_message.word1;
	  board_data_ptr->config_data[6] = next_message.word2;
	  board_data_ptr->config_data[7] = next_message.word3;
	  break;
	  
	default:
	  debug_data_ecb.can_unknown_msg_id++;
	  break;
	} 
    }
  }
}


void ETMCanMasterCheckForTimeOut(void) {
  
  if (_T5IF) {
    // Update this once every 250mS
    if (_LATG13) {
      _LATG13 = 0;
    } else {
      _LATG13 = 1;
    }
    _T5IF = 0;
    //TMR5 = 0;

    if (board_status_received.ion_pump_board != board_com_ok.ion_pump_board) {
      if (board_status_received.ion_pump_board) {
      // The board just connected
	SendToEventLog(LOG_ID_CONNECTED_ION_PUMP_BOARD);	
      } else {
      // The board just lost connection
	SendToEventLog(LOG_ID_NOT_CONNECTED_ION_PUMP_BOARD);
      }
    }

    if (board_status_received.magnetron_current_board != board_com_ok.magnetron_current_board) {
      if (board_status_received.magnetron_current_board) {
      // The board just connected
	SendToEventLog(LOG_ID_CONNECTED_MAGNETRON_CURRENT_BOARD);	
      } else {
      // The board just lost connection
	SendToEventLog(LOG_ID_NOT_CONNECTED_MAGNETRON_CURRENT_BOARD);
      }
    }

    if (board_status_received.pulse_sync_board != board_com_ok.pulse_sync_board) {
      if (board_status_received.pulse_sync_board) {
      // The board just connected
	SendToEventLog(LOG_ID_CONNECTED_PULSE_SYNC_BOARD);	
      } else {
      // The board just lost connection
	SendToEventLog(LOG_ID_NOT_CONNECTED_PULSE_SYNC_BOARD);
      }
    }

    if (board_status_received.hv_lambda_board != board_com_ok.hv_lambda_board) {
      if (board_status_received.hv_lambda_board) {
      // The board just connected
	SendToEventLog(LOG_ID_CONNECTED_HV_LAMBDA_BOARD);	
      } else {
      // The board just lost connection
	SendToEventLog(LOG_ID_NOT_CONNECTED_HV_LAMBDA_BOARD);
      }
    }

    if (board_status_received.afc_board != board_com_ok.afc_board) {
      if (board_status_received.afc_board) {
      // The board just connected
	SendToEventLog(LOG_ID_CONNECTED_AFC_BOARD);	
      } else {
      // The board just lost connection
	SendToEventLog(LOG_ID_NOT_CONNECTED_AFC_BOARD);
      }
    }

    if (board_status_received.cooling_interface_board != board_com_ok.cooling_interface_board) {
      if (board_status_received.cooling_interface_board) {
      // The board just connected
	SendToEventLog(LOG_ID_CONNECTED_COOLING_BOARD);	
      } else {
      // The board just lost connection
	SendToEventLog(LOG_ID_NOT_CONNECTED_COOLING_BOARD);
      }
    }

    if (board_status_received.heater_magnet_board != board_com_ok.heater_magnet_board) {
      if (board_status_received.heater_magnet_board) {
      // The board just connected
	SendToEventLog(LOG_ID_CONNECTED_HEATER_MAGNET_BOARD);	
      } else {
      // The board just lost connection
	SendToEventLog(LOG_ID_NOT_CONNECTED_HEATER_MAGNET_BOARD);
      }
    }

    if (board_status_received.gun_driver_board != board_com_ok.gun_driver_board) {
      if (board_status_received.gun_driver_board) {
      // The board just connected
	SendToEventLog(LOG_ID_CONNECTED_GUN_DRIVER_BOARD);	
      } else {
      // The board just lost connection
	SendToEventLog(LOG_ID_NOT_CONNECTED_GUN_DRIVER_BOARD);
      }
    }
    
    board_com_ok = board_status_received;
    
#ifdef __IGNORE_ION_PUMP_MODULE
    board_status_received.ion_pump_board = 1;
#endif
    
#ifdef __IGNORE_AFC_MODULE
    board_status_received.afc_board = 1;
#endif
    
#ifdef __IGNORE_GUN_DRIVER_MODULE
    board_status_received.gun_driver_board = 1;
#endif
    
#ifdef __IGNORE_COOLING_INTERFACE_MODULE
    board_status_received.cooling_interface_board = 1;
#endif
    
#ifdef __IGNORE_HEATER_MAGNET_MODULE
    board_status_received.heater_magnet_board = 1;
#endif
    
#ifdef __IGNORE_HV_LAMBDA_MODULE
    board_status_received.hv_lambda_board = 1;
#endif
    
#ifdef __IGNORE_PULSE_CURRENT_MODULE
    board_status_received.magnetron_current_board = 1;
#endif
    
#ifdef __IGNORE_PULSE_SYNC_MODULE
    board_status_received.pulse_sync_board = 1;
#endif

    if (((*(unsigned int*)&board_status_received) & 0b0000000111111110) != 0b0000000111111110) {
      debug_data_ecb.can_timeout++;
      etm_can_persistent_data.can_timeout_count = debug_data_ecb.can_timeout;
    }

    *(unsigned int*)&board_status_received = 0x0000;
    
  }
}


void SendCalibrationSetPointToSlave(unsigned int index, unsigned int data_1, unsigned int data_0) {
  ETMCanMessage can_message;
  unsigned int board_id;
  board_id = index & 0xF000;
  board_id >>= 12;
  
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (board_id << 2));
  can_message.word3 = index;
  can_message.word2 = 0;
  can_message.word1 = data_1;
  can_message.word0 = data_0;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}

void ReadCalibrationSetPointFromSlave(unsigned int index) {
  ETMCanMessage can_message;
  unsigned int board_id;
  board_id = index & 0xF000;
  board_id >>= 12;
  
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (board_id << 2));
  can_message.word3 = index;
  can_message.word2 = 0;
  can_message.word1 = 0;
  can_message.word0 = 0;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}

void SendSlaveLoadDefaultEEpromData(unsigned int board_id) {
  ETMCanMessage can_message;
  board_id &= 0x000F;
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (board_id << 2));
  can_message.word3 = (board_id << 12) + ETM_CAN_REGISTER_DEFAULT_CMD_RESET_ANALOG_CALIBRATION;
  can_message.word2 = 0;
  can_message.word1 = 0;
  can_message.word0 = 0;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}

void SendSlaveReset(unsigned int board_id) {
  ETMCanMessage can_message;
  board_id &= 0x000F;
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (board_id << 2));
  can_message.word3 = (board_id << 12) + ETM_CAN_REGISTER_DEFAULT_CMD_RESET_MCU;
  can_message.word2 = 0;
  can_message.word1 = 0;
  can_message.word0 = 0;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}

void ETMCanMasterClearHighSpeedLogging(void) {
  unsigned int n;
  n = 0;
  while (n < HIGH_SPEED_DATA_BUFFER_SIZE) {
    high_speed_data_buffer_a[n].pulse_count = 0;
    high_speed_data_buffer_a[n].status_bits.high_energy_pulse = 0;
    high_speed_data_buffer_a[n].status_bits.arc_this_pulse = 0;
    high_speed_data_buffer_a[n].x_ray_on_seconds_lsw = 0;
    high_speed_data_buffer_a[n].x_ray_on_milliseconds = 0;
    high_speed_data_buffer_a[n].hvlambda_vmon_at_eoc_period = 0;
    high_speed_data_buffer_a[n].hvlambda_vmon_at_trigger = 0;
    high_speed_data_buffer_a[n].hvlambda_vpeak_at_eoc_period = 0;
    high_speed_data_buffer_a[n].afc_readback_current_position= 0;
    high_speed_data_buffer_a[n].afc_readback_target_position= 0;
    high_speed_data_buffer_a[n].afc_readback_a_input = 0;
    high_speed_data_buffer_a[n].afc_readback_b_input = 0;
    high_speed_data_buffer_a[n].afc_readback_filtered_error_reading = 0;
    high_speed_data_buffer_a[n].ionpump_readback_high_energy_target_current_reading = 0;
    high_speed_data_buffer_a[n].ionpump_readback_low_energy_target_current_reading = 0;
    high_speed_data_buffer_a[n].magmon_internal_adc_reading = 0;
    high_speed_data_buffer_a[n].magmon_external_adc_reading = 0;
    high_speed_data_buffer_a[n].psync_trigger_width_and_filtered_trigger_width = 0;
    high_speed_data_buffer_a[n].psync_grid_width_and_delay = 0;
    high_speed_data_buffer_a[n].psync_period = 0;

    high_speed_data_buffer_b[n].pulse_count = 0;
    high_speed_data_buffer_b[n].status_bits.high_energy_pulse = 0;
    high_speed_data_buffer_b[n].status_bits.arc_this_pulse = 0;
    high_speed_data_buffer_b[n].x_ray_on_seconds_lsw = 0;
    high_speed_data_buffer_b[n].x_ray_on_milliseconds = 0;
    high_speed_data_buffer_b[n].hvlambda_vmon_at_eoc_period = 0;
    high_speed_data_buffer_b[n].hvlambda_vmon_at_trigger = 0;
    high_speed_data_buffer_b[n].hvlambda_vpeak_at_eoc_period = 0;
    high_speed_data_buffer_b[n].afc_readback_current_position= 0;
    high_speed_data_buffer_b[n].afc_readback_target_position= 0;
    high_speed_data_buffer_b[n].afc_readback_a_input = 0;
    high_speed_data_buffer_b[n].afc_readback_b_input = 0;
    high_speed_data_buffer_b[n].afc_readback_filtered_error_reading = 0;
    high_speed_data_buffer_b[n].ionpump_readback_high_energy_target_current_reading = 0;
    high_speed_data_buffer_b[n].ionpump_readback_low_energy_target_current_reading = 0;
    high_speed_data_buffer_b[n].magmon_internal_adc_reading = 0;
    high_speed_data_buffer_b[n].magmon_external_adc_reading = 0;
    high_speed_data_buffer_b[n].psync_trigger_width_and_filtered_trigger_width = 0;
    high_speed_data_buffer_b[n].psync_grid_width_and_delay = 0;
    high_speed_data_buffer_b[n].psync_period = 0;
    n++;
  }
}


void ETMCanMasterClearDebug(void) {
  debug_data_ecb.debug_reg[0]        = 0;
  debug_data_ecb.debug_reg[1]        = 0;
  debug_data_ecb.debug_reg[2]        = 0;
  debug_data_ecb.debug_reg[3]        = 0;

  debug_data_ecb.debug_reg[4]        = 0;
  debug_data_ecb.debug_reg[5]        = 0;
  debug_data_ecb.debug_reg[6]        = 0;
  debug_data_ecb.debug_reg[7]        = 0;

  debug_data_ecb.debug_reg[8]        = 0;
  debug_data_ecb.debug_reg[9]        = 0;
  debug_data_ecb.debug_reg[10]       = 0;
  debug_data_ecb.debug_reg[11]       = 0;

  debug_data_ecb.debug_reg[12]       = 0;
  debug_data_ecb.debug_reg[13]       = 0;
  debug_data_ecb.debug_reg[14]       = 0;
  debug_data_ecb.debug_reg[15]       = 0;


  debug_data_ecb.can_tx_0            = 0;
  debug_data_ecb.can_tx_1            = 0;
  debug_data_ecb.can_tx_2            = 0;
  debug_data_ecb.CXEC_reg_max        = 0;

  debug_data_ecb.can_rx_0_filt_0     = 0;
  debug_data_ecb.can_rx_0_filt_1     = 0;
  debug_data_ecb.can_rx_1_filt_2     = 0;
  debug_data_ecb.CXINTF_max          = 0;

  debug_data_ecb.can_unknown_msg_id  = 0;
  debug_data_ecb.can_invalid_index   = 0;
  debug_data_ecb.can_address_error   = 0;
  debug_data_ecb.can_error_flag      = 0;

  debug_data_ecb.can_tx_buf_overflow = 0;
  debug_data_ecb.can_rx_buf_overflow = 0;
  debug_data_ecb.can_rx_log_buf_overflow = 0;
  debug_data_ecb.can_timeout         = 0;

  debug_data_ecb.reset_count         = 0;
  debug_data_ecb.RCON_value          = 0;
  debug_data_ecb.reserved_1          = 0;
  debug_data_ecb.reserved_0          = 0;

  debug_data_ecb.i2c_bus_error_count = 0;
  debug_data_ecb.spi_bus_error_count = 0;
  debug_data_ecb.scale_error_count   = 0;
  //self test results

  etm_can_master_tx_message_buffer.message_overwrite_count = 0;
  etm_can_master_rx_message_buffer.message_overwrite_count = 0;
  etm_can_master_rx_data_log_buffer.message_overwrite_count = 0;
  etm_can_persistent_data.reset_count = 0;
  etm_can_persistent_data.can_timeout_count = 0;


  _TRAPR = 0;
  _IOPUWR = 0;
  _EXTR = 0;
  _WDTO = 0;
  _SLEEP = 0;
  _IDLE = 0;
  _BOR = 0;
  _POR = 0;
  _SWR = 0;

  *CXINTF_ptr = 0;




  global_data_can_master.no_connect_count_ion_pump_board = 0;
  global_data_can_master.no_connect_count_magnetron_current_board = 0;
  global_data_can_master.no_connect_count_pulse_sync_board = 0;
  global_data_can_master.no_connect_count_hv_lambda_board = 0;
  global_data_can_master.no_connect_count_afc_board = 0;
  global_data_can_master.no_connect_count_cooling_interface_board = 0;
  global_data_can_master.no_connect_count_heater_magnet_board = 0;
  global_data_can_master.no_connect_count_gun_driver_board = 0;

}



void SendToEventLog(unsigned int log_id) {
  event_log.event_data[event_log.write_index].event_number = global_data_can_master.event_log_counter;
  event_log.event_data[event_log.write_index].event_time   = mem_time_seconds_now;
  event_log.event_data[event_log.write_index].event_id     = log_id;
  event_log.write_index++;
  event_log.write_index &= 0x7F;
  if (event_log.write_index == event_log.gui_index) {
    // The event log is full, over write th old events
    event_log.gui_index++;
    event_log.gui_index &= 0x7F;
  }
  global_data_can_master.event_log_counter++;
}





/*
  This is the CAN Interrupt for the Master module
*/
void DoCanInterrupt(void);

void __attribute__((interrupt(__save__(CORCON,SR)), no_auto_psv)) _C1Interrupt(void) {
  _C1IF = 0;
  DoCanInterrupt();
}

void __attribute__((interrupt(__save__(CORCON,SR)), no_auto_psv)) _C2Interrupt(void) {
  _C2IF = 0;
  DoCanInterrupt();
}


// DPARKER REMVOE THIS FUNCTION AFTER PULSE LOG IS WORKING ON MACHINE

void TestDataLog(void) {
  unsigned int fast_log_buffer_index;
  unsigned int pulse_time = 0;
  static unsigned long test_data_long_timer_holding_var;

  if (_SYNC_CONTROL_HIGH_SPEED_LOGGING) {
    
    if (ETMTickRunOnceEveryNMilliseconds(100, &test_data_long_timer_holding_var)) {
      // 100ms has passed - Send the next Message
      etm_can_master_next_pulse_count++;
    
      // Prepare the buffer to store the data
      fast_log_buffer_index = etm_can_master_next_pulse_count & 0x000F;
      if (etm_can_master_next_pulse_count & 0x0010) {
	// We are putting data into buffer A
	global_data_can_master.buffer_a_ready_to_send = 0;
	if (fast_log_buffer_index >= 3) {
	  global_data_can_master.buffer_b_ready_to_send = 1;
	  global_data_can_master.buffer_a_sent = 0;
	}
	  
	*(unsigned int*)&high_speed_data_buffer_a[fast_log_buffer_index].status_bits = 0; // clear the status bits register
	high_speed_data_buffer_a[fast_log_buffer_index].pulse_count = etm_can_master_next_pulse_count;
	if (etm_can_master_next_pulse_level) {
	  high_speed_data_buffer_a[fast_log_buffer_index].status_bits.high_energy_pulse = 1;
	}

	high_speed_data_buffer_a[fast_log_buffer_index].x_ray_on_seconds_lsw = mem_time_seconds_now;
	high_speed_data_buffer_a[fast_log_buffer_index].x_ray_on_milliseconds = pulse_time;
	  
	high_speed_data_buffer_a[fast_log_buffer_index].hvlambda_vmon_at_eoc_period = 0;
	high_speed_data_buffer_a[fast_log_buffer_index].hvlambda_vmon_at_trigger = 0;
	high_speed_data_buffer_a[fast_log_buffer_index].hvlambda_vpeak_at_eoc_period = 0;
	  
	high_speed_data_buffer_a[fast_log_buffer_index].afc_readback_current_position = 0;
	high_speed_data_buffer_a[fast_log_buffer_index].afc_readback_target_position = 0;
	high_speed_data_buffer_a[fast_log_buffer_index].afc_readback_a_input = 0;
	high_speed_data_buffer_a[fast_log_buffer_index].afc_readback_b_input = 0;
	high_speed_data_buffer_a[fast_log_buffer_index].afc_readback_filtered_error_reading = 0;
	  
	high_speed_data_buffer_a[fast_log_buffer_index].ionpump_readback_high_energy_target_current_reading = 0;
	high_speed_data_buffer_a[fast_log_buffer_index].ionpump_readback_low_energy_target_current_reading = 0;
	  
	high_speed_data_buffer_a[fast_log_buffer_index].magmon_internal_adc_reading = 0;
	high_speed_data_buffer_a[fast_log_buffer_index].magmon_external_adc_reading = 0;
	  
	high_speed_data_buffer_a[fast_log_buffer_index].psync_trigger_width_and_filtered_trigger_width = 0;
	high_speed_data_buffer_a[fast_log_buffer_index].psync_grid_width_and_delay = 0;
	high_speed_data_buffer_a[fast_log_buffer_index].psync_period = 0;
	  
      } else {
	// We are putting data into buffer B
	global_data_can_master.buffer_b_ready_to_send = 0;
	if (fast_log_buffer_index >= 3) {
	  global_data_can_master.buffer_a_ready_to_send = 1;
	  global_data_can_master.buffer_b_sent = 0;
	}

	  	  
	*(unsigned int*)&high_speed_data_buffer_b[fast_log_buffer_index].status_bits = 0; // Clear the status bits register
	high_speed_data_buffer_b[fast_log_buffer_index].pulse_count = etm_can_master_next_pulse_count;
	if (etm_can_master_next_pulse_level) {
	  high_speed_data_buffer_b[fast_log_buffer_index].status_bits.high_energy_pulse = 1;
	}
	  
	high_speed_data_buffer_b[fast_log_buffer_index].x_ray_on_seconds_lsw = mem_time_seconds_now;
	high_speed_data_buffer_b[fast_log_buffer_index].x_ray_on_milliseconds = pulse_time;
	  
	high_speed_data_buffer_b[fast_log_buffer_index].hvlambda_vmon_at_eoc_period = 0;
	high_speed_data_buffer_b[fast_log_buffer_index].hvlambda_vmon_at_trigger = 0;
	high_speed_data_buffer_b[fast_log_buffer_index].hvlambda_vpeak_at_eoc_period = 0;
	  
	high_speed_data_buffer_b[fast_log_buffer_index].afc_readback_current_position = 0;
	high_speed_data_buffer_b[fast_log_buffer_index].afc_readback_target_position = 0;
	high_speed_data_buffer_b[fast_log_buffer_index].afc_readback_a_input = 0;
	high_speed_data_buffer_b[fast_log_buffer_index].afc_readback_b_input = 0;
	high_speed_data_buffer_b[fast_log_buffer_index].afc_readback_filtered_error_reading = 0;
	  
	high_speed_data_buffer_b[fast_log_buffer_index].ionpump_readback_high_energy_target_current_reading = 0;
	high_speed_data_buffer_b[fast_log_buffer_index].ionpump_readback_low_energy_target_current_reading = 0;
	  
	high_speed_data_buffer_b[fast_log_buffer_index].magmon_internal_adc_reading = 0;
	high_speed_data_buffer_b[fast_log_buffer_index].magmon_external_adc_reading = 0;
	  
	high_speed_data_buffer_b[fast_log_buffer_index].psync_trigger_width_and_filtered_trigger_width = 0;
	high_speed_data_buffer_b[fast_log_buffer_index].psync_grid_width_and_delay = 0;
	high_speed_data_buffer_b[fast_log_buffer_index].psync_period = 0;
      }
    }
  }
}


void DoCanInterrupt(void) {
  ETMCanMessage can_message;
  unsigned int fast_log_buffer_index;
  unsigned int pulse_time;
  
  // Calculate the time (in milliseconds) that this interrupt occured
  // This is used for logging pulse messages
  pulse_time = can_master_millisecond_counter;
  pulse_time += ETMScaleFactor2((TMR5>>11),MACRO_DEC_TO_CAL_FACTOR_2(.8192),0);
  if (_T2IF) {
    pulse_time += 10;
  }

  debug_data_ecb.CXINTF_max |= *CXINTF_ptr;
  
  
  if (*CXRX0CON_ptr & BUFFER_FULL_BIT) {
    /*
      A message has been received in Buffer Zero
    */
    if (!(*CXRX0CON_ptr & FILTER_SELECT_BIT)) {
      // The command was received by Filter 0
      // It is a Next Pulse Level Command 
      debug_data_ecb.can_rx_0_filt_0++;
      ETMCanRXMessage(&can_message, CXRX0CON_ptr);
      etm_can_master_next_pulse_prf   = can_message.word2;
      etm_can_master_next_pulse_level = can_message.word1;
      etm_can_master_next_pulse_count = can_message.word0;

      if (_SYNC_CONTROL_HIGH_SPEED_LOGGING) {
	// Prepare the buffer to store the data
	fast_log_buffer_index = etm_can_master_next_pulse_count & 0x000F;
	if (etm_can_master_next_pulse_count & 0x0010) {
	  // We are putting data into buffer A
	  global_data_can_master.buffer_a_ready_to_send = 0;
	  if (fast_log_buffer_index >= 3) {
	    global_data_can_master.buffer_b_ready_to_send = 1;
	    global_data_can_master.buffer_a_sent = 0;
	  }
	  
	  *(unsigned int*)&high_speed_data_buffer_a[fast_log_buffer_index].status_bits = 0; // clear the status bits register
	  high_speed_data_buffer_a[fast_log_buffer_index].pulse_count = etm_can_master_next_pulse_count;
	  if (etm_can_master_next_pulse_level) {
	    high_speed_data_buffer_a[fast_log_buffer_index].status_bits.high_energy_pulse = 1;
	  }

	  high_speed_data_buffer_a[fast_log_buffer_index].x_ray_on_seconds_lsw = mem_time_seconds_now;
	  high_speed_data_buffer_a[fast_log_buffer_index].x_ray_on_milliseconds = pulse_time;
	  
	  high_speed_data_buffer_a[fast_log_buffer_index].hvlambda_vmon_at_eoc_period = 0;
	  high_speed_data_buffer_a[fast_log_buffer_index].hvlambda_vmon_at_trigger = 0;
	  high_speed_data_buffer_a[fast_log_buffer_index].hvlambda_vpeak_at_eoc_period = 0;
	  
	  high_speed_data_buffer_a[fast_log_buffer_index].afc_readback_current_position = 0;
	  high_speed_data_buffer_a[fast_log_buffer_index].afc_readback_target_position = 0;
	  high_speed_data_buffer_a[fast_log_buffer_index].afc_readback_a_input = 0;
	  high_speed_data_buffer_a[fast_log_buffer_index].afc_readback_b_input = 0;
	  high_speed_data_buffer_a[fast_log_buffer_index].afc_readback_filtered_error_reading = 0;
	  
	  high_speed_data_buffer_a[fast_log_buffer_index].ionpump_readback_high_energy_target_current_reading = 0;
	  high_speed_data_buffer_a[fast_log_buffer_index].ionpump_readback_low_energy_target_current_reading = 0;
	  
	  high_speed_data_buffer_a[fast_log_buffer_index].magmon_internal_adc_reading = 0;
	  high_speed_data_buffer_a[fast_log_buffer_index].magmon_external_adc_reading = 0;
	  
	  high_speed_data_buffer_a[fast_log_buffer_index].psync_trigger_width_and_filtered_trigger_width = 0;
	  high_speed_data_buffer_a[fast_log_buffer_index].psync_grid_width_and_delay = 0;
	  high_speed_data_buffer_a[fast_log_buffer_index].psync_period = 0;
	  
	} else {
	  // We are putting data into buffer B
	  global_data_can_master.buffer_b_ready_to_send = 0;
	  if (fast_log_buffer_index >= 3) {
	    global_data_can_master.buffer_a_ready_to_send = 1;
	    global_data_can_master.buffer_b_sent = 0;
	  }
	  
	  *(unsigned int*)&high_speed_data_buffer_b[fast_log_buffer_index].status_bits = 0; // Clear the status bits register
	  high_speed_data_buffer_b[fast_log_buffer_index].pulse_count = etm_can_master_next_pulse_count;
	  if (etm_can_master_next_pulse_level) {
	    high_speed_data_buffer_b[fast_log_buffer_index].status_bits.high_energy_pulse = 1;
	  }
	  
	  high_speed_data_buffer_b[fast_log_buffer_index].x_ray_on_seconds_lsw = mem_time_seconds_now;
	  high_speed_data_buffer_b[fast_log_buffer_index].x_ray_on_milliseconds = pulse_time;
	  
	  high_speed_data_buffer_b[fast_log_buffer_index].hvlambda_vmon_at_eoc_period = 0;
	  high_speed_data_buffer_b[fast_log_buffer_index].hvlambda_vmon_at_trigger = 0;
	  high_speed_data_buffer_b[fast_log_buffer_index].hvlambda_vpeak_at_eoc_period = 0;
	  
	  high_speed_data_buffer_b[fast_log_buffer_index].afc_readback_current_position = 0;
	  high_speed_data_buffer_b[fast_log_buffer_index].afc_readback_target_position = 0;
	  high_speed_data_buffer_b[fast_log_buffer_index].afc_readback_a_input = 0;
	  high_speed_data_buffer_b[fast_log_buffer_index].afc_readback_b_input = 0;
	  high_speed_data_buffer_b[fast_log_buffer_index].afc_readback_filtered_error_reading = 0;
	  
	  high_speed_data_buffer_b[fast_log_buffer_index].ionpump_readback_high_energy_target_current_reading = 0;
	  high_speed_data_buffer_b[fast_log_buffer_index].ionpump_readback_low_energy_target_current_reading = 0;
	  
	  high_speed_data_buffer_b[fast_log_buffer_index].magmon_internal_adc_reading = 0;
	  high_speed_data_buffer_b[fast_log_buffer_index].magmon_external_adc_reading = 0;
	  
	  high_speed_data_buffer_b[fast_log_buffer_index].psync_trigger_width_and_filtered_trigger_width = 0;
	  high_speed_data_buffer_b[fast_log_buffer_index].psync_grid_width_and_delay = 0;
	  high_speed_data_buffer_b[fast_log_buffer_index].psync_period = 0;
	  
	}
      }
    } else {
      // The commmand was received by Filter 1
      // This command gets pushed onto the command message buffer
      ETMCanRXMessageBuffer(&etm_can_master_rx_message_buffer, CXRX0CON_ptr);  
      debug_data_ecb.can_rx_0_filt_1++;
    }
    *CXINTF_ptr &= RX0_INT_FLAG_BIT; // Clear the RX0 Interrupt Flag  
  }
  
  if (*CXRX1CON_ptr & BUFFER_FULL_BIT) { 
    // A message has been recieved in Buffer 1
    // The command is a data log.  Add it to the data log buffer
    ETMCanRXMessageBuffer(&etm_can_master_rx_data_log_buffer, CXRX1CON_ptr);   
    debug_data_ecb.can_rx_1_filt_2++;
    *CXINTF_ptr &= RX1_INT_FLAG_BIT; // Clear the RX1 Interrupt Flag
  }

  if (!(*CXTX0CON_ptr & TX_REQ_BIT) && ((ETMCanBufferNotEmpty(&etm_can_master_tx_message_buffer)))) {  
    /*
      TX0 is empty and there is a message waiting in the transmit message buffer
      Load the next message into TX0
    */
    ETMCanTXMessageBuffer(&etm_can_master_tx_message_buffer, CXTX0CON_ptr);
    *CXINTF_ptr &= 0xFFFB; // Clear the TX0 Interrupt Flag
    debug_data_ecb.can_tx_0++;
  }
  
  if (*CXINTF_ptr & ERROR_FLAG_BIT) {
    // There was some sort of CAN Error
    debug_data_ecb.can_error_flag++;
    *CXINTF_ptr &= 0x001F; // Clear all the ERR Flags
  } else {
    // FLASH THE CAN LED
    if (ETMReadPinLatch(can_params.led)) {
      ETMClearPin(can_params.led);
    } else {
      ETMSetPin(can_params.led);
    }
  }

}

unsigned int ETMCanMasterGetPulsePRF(void) {
  return etm_can_master_next_pulse_prf;
}


void ETMCanMasterSendMsg(unsigned int id, unsigned int word3, unsigned int word2, unsigned int word1, unsigned int word0) {
  ETMCanMessage can_message;
  can_message.identifier = id;
  can_message.word3 = word3;
  can_message.word2 = word2;
  can_message.word1 = word1;
  can_message.word0 = word0;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();
}
