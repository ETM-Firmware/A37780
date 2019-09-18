#include <xc.h>
#include "P1395_CAN_MASTER.h"
#include "ETM.h"
#include "P1395_CAN_CORE.h"
#include "ETM_LINAC_COM.h"



/*
#define ETM_CAN_DATA_LOG_REGISTER_SCOPE_A                        0x180
#define ETM_CAN_DATA_LOG_REGISTER_SCOPE_B                        0x190
#define ETM_CAN_DATA_LOG_REGISTER_PULSE_SCOPE_DATA               0x1A0
#define ETM_CAN_DATA_LOG_REGISTER_HV_VMON_DATA                   0x1B0
*/

TYPE_SCOPE_PULSE_DATA scope_data_magnetron_current;
TYPE_SCOPE_PULSE_DATA scope_data_target_current;






static void ScopeDataLog(unsigned int data_log_id, unsigned int board_id, unsigned int data_3, unsigned int data_2, unsigned int data_1, unsigned int data_0);





void SetupPulseLog(void);



// ---------- Debug Holding Variables ------------------- //
ETMCanBoardData local_data_mirror[NUMBER_OF_DATA_MIRRORS];
ETMCanBoardDebuggingData debug_data_slave_mirror;


// ---------------  Global Variables that are to be used by the main source code ------------------- //
ETMCanBoardDebuggingData debug_data_ecb;



TYPE_SCOPE_DATA scope_data_a;
TYPE_SCOPE_DATA scope_data_b;
TYPE_PULSE_LOG  pulse_log;



static void LocalTransmitSync(void);
static void LocalTransmitToSlave(unsigned int cmd_id, unsigned int word_3, unsigned int word_2, unsigned int word_1, unsigned int word_0);
static void LocalTimedTransmit(void);
static void LocalReceiveSlaveStatus(ETMCanMessage* message_ptr);
static void LocalUpdateSlaveEventLog(ETMCanStatusRegister* previous_status, ETMCanStatusRegister* current_status, unsigned int source_board);
static void LocalUpdateSlaveTimeout(void);

  
static void LocalReceiveLogData(void);
static void LocalLogDataFromSlave(unsigned int data_log_id, unsigned int board_id, unsigned int data_3, unsigned int data_2, unsigned int data_1, unsigned int data_0);
static void LocalLogDebugDataFromSlave(unsigned int data_log_id, unsigned int board_id, unsigned int data_3, unsigned int data_2, unsigned int data_1, unsigned int data_0);
static void LocalClearDebug(void);
static void LocalUpdateSlaveNotReady(void);

unsigned int etm_can_active_debugging_board_id;


// ----------  local variables that are set through function calls -------------//
static ETMCanSyncMessage sync_message;
static unsigned int etm_can_magnetron_heater_scaled_heater_current;
static unsigned int etm_can_master_boards_to_ignore;

// ----------- local variables that are returned through function calls ------------ //
static volatile unsigned int can_master_all_slaves_ready;  // DPARKER HOW IS THIS RESET
static unsigned int etm_can_master_all_boards_connected;


// --------------------- Local Variables -------------------------- //
static unsigned long etm_can_master_can_led;
static unsigned long etm_can_master_sync_message_timer_holding_var;
static unsigned long etm_can_master_check_slave_timeout_timer_holding_var;
static unsigned int  persistent_data_reset_count __attribute__ ((persistent));
static unsigned int  persistent_data_can_timeout_count __attribute__ ((persistent));
static unsigned int  scope_a_settings = 0xFFFF;
static unsigned int  scope_b_settings = 0xFFFF;
static unsigned int  scope_hv_vmon_settings;



// --------- Local Buffers ---------------- // 
static ETMCanMessageBuffer         etm_can_master_rx_data_log_buffer;
static ETMCanMessageBuffer         etm_can_master_tx_message_buffer;


// ---------- Pointers to CAN stucutres so that we can use CAN1 or CAN2
static volatile unsigned int *CXEC_ptr;
static volatile unsigned int *CXINTF_ptr;
static volatile unsigned int *CXRX0CON_ptr;
static volatile unsigned int *CXRX1CON_ptr;
static volatile unsigned int *CXTX0CON_ptr;
static volatile unsigned int *CXTX1CON_ptr;
static volatile unsigned int *CXTX2CON_ptr;


// ---------------- CAN Message Defines ---------------------- //






void ETMCanMasterInitialize(unsigned int requested_can_port, unsigned long fcy, unsigned long can_operation_led, unsigned int can_interrupt_priority, unsigned int boards_to_ignore) {

  etm_can_master_boards_to_ignore = boards_to_ignore;
  
  if (can_interrupt_priority > 7) {
    can_interrupt_priority = 7;
  }

  etm_can_master_can_led = can_operation_led;

  persistent_data_reset_count++;
  // DPARKER this needs to be a setting somehow
  if (_NOT_LOGGED_0) {
    persistent_data_reset_count = 0;
    persistent_data_can_timeout_count = 0;
  }

  sync_message.sync_0_control_word.sync_0_reset_enable = 0;
  sync_message.sync_0_control_word.sync_1_high_speed_logging_enabled = 0;
  sync_message.sync_0_control_word.sync_4_cooling_fault = 1;
  sync_message.sync_0_control_word.sync_5_system_hv_disable = 1;
  sync_message.sync_0_control_word.sync_6_gun_driver_disable_heater = 1;
  sync_message.sync_0_control_word.sync_D_scope_HV_HVMON_active = 1;
  sync_message.sync_0_control_word.sync_E_ingnore_faults_enabled = 0;
  sync_message.sync_0_control_word.sync_F_clear_debug_data = 0;

  sync_message.pulse_count = 0;
  sync_message.next_energy_level = 0;
  sync_message.prf_from_ecb = 0;
  sync_message.scope_A_select = 0;
  sync_message.scope_B_select = 0;
      
  debug_data_ecb.reset_count = persistent_data_reset_count;
  debug_data_ecb.can_timeout = persistent_data_can_timeout_count;

  ETMCanBufferInitialize(&etm_can_master_tx_message_buffer);
  ETMCanBufferInitialize(&etm_can_master_rx_data_log_buffer);

  ETMPinTrisOutput(etm_can_master_can_led);
  
  scope_data_a.write_location = 0;
  scope_data_b.write_location = 0;
    
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
    
    // Enable Can interrupt
    _C1IE = 1;
  } else {
    // Use CAN2
    // DPARKER THIS IS NOT IMPLIMENTED YET
  }

}


void ETMCanMasterDoCan(void) {
  
  // A sync message is sent out with every trigger pulse
  // If there are no trigger pulses, a sync message will not be sent out
  // Check here to see if a sync message has been sent in the previous 50mS (20 Hz).  If not, send one here
  if (ETMTickGreaterThanNMilliseconds(SYNC_MESSAGE_MAX_TRANSMISSION_PERIOD, etm_can_master_sync_message_timer_holding_var)) {
    LocalTransmitSync();
  }
  
  LocalTimedTransmit();
  LocalReceiveLogData();
  LocalUpdateSlaveNotReady();

  if (sync_message.sync_0_control_word.sync_F_clear_debug_data) {
    //LocalClearDebug();
  }
  
  // DPARKER SET TO RUN ONCE EVERY 100mS
  if (ETMTickGreaterThanNMilliseconds(UPDATE_SLAVE_TIMEOUT_CHECK_PERIOD_MILLISECONDS, etm_can_master_check_slave_timeout_timer_holding_var)) {
    LocalUpdateSlaveTimeout();
  }
  

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

  debug_data_ecb.can_tx_buf_overflow = etm_can_master_tx_message_buffer.message_overwrite_count;
  debug_data_ecb.can_rx_log_buf_overflow = etm_can_master_rx_data_log_buffer.message_overwrite_count;

  
  debug_data_ecb.eeprom_internal_read_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_INTERNAL_COUNT);
  debug_data_ecb.eeprom_internal_read_error = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_INTERNAL_ERROR);
  debug_data_ecb.eeprom_internal_write_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_INTERNAL_COUNT);
  debug_data_ecb.eeprom_internal_write_error = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_INTERNAL_ERROR);


  debug_data_ecb.eeprom_i2c_read_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_I2C_COUNT);
  debug_data_ecb.eeprom_i2c_read_error = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_I2C_ERROR);
  debug_data_ecb.eeprom_i2c_write_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_I2C_COUNT);
  debug_data_ecb.eeprom_i2c_write_error = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_I2C_ERROR);

  debug_data_ecb.eeprom_spi_read_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_SPI_COUNT);
  debug_data_ecb.eeprom_spi_read_error = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_SPI_ERROR);
  debug_data_ecb.eeprom_spi_write_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_SPI_COUNT);
  debug_data_ecb.eeprom_spi_write_error = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_SPI_ERROR);
  

  debug_data_ecb.eeprom_crc_error_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_CRC_ERROR);


}

static void LocalTransmitSync(void) {
  ETMCanMessage sync_transmit;
  sync_transmit.identifier = ETM_CAN_MSG_SYNC_TX;
  sync_transmit.word0 = *(unsigned int*)&sync_message.sync_0_control_word;
  sync_transmit.word1 = *(unsigned int*)&sync_message.pulse_count;
  sync_transmit.word2 = sync_message.prf_from_ecb;
  sync_transmit.word3 = *(unsigned int*)&sync_message.scope_A_select;
  
  ETMCanTXMessage(&sync_transmit, CXTX1CON_ptr);
  debug_data_ecb.can_tx_1++;
  etm_can_master_sync_message_timer_holding_var = ETMTickGet();
}


static void LocalTransmitToSlave(unsigned int cmd_id, unsigned int word_3, unsigned int word_2, unsigned int word_1, unsigned int word_0) {
  ETMCanMessage can_message;
  if (cmd_id > 0x3F) {
    // DPARKER add debug data
    return;
  }
  can_message.identifier = (ETM_CAN_MSG_CMD_TX | (cmd_id << 2));
  can_message.word3 = word_3;
  can_message.word2 = word_2;
  can_message.word1 = word_1;
  can_message.word0 = word_0;
  ETMCanAddMessageToBuffer(&etm_can_master_tx_message_buffer, &can_message);
  MacroETMCanCheckTXBuffer();  // DPARKER - Figure out how to build this into ETMCanAddMessageToBuffer()  
}





static void LocalTimedTransmit(void) {
  static unsigned int transmit_message_select_counter;
  static unsigned long etm_can_master_timed_transmission_period_holding_var;
  
  // Once every 100ms, send out message to update board settings
  if (ETMTickRunOnceEveryNMilliseconds(ETM_CAN_MASTER_TIMED_TRANSMISSION_PERIOD_MILLI_SECONDS, &etm_can_master_timed_transmission_period_holding_var)) {
    transmit_message_select_counter++;
    if (transmit_message_select_counter >= 8) {
      transmit_message_select_counter = 0;
    }

    switch (transmit_message_select_counter) 
      {
      case 0x0:
	LocalTransmitToSlave(ETM_CAN_CMD_ID_HVPS_SET_POINTS,
			     ecb_data.dose_level_3.hvps_set_point,
			     ecb_data.dose_level_2.hvps_set_point,
			     ecb_data.dose_level_1.hvps_set_point,
			     ecb_data.dose_level_0.hvps_set_point);
	break;
	
      case 0x1:
	LocalTransmitToSlave(ETM_CAN_CMD_ID_MAGNET_SET_POINTS,
			     ecb_data.dose_level_3.electromagnet_set_point,
			     ecb_data.dose_level_2.electromagnet_set_point,
			     ecb_data.dose_level_1.electromagnet_set_point,
			     ecb_data.dose_level_0.electromagnet_set_point);
	break;
	
      case 0x2:
	LocalTransmitToSlave(ETM_CAN_CMD_ID_GUN_PULSE_TOP_SET_POINTS,
			     ecb_data.dose_level_3.gun_driver_pulse_top_voltage,
			     ecb_data.dose_level_2.gun_driver_pulse_top_voltage,
			     ecb_data.dose_level_1.gun_driver_pulse_top_voltage,
			     ecb_data.dose_level_0.gun_driver_pulse_top_voltage);
	break;
	
      case 0x3:
	LocalTransmitToSlave(ETM_CAN_CMD_ID_GUN_CATHODE_SET_POINTS,
			     ecb_data.dose_level_3.gun_driver_cathode_voltage,
			     ecb_data.dose_level_2.gun_driver_cathode_voltage,
			     ecb_data.dose_level_1.gun_driver_cathode_voltage,
			     ecb_data.dose_level_0.gun_driver_cathode_voltage);
	break;
	
      case 0x4:
	LocalTransmitToSlave(ETM_CAN_CMD_ID_AFC_HOME_POSTION,
			     ecb_data.dose_level_3.afc_home_poistion,
			     ecb_data.dose_level_2.afc_home_poistion,
			     ecb_data.dose_level_1.afc_home_poistion,
			     ecb_data.dose_level_0.afc_home_poistion);
	break;
	
      case 0x5:
	LocalTransmitToSlave(ETM_CAN_CMD_ID_ALL_DOSE_SET_POINTS_REGISTER_A,
			     etm_can_magnetron_heater_scaled_heater_current,
			     ecb_data.dose_level_all.afc_aux_control_or_offset,
			     ecb_data.dose_level_all.gun_driver_bias_voltage,
			     ecb_data.dose_level_all.gun_driver_heater_voltage);
	break;

      case 0x6:
	LocalTransmitToSlave(ETM_CAN_CMD_ID_ALL_DOSE_SET_POINTS_REGISTER_B,
			     0,
			     0,
			     0,
			     ecb_data.dose_level_all.afc_manual_target_position);
	break;


      case 0x7:
	LocalTransmitToSlave(ETM_CAN_CMD_ID_SCOPE_SETTINGS,
			     scope_a_settings,
			     scope_b_settings,
			     scope_hv_vmon_settings,
			     0);
	break;
	
      default:
	// Don't send out anything
	break;
      }

  }
}



void ETMCanMasterSendSlaveResetMCU(unsigned int board_id) {
  LocalTransmitToSlave(ETM_CAN_CMD_ID_RESET_MCU, board_id, 0, 0, 0);
}

void ETMCanMasterSendSlaveLoadDefaultEEpromData(unsigned int board_id) {
  LocalTransmitToSlave(ETM_CAN_CMD_ID_LOAD_DEFAULT_CALIBRATION, board_id, 0, 0, 0);
}

void ETMCanMasterSendSlaveRevAndSerialNumber(unsigned int board_id, unsigned int serial_number, unsigned int rev_char) {
  LocalTransmitToSlave(ETM_CAN_CMD_ID_LOAD_REV_AND_SERIAL_NUMBER, board_id, 0, rev_char, serial_number);
}

void ETMCanMasterSendSlaveCalibrationPair(unsigned int board_id, unsigned int cal_pair_select, unsigned int cal_offset, unsigned int cal_gain) {
  LocalTransmitToSlave(ETM_CAN_CMD_ID_SET_CAL_PAIR, board_id, cal_pair_select, cal_gain, cal_offset);
}

void ETMCanMasterSendSlaveRAMDebugLocations(unsigned int board_id, unsigned int address_A, unsigned int address_B, unsigned int address_C) {
  LocalTransmitToSlave(ETM_CAN_CMD_ID_SET_RAM_DEBUG, board_id, address_C, address_B, address_A);
}

void ETMCanMasterSendSlaveEEPROMDebug(unsigned int board_id, unsigned int eeprom_register) {
  LocalTransmitToSlave(ETM_CAN_CMD_ID_SET_EEPROM_DEBUG, board_id, eeprom_register, 0, 0);
}  

void ETMCanMasterSendDiscreteCMD(unsigned int discrete_cmd_id) {
  LocalTransmitToSlave(ETM_CAN_CMD_ID_DISCRETE_CMD, discrete_cmd_id, 0, 0, 0);
}

void ETMCanMasterSendSlaveClearDebug(void) {
  LocalTransmitToSlave(ETM_CAN_CMD_ID_CLEAR_DEBUG, 0, 0, 0, 0);
}

unsigned int ETMCanMasterCheckResetActive(void) {
  if (sync_message.sync_0_control_word.sync_0_reset_enable) {
    return 0xFFFF;
  }
  return 0;
}


void ETMCanMasterSyncSet(unsigned char sync_setting_select, unsigned char value) {
  switch (sync_setting_select) {

  case SYNC_BIT_RESET_ENABLE:
    sync_message.sync_0_control_word.sync_0_reset_enable = 0;
    if (value) {
      sync_message.sync_0_control_word.sync_0_reset_enable = 1;
    }
    break;

  case SYNC_BIT_ENABLE_PULSE_LOG:
    sync_message.sync_0_control_word.sync_1_high_speed_logging_enabled = 0;
    if (value) {
      sync_message.sync_0_control_word.sync_1_high_speed_logging_enabled = 1;
    }
    break;

  case SYNC_BIT_COOLING_FAULT:
    sync_message.sync_0_control_word.sync_4_cooling_fault = 0;
    if (value) {
      sync_message.sync_0_control_word.sync_4_cooling_fault = 1;
    }
    break;
    
  case SYNC_BIT_HV_DISABLE:
    sync_message.sync_0_control_word.sync_5_system_hv_disable = 0;
    if (value) {
      sync_message.sync_0_control_word.sync_5_system_hv_disable = 1;
    }
    break;

  case SYNC_BIT_GUN_HTR_DISABLE:
    sync_message.sync_0_control_word.sync_6_gun_driver_disable_heater = 0;
    if (value) {
      sync_message.sync_0_control_word.sync_6_gun_driver_disable_heater = 1;
    }
    break;
    
  case SYNC_BIT_CLEAR_DEBUG_DATA:
    sync_message.sync_0_control_word.sync_F_clear_debug_data = 0;
    if (value) {
      sync_message.sync_0_control_word.sync_F_clear_debug_data = 1;
    }
    break;
    
  }
}

void ETMCanMasterSendSyncMessage(unsigned char dose_level, unsigned char pulse_count) {
  // Set up the fast log register
  sync_message.pulse_count = pulse_count;
  sync_message.next_energy_level = dose_level;
  LocalTransmitSync();

  if (sync_message.sync_0_control_word.sync_1_high_speed_logging_enabled) {
    SetupPulseLog();
  }
}


#define LOG_DEFAULT_VALUE 0xFFEF



void ETMCanMasterClearHighSpeedLogging(void) {
  unsigned int *data_ptr;
  unsigned int n;
  data_ptr = (unsigned int*)&pulse_log;

  for (n = 0; n < (sizeof(TYPE_PULSE_LOG)>>1); n++) {
    *data_ptr = 0;
    data_ptr++;
  }
}


void SetupPulseLog(void) {
  unsigned char index;
  
  if (sync_message.pulse_count & 0x10) {
    // Filling data_b
    index = (sync_message.pulse_count >> 1) & 0x07;  
    if (sync_message.pulse_count & 0x1) {
      // This is an odd pulse count - Only update the ECB Trigger data
      pulse_log.data_b[index].gun_trigger_width_b = 100;
      pulse_log.data_b[index].gun_trigger_start_b = sync_message.pulse_count;
    } else {
      // Initialize the array
      if (index == 3) {
      // It is time to send the other log
      pulse_log.data_a_ready_to_send = 1;
      }

      pulse_log.data_b[index].pulse_count_a = sync_message.pulse_count;
      pulse_log.data_b[index].status_bits_a_and_b = 0; // DPARKER what to put here 
      pulse_log.data_b[index].trigger_time_a = ETMTickGet();
      
      pulse_log.data_b[index].gun_trigger_width_a = 200; // DPARKER PUT REAL GRID TIMING HERE
      pulse_log.data_b[index].gun_trigger_start_a = sync_message.pulse_count; // DPARKER PUT REAL GRID TIMING HERE
      pulse_log.data_b[index].gun_trigger_width_b = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].gun_trigger_start_b = LOG_DEFAULT_VALUE;
      
      pulse_log.data_b[index].hvps_eoc_a = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].hvps_spare_a = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].hvps_eoc_b = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].hvps_spare_b = LOG_DEFAULT_VALUE;
      
      pulse_log.data_b[index].afc_current_position_a = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].afc_reverse_reading_a = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].afc_current_position_b = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].afc_reverse_reading_b = LOG_DEFAULT_VALUE;
      
      pulse_log.data_b[index].magnetron_current_sample_a = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].magnetron_current_integral_a = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].magnetron_current_sample_b = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].magnetron_current_integral_b = LOG_DEFAULT_VALUE;
      
      pulse_log.data_b[index].target_current_sample_a = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].target_current_integral_a = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].target_current_sample_b = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].target_current_integral_b = LOG_DEFAULT_VALUE;
      
      pulse_log.data_b[index].gun_driver_data_0_a = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].gun_driver_data_1_a = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].gun_driver_data_0_b = LOG_DEFAULT_VALUE;
      pulse_log.data_b[index].gun_driver_data_1_b = LOG_DEFAULT_VALUE;
    }
  } else {
    // Filling data_a
    index = (sync_message.pulse_count >> 1) & 0x07;  
    if (sync_message.pulse_count & 0x1) {
      // This is an odd pulse count - Only update the ECB Trigger data
      pulse_log.data_a[index].gun_trigger_width_b = 300;
      pulse_log.data_a[index].gun_trigger_start_b = sync_message.pulse_count;
    } else {
      // Initialize the array
      if (index == 3) {
	// It is time to send the other log
	pulse_log.data_b_ready_to_send = 1;
      }

      pulse_log.data_a[index].pulse_count_a = sync_message.pulse_count;
      pulse_log.data_a[index].status_bits_a_and_b = 0; // DPARKER what to put here 
      pulse_log.data_a[index].trigger_time_a = ETMTickGet();
      
      pulse_log.data_a[index].gun_trigger_width_a = 400; // DPARKER PUT REAL GRID TIMING HERE
      pulse_log.data_a[index].gun_trigger_start_a = sync_message.pulse_count; // DPARKER PUT REAL GRID TIMING HERE
      pulse_log.data_a[index].gun_trigger_width_b = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].gun_trigger_start_b = LOG_DEFAULT_VALUE;
      
      pulse_log.data_a[index].hvps_eoc_a = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].hvps_spare_a = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].hvps_eoc_b = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].hvps_spare_b = LOG_DEFAULT_VALUE;
      
      pulse_log.data_a[index].afc_current_position_a = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].afc_reverse_reading_a = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].afc_current_position_b = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].afc_reverse_reading_b = LOG_DEFAULT_VALUE;
      
      pulse_log.data_a[index].magnetron_current_sample_a = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].magnetron_current_integral_a = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].magnetron_current_sample_b = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].magnetron_current_integral_b = LOG_DEFAULT_VALUE;
      
      pulse_log.data_a[index].target_current_sample_a = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].target_current_integral_a = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].target_current_sample_b = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].target_current_integral_b = LOG_DEFAULT_VALUE;
      
      pulse_log.data_a[index].gun_driver_data_0_a = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].gun_driver_data_1_a = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].gun_driver_data_0_b = LOG_DEFAULT_VALUE;
      pulse_log.data_a[index].gun_driver_data_1_b = LOG_DEFAULT_VALUE;
    }
  }
}


//void ETMCanMasterSendDefaultRequestRTSP(unsigned int board_id);
//void ETMCanMasterSendDefaultConfirmRTSP(unsigned int board_id);
//void ETMCanMasterSendDefaultIgnoreFaults(unsigned int board_id, unsigned int faults_to_ignore);


static void LocalReceiveSlaveStatus(ETMCanMessage* message_ptr) {
  ETMCanStatusRegister status_message;
  unsigned int source_board;

  source_board = (message_ptr->identifier >> 2);
  source_board &= 0x000F;

  if (source_board > NUMBER_OF_DATA_MIRRORS) {
    // Not a valid RAM address, this would do bad things to the RAM
    // DPAKER Increment some fault counter;
    debug_data_ecb.can_address_error++;
    return;
  }

  status_message.control_notice_bits = *(ETMCanStatusRegisterControlAndNoticeBits*)&message_ptr->word0;
  status_message.fault_bits          = *(ETMCanStatusRegisterFaultBits*)&message_ptr->word1;
  status_message.warning_bits        = *(ETMCanStatusRegisterWarningBits*)&message_ptr->word2;
  status_message.not_logged_bits     = *(ETMCanStatusRegisterNotLoggedBits*)&message_ptr->word3;


  if (((etm_can_master_boards_to_ignore >> source_board) & 0x0001) == 0) {
    // We are not ignoring faults from this board
    if (status_message.control_notice_bits.control_not_ready) {
      can_master_all_slaves_ready = 0x0000;
    }
  }
  
  LocalUpdateSlaveEventLog(&local_data_mirror[source_board].status, &status_message, source_board);
  
  local_data_mirror[source_board].status = status_message;
  local_data_mirror[source_board].time_last_status_message_recieved = ETMTickGet();
}


static void LocalUpdateSlaveEventLog(ETMCanStatusRegister* previous_status, ETMCanStatusRegister* current_status, unsigned int source_board) {
  unsigned int log_id;

  /*
    Log ID for status message changes
    0xCZYN
    0xC000 is base number for slave message
    Z is the address of the slave
    Y is the register select, 0/1 is control bit set/clear, 2 is a notice bit, 3/4 is fault set/cleared, 5/6 is log bit set/cleared 
    N is the number of the bit set
  */

  log_id = 0xC000;
  log_id += source_board << 8;
  
  // First update the control_notice_bits
  if ((*(unsigned int*)&previous_status->control_notice_bits) != (*(unsigned int*)&current_status->control_notice_bits)) {
    //update based on changes
    if (previous_status->control_notice_bits.control_not_ready != current_status->control_notice_bits.control_not_ready) {
      if (current_status->control_notice_bits.control_not_ready) {
	SendToEventLog(log_id + 0x00);
      } else {
	SendToEventLog(log_id + 0x10);
      }
    }

    if (previous_status->control_notice_bits.control_not_configured != current_status->control_notice_bits.control_not_configured) {
      if (current_status->control_notice_bits.control_not_configured) {
	SendToEventLog(log_id + 0x01);
      } else {
	SendToEventLog(log_id + 0x11);
      }
    }

    if (previous_status->control_notice_bits.control_self_check_error != current_status->control_notice_bits.control_self_check_error) {
      if (current_status->control_notice_bits.control_self_check_error) {
	SendToEventLog(log_id + 0x02);
      } else {
	SendToEventLog(log_id + 0x12);
      }
    }

    if (previous_status->control_notice_bits.control_3_unused != current_status->control_notice_bits.control_3_unused) {
      if (current_status->control_notice_bits.control_3_unused) {
	SendToEventLog(log_id + 0x03);
      } else {
	SendToEventLog(log_id + 0x13);
      }
    }

    if (previous_status->control_notice_bits.control_4_unused != current_status->control_notice_bits.control_4_unused) {
      if (current_status->control_notice_bits.control_4_unused) {
	SendToEventLog(log_id + 0x04);
      } else {
	SendToEventLog(log_id + 0x14);
      }
    }

    if (previous_status->control_notice_bits.control_5_unused != current_status->control_notice_bits.control_5_unused) {
      if (current_status->control_notice_bits.control_5_unused) {
	SendToEventLog(log_id + 0x05);
      } else {
	SendToEventLog(log_id + 0x15);
      }
    }

    if (previous_status->control_notice_bits.control_6_unused != current_status->control_notice_bits.control_6_unused) {
      if (current_status->control_notice_bits.control_6_unused) {
	SendToEventLog(log_id + 0x06);
      } else {
	SendToEventLog(log_id + 0x16);
      }
    }

    if (previous_status->control_notice_bits.control_7_unused != current_status->control_notice_bits.control_7_unused) {
      if (current_status->control_notice_bits.control_7_unused) {
	SendToEventLog(log_id + 0x07);
      } else {
	SendToEventLog(log_id + 0x17);
      }
    }

    if (current_status->control_notice_bits.notice_0) {
      SendToEventLog(log_id + 0x20);
      current_status->control_notice_bits.notice_0 = 0;
    }

    if (current_status->control_notice_bits.notice_1) {
      SendToEventLog(log_id + 0x21);
      current_status->control_notice_bits.notice_1 = 0;
    }

    if (current_status->control_notice_bits.notice_2) {
      SendToEventLog(log_id + 0x22);
      current_status->control_notice_bits.notice_2 = 0;
    }

    if (current_status->control_notice_bits.notice_3) {
      SendToEventLog(log_id + 0x23);
      current_status->control_notice_bits.notice_3 = 0;
    }

    if (current_status->control_notice_bits.notice_4) {
      SendToEventLog(log_id + 0x24);
      current_status->control_notice_bits.notice_4 = 0;
    }

    if (current_status->control_notice_bits.notice_5) {
      SendToEventLog(log_id + 0x25);
      current_status->control_notice_bits.notice_5 = 0;
    }

    if (current_status->control_notice_bits.notice_6) {
      SendToEventLog(log_id + 0x26);
      current_status->control_notice_bits.notice_6 = 0;
    }

    if (current_status->control_notice_bits.notice_7) {
      SendToEventLog(log_id + 0x27);
      current_status->control_notice_bits.notice_7 = 0;
    }
  }
  
  
  if ((*(unsigned int*)&previous_status->fault_bits) != (*(unsigned int*)&current_status->fault_bits)) {
    
    if (previous_status->fault_bits.fault_0 != current_status->fault_bits.fault_0) {
      if (current_status->fault_bits.fault_0) {
	SendToEventLog(log_id + 0x30);
      }	else {
	SendToEventLog(log_id + 0x40);
      }
    }
    
    if (previous_status->fault_bits.fault_1 != current_status->fault_bits.fault_1) {
      if (current_status->fault_bits.fault_1) {
	SendToEventLog(log_id + 0x31);
      }	else {
	SendToEventLog(log_id + 0x41);
      }
    }

    if (previous_status->fault_bits.fault_2 != current_status->fault_bits.fault_2) {
      if (current_status->fault_bits.fault_2) {
	SendToEventLog(log_id + 0x32);
      }	else {
	SendToEventLog(log_id + 0x42);
      }
    }

    if (previous_status->fault_bits.fault_3 != current_status->fault_bits.fault_3) {
      if (current_status->fault_bits.fault_3) {
	SendToEventLog(log_id + 0x33);
      }	else {
	SendToEventLog(log_id + 0x43);
      }
    }

    if (previous_status->fault_bits.fault_4 != current_status->fault_bits.fault_4) {
      if (current_status->fault_bits.fault_4) {
	SendToEventLog(log_id + 0x34);
      }	else {
	SendToEventLog(log_id + 0x44);
      }
    }

    if (previous_status->fault_bits.fault_5 != current_status->fault_bits.fault_5) {
      if (current_status->fault_bits.fault_5) {
	SendToEventLog(log_id + 0x35);
      }	else {
	SendToEventLog(log_id + 0x45);
      }
    }

    if (previous_status->fault_bits.fault_6 != current_status->fault_bits.fault_6) {
      if (current_status->fault_bits.fault_6) {
	SendToEventLog(log_id + 0x36);
      }	else {
	SendToEventLog(log_id + 0x46);
      }
    }

    if (previous_status->fault_bits.fault_7 != current_status->fault_bits.fault_7) {
      if (current_status->fault_bits.fault_7) {
	SendToEventLog(log_id + 0x37);
      }	else {
	SendToEventLog(log_id + 0x47);
      }
    }

    if (previous_status->fault_bits.fault_8 != current_status->fault_bits.fault_8) {
      if (current_status->fault_bits.fault_8) {
	SendToEventLog(log_id + 0x38);
      }	else {
	SendToEventLog(log_id + 0x48);
      }
    }

    if (previous_status->fault_bits.fault_9 != current_status->fault_bits.fault_9) {
      if (current_status->fault_bits.fault_9) {
	SendToEventLog(log_id + 0x39);
      }	else {
	SendToEventLog(log_id + 0x49);
      }
    }

    if (previous_status->fault_bits.fault_A != current_status->fault_bits.fault_A) {
      if (current_status->fault_bits.fault_A) {
	SendToEventLog(log_id + 0x3A);
      }	else {
	SendToEventLog(log_id + 0x4A);
      }
    }

    if (previous_status->fault_bits.fault_B != current_status->fault_bits.fault_B) {
      if (current_status->fault_bits.fault_B) {
	SendToEventLog(log_id + 0x3B);
      }	else {
	SendToEventLog(log_id + 0x4B);
      }
    }

    if (previous_status->fault_bits.fault_C != current_status->fault_bits.fault_C) {
      if (current_status->fault_bits.fault_C) {
	SendToEventLog(log_id + 0x3C);
      }	else {
	SendToEventLog(log_id + 0x4C);
      }
    }

    if (previous_status->fault_bits.fault_D != current_status->fault_bits.fault_D) {
      if (current_status->fault_bits.fault_D) {
	SendToEventLog(log_id + 0x3D);
      }	else {
	SendToEventLog(log_id + 0x4D);
      }
    }

    if (previous_status->fault_bits.fault_E != current_status->fault_bits.fault_E) {
      if (current_status->fault_bits.fault_E) {
	SendToEventLog(log_id + 0x3E);
      }	else {
	SendToEventLog(log_id + 0x4E);
      }
    }

    if (previous_status->fault_bits.fault_F != current_status->fault_bits.fault_F) {
      if (current_status->fault_bits.fault_F) {
	SendToEventLog(log_id + 0x3F);
      }	else {
	SendToEventLog(log_id + 0x4F);
      }
    }
  }
  

  if ((*(unsigned int*)&previous_status->warning_bits) != (*(unsigned int*)&current_status->warning_bits)) {

    if (previous_status->warning_bits.warning_0 != current_status->warning_bits.warning_0) {
      if (current_status->warning_bits.warning_0) {
	SendToEventLog(log_id + 0x50);
      }	else {
	SendToEventLog(log_id + 0x60);
      }
    }

    if (previous_status->warning_bits.warning_1 != current_status->warning_bits.warning_1) {
      if (current_status->warning_bits.warning_1) {
	SendToEventLog(log_id + 0x51);
      }	else {
	SendToEventLog(log_id + 0x61);
      }
    }

    if (previous_status->warning_bits.warning_2 != current_status->warning_bits.warning_2) {
      if (current_status->warning_bits.warning_2) {
	SendToEventLog(log_id + 0x52);
      }	else {
	SendToEventLog(log_id + 0x62);
      }
    }

    if (previous_status->warning_bits.warning_3 != current_status->warning_bits.warning_3) {
      if (current_status->warning_bits.warning_3) {
	SendToEventLog(log_id + 0x53);
      }	else {
	SendToEventLog(log_id + 0x63);
      }
    }

    if (previous_status->warning_bits.warning_4 != current_status->warning_bits.warning_4) {
      if (current_status->warning_bits.warning_4) {
	SendToEventLog(log_id + 0x54);
      }	else {
	SendToEventLog(log_id + 0x64);
      }
    }

    if (previous_status->warning_bits.warning_5 != current_status->warning_bits.warning_5) {
      if (current_status->warning_bits.warning_5) {
	SendToEventLog(log_id + 0x55);
      }	else {
	SendToEventLog(log_id + 0x65);
      }
    }

    if (previous_status->warning_bits.warning_6 != current_status->warning_bits.warning_6) {
      if (current_status->warning_bits.warning_6) {
	SendToEventLog(log_id + 0x56);
      }	else {
	SendToEventLog(log_id + 0x66);
      }
    }

    if (previous_status->warning_bits.warning_7 != current_status->warning_bits.warning_7) {
      if (current_status->warning_bits.warning_7) {
	SendToEventLog(log_id + 0x57);
      }	else {
	SendToEventLog(log_id + 0x67);
      }
    }

    if (previous_status->warning_bits.warning_8 != current_status->warning_bits.warning_8) {
      if (current_status->warning_bits.warning_8) {
	SendToEventLog(log_id + 0x58);
      }	else {
	SendToEventLog(log_id + 0x68);
      }
    }

    if (previous_status->warning_bits.warning_9 != current_status->warning_bits.warning_9) {
      if (current_status->warning_bits.warning_9) {
	SendToEventLog(log_id + 0x59);
      }	else {
	SendToEventLog(log_id + 0x69);
      }
    }

    if (previous_status->warning_bits.warning_A != current_status->warning_bits.warning_A) {
      if (current_status->warning_bits.warning_A) {
	SendToEventLog(log_id + 0x5A);
      }	else {
	SendToEventLog(log_id + 0x6A);
      }
    }

    if (previous_status->warning_bits.warning_B != current_status->warning_bits.warning_B) {
      if (current_status->warning_bits.warning_B) {
	SendToEventLog(log_id + 0x5B);
      }	else {
	SendToEventLog(log_id + 0x6B);
      }
    }

    if (previous_status->warning_bits.warning_C != current_status->warning_bits.warning_C) {
      if (current_status->warning_bits.warning_C) {
	SendToEventLog(log_id + 0x5C);
      }	else {
	SendToEventLog(log_id + 0x6C);
      }
    }

    if (previous_status->warning_bits.warning_D != current_status->warning_bits.warning_D) {
      if (current_status->warning_bits.warning_D) {
	SendToEventLog(log_id + 0x5D);
      }	else {
	SendToEventLog(log_id + 0x6D);
      }
    }

    if (previous_status->warning_bits.warning_E != current_status->warning_bits.warning_E) {
      if (current_status->warning_bits.warning_E) {
	SendToEventLog(log_id + 0x5E);
      }	else {
	SendToEventLog(log_id + 0x6E);
      }
    }

    if (previous_status->warning_bits.warning_F != current_status->warning_bits.warning_F) {
      if (current_status->warning_bits.warning_F) {
	SendToEventLog(log_id + 0x5F);
      }	else {
	SendToEventLog(log_id + 0x6F);
      }
    }
  }
}


void PulseDataLog(log_id, board_id, word3, word2, word1, word0) {
  unsigned int index;
  index = log_id & 0x0007;
  
  if (board_id == ETM_CAN_ADDR_HV_LAMBDA_BOARD) {
    if (log_id & 0x0008) {
      pulse_log.data_b[index].hvps_eoc_a = word3;
      pulse_log.data_b[index].hvps_spare_a = word2;
      pulse_log.data_b[index].hvps_eoc_b = word1;
      pulse_log.data_b[index].hvps_spare_b = word0;
    } else {
      pulse_log.data_a[index].hvps_eoc_a = word3;
      pulse_log.data_a[index].hvps_spare_a = word2;
      pulse_log.data_a[index].hvps_eoc_b = word1;
      pulse_log.data_a[index].hvps_spare_b = word0;
    }
  }

  if (board_id == ETM_CAN_ADDR_AFC_CONTROL_BOARD) {
    if (log_id & 0x0008) {
      pulse_log.data_b[index].afc_current_position_a = word3;
      pulse_log.data_b[index].afc_reverse_reading_a = word2;
      pulse_log.data_b[index].afc_current_position_b = word1;
      pulse_log.data_b[index].afc_reverse_reading_b = word0;
    } else {
      pulse_log.data_a[index].afc_current_position_a = word3;
      pulse_log.data_a[index].afc_reverse_reading_a = word2;
      pulse_log.data_a[index].afc_current_position_b = word1;
      pulse_log.data_a[index].afc_reverse_reading_b = word0;
    }
  }

  if (board_id == ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD) {
    if (log_id & 0x0008) {
      pulse_log.data_b[index].magnetron_current_sample_a = word3;
      pulse_log.data_b[index].magnetron_current_integral_a = word2;
      pulse_log.data_b[index].magnetron_current_sample_b = word1;
      pulse_log.data_b[index].magnetron_current_integral_b = word0;
    } else {
      pulse_log.data_a[index].magnetron_current_sample_a = word3;
      pulse_log.data_a[index].magnetron_current_integral_a = word2;
      pulse_log.data_a[index].magnetron_current_sample_b = word1;
      pulse_log.data_a[index].magnetron_current_integral_b = word0;
    }
  }

  if (board_id == ETM_CAN_ADDR_TARGET_CURRENT_BOARD) {
    if (log_id & 0x0008) {
      pulse_log.data_b[index].target_current_sample_a = word3;
      pulse_log.data_b[index].target_current_integral_a = word2;
      pulse_log.data_b[index].target_current_sample_b = word1;
      pulse_log.data_b[index].target_current_integral_b = word0;
    } else {
      pulse_log.data_a[index].target_current_sample_a = word3;
      pulse_log.data_a[index].target_current_integral_a = word2;
      pulse_log.data_a[index].target_current_sample_b = word1;
      pulse_log.data_a[index].target_current_integral_b = word0;
    }
  }

  if (board_id == ETM_CAN_ADDR_GUN_DRIVER_BOARD) {
    if (log_id & 0x0008) {
      pulse_log.data_b[index].gun_driver_data_0_a = word3;
      pulse_log.data_b[index].gun_driver_data_1_a = word2;
      pulse_log.data_b[index].gun_driver_data_0_b = word1;
      pulse_log.data_b[index].gun_driver_data_1_b = word0;
    } else {
      pulse_log.data_a[index].gun_driver_data_0_a = word3;
      pulse_log.data_a[index].gun_driver_data_1_a = word2;
      pulse_log.data_a[index].gun_driver_data_0_b = word1;
      pulse_log.data_a[index].gun_driver_data_1_b = word0;
    }
  }
}




#define LOG_ID_SLAVE_CONNECTION_TIMEOUT_BASE_ID           0x0000
#define LOG_ID_SLAVE_CONNECTION_ESTABLISHED_BASE_ID       0x0080

static void LocalUpdateSlaveTimeout(void) {
  // DPARKER, NEED A WAY TO TELL THE CAN MODULE TO AVOID CERTAIN BOARDS.
  // SUGGEST AN "IGNORE" INTERGER THAT IS PASSED IN
  // IF THE BIT IS SET, THAT NUMBER BOARD WILL BE IGNORED.

  unsigned int ignore;
  unsigned int n;

  ignore = etm_can_master_boards_to_ignore;
  etm_can_master_all_boards_connected = 0xFFFF;
  
  // Check all 11 Slaves for timeout
  // No need to check the ECB for timeout as that does not make sense
  for(n = 0; n < NUMBER_OF_DATA_MIRRORS; n++) {
    if (ignore & 0x0001) {
      // We are ignoring this board
      local_data_mirror[n].connection_timeout = 0;
    } else {
      if (ETMTickGreaterThanNMilliseconds(ETM_CAN_MASTER_SLAVE_TIMEOUT_MILLI_SECONDS, local_data_mirror[n].time_last_status_message_recieved)) {
	if (local_data_mirror[n].connection_timeout == 0) {
	  // This is a new timeout
	  SendToEventLog(LOG_ID_SLAVE_CONNECTION_TIMEOUT_BASE_ID + n);
	  persistent_data_can_timeout_count++;
	  debug_data_ecb.can_timeout = persistent_data_can_timeout_count;
	}
	local_data_mirror[n].connection_timeout = 0xFFFF;
	etm_can_master_all_boards_connected = 0x0000;
      } else {
	if (local_data_mirror[n].connection_timeout == 0xFFFF) {
	  // This board has just connected
	  SendToEventLog(LOG_ID_SLAVE_CONNECTION_ESTABLISHED_BASE_ID + n);
	}
	local_data_mirror[n].connection_timeout = 0;
      }
    }
    
    ignore >>= 1;
  }
}



static void LocalReceiveLogData(void) {
  /*
    Logging data is not used by the ECB.
    It is only used to generate data that is sent to the GUI
  */
  
  ETMCanMessage          next_message;
  unsigned int           data_log_index;
  unsigned int           board_id;
  unsigned int           log_id;
  
  while (ETMCanBufferNotEmpty(&etm_can_master_rx_data_log_buffer)) {
    ETMCanReadMessageFromBuffer(&etm_can_master_rx_data_log_buffer, &next_message);
    data_log_index = next_message.identifier;
    data_log_index >>= 2;
    data_log_index &= 0x03FF;
    board_id = data_log_index & 0x000F;
    log_id = data_log_index & 0x03F0;
    log_id >>= 4;


    if (log_id <= 0x0F) {
      // Pulse by Pulse logging
      PulseDataLog(log_id, board_id, next_message.word3, next_message.word2, next_message.word1, next_message.word0);
      Nop();
      Nop();
      Nop();

      
    } else if (log_id <= 0x17) {
      // This is board data that is always up to date on the ECB.  In needs to be stored in RAM
      LocalLogDataFromSlave(log_id, board_id, next_message.word3, next_message.word2, next_message.word1, next_message.word0);
      
    } else if (log_id <= 0x1B) {
      ScopeDataLog(log_id<<4, board_id, next_message.word3, next_message.word2, next_message.word1, next_message.word0);
      
    } else if (log_id <= 0x3F) {
      // Board specific Debugging/Calibration Data, only store if this board is selected
      if (board_id == etm_can_active_debugging_board_id) {
	LocalLogDebugDataFromSlave(log_id, board_id, next_message.word3, next_message.word2, next_message.word1, next_message.word0);
      }
    }
  }
}


static void LocalLogDataFromSlave(unsigned int data_log_id, unsigned int board_id, unsigned int data_3, unsigned int data_2, unsigned int data_1, unsigned int data_0) {
  unsigned int *register_to_write;

  data_log_id -= 0x10;
  
  if (board_id > NUMBER_OF_DATA_MIRRORS) {
    // Not a valid RAM address, this would do bad things to the RAM
    debug_data_ecb.can_address_error++;
    return;
  }
  
  if (board_id == ETM_CAN_ADDR_ETHERNET_BOARD) {
    // This is the ECB, can messages shouldn't overwrite ECB Data
    debug_data_ecb.can_address_error++;
    return;
  }

  if (data_log_id > 7) {
    // The data log
    debug_data_ecb.can_address_error++;
    return;
  }
  
  register_to_write = (unsigned int*)&local_data_mirror[board_id].log_data;
  register_to_write += 4*data_log_id;

  *register_to_write = data_0;
  register_to_write++;

  *register_to_write = data_1;
  register_to_write++;

  *register_to_write = data_2;
  register_to_write++;
    
  *register_to_write = data_3;
}






static void ScopeDataLog(unsigned int data_log_id, unsigned int board_id, unsigned int data_3, unsigned int data_2, unsigned int data_1, unsigned int data_0) {
  unsigned int location;
  unsigned int n;
  if (data_log_id == ETM_CAN_DATA_LOG_REGISTER_PULSE_SCOPE_DATA) {
    if (board_id == ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD) {
      // We are receiving magnetron current data
      location = (data_3 >> 12) & 0x000F;
      if (location > 9) {
	// there is no place to put the data
	return;
      }

      if (scope_data_magnetron_current.data_status == SCOPE_DATA_FULL) {
	// Still waitiing to send the data
	return;
      }

      if (location == 0) {
	// FORMAT THE DATA
	for (n = 0; n < 10; n++) {
	  scope_data_magnetron_current.pulse_data[n*4 + 0] = 0xFFEF;
	  scope_data_magnetron_current.pulse_data[n*4 + 1] = 0xEFFE;
	  scope_data_magnetron_current.pulse_data[n*4 + 2] = 0xFEFF;
	  scope_data_magnetron_current.pulse_data[n*4 + 3] = 0x0FEF;
	}
	scope_data_magnetron_current.data_status = SCOPE_DATA_FILLING;
      }
      
      if (scope_data_magnetron_current.data_status == SCOPE_DATA_FILLING) {
	scope_data_magnetron_current.pulse_data[location*4 + 0] = data_0;
	scope_data_magnetron_current.pulse_data[location*4 + 1] = data_1;
	scope_data_magnetron_current.pulse_data[location*4 + 2] = data_2;
	scope_data_magnetron_current.pulse_data[location*4 + 3] = data_3;// & 0x0FFF);
      }

      if (location == 9) {
	scope_data_magnetron_current.data_status = SCOPE_DATA_FULL;
      }  
    } else {
      // Assume target current Data
      location = (data_3 >> 12) & 0x000F;
      if (location > 9) {
	// there is no place to put the data
	return;
      }

      if (scope_data_target_current.data_status == SCOPE_DATA_FULL) {
	// Still waitiing to send the data
	return;
      }

      if (location == 0) {
	// FORMAT THE DATA

	for (n = 0; n < 10; n++) {
	  scope_data_target_current.pulse_data[n*4 + 0] = 0xFFEF;
	  scope_data_target_current.pulse_data[n*4 + 1] = 0xEFFE;
	  scope_data_target_current.pulse_data[n*4 + 2] = 0xFEFF;
	  scope_data_target_current.pulse_data[n*4 + 3] = 0x0FEF;
	}
	scope_data_target_current.data_status = SCOPE_DATA_FILLING;
      }
      
      if (scope_data_target_current.data_status == SCOPE_DATA_FILLING) {
	scope_data_target_current.pulse_data[location*4 + 0] = data_0;
	scope_data_target_current.pulse_data[location*4 + 1] = data_1;
	scope_data_target_current.pulse_data[location*4 + 2] = data_2;
	scope_data_target_current.pulse_data[location*4 + 3] = data_3;// & 0x0FFF);
      }

      if (location == 9) {
	scope_data_target_current.data_status = SCOPE_DATA_FULL;
      }  

    }
  } else if ((data_log_id == ETM_CAN_DATA_LOG_REGISTER_SCOPE_A) && !scope_data_a.hv_vmon_enabled) {
    scope_data_a.data[scope_data_a.write_location++] = data_0;
    scope_data_a.data[scope_data_a.write_location++] = data_1;
    scope_data_a.data[scope_data_a.write_location++] = data_2;
    scope_data_a.data[scope_data_a.write_location++] = data_3;
    scope_data_a.write_location &= (SCOPE_DATA_SIZE - 1);

    if (scope_data_a.write_location == (SCOPE_DATA_SIZE >> 1)) {
      // We just finished filling the first half of the data, send out that data
      scope_data_a.data_x_ready_to_send = 1;
    }
    
    if (scope_data_a.write_location == 0) {
      // We just finished filling the second half of the register, send out that data
      scope_data_a.data_y_ready_to_send = 1;
    }
    
    
  } else if ((data_log_id == ETM_CAN_DATA_LOG_REGISTER_SCOPE_B) && !scope_data_a.hv_vmon_enabled) {
    scope_data_b.data[scope_data_b.write_location++] = data_0;
    scope_data_b.data[scope_data_b.write_location++] = data_1;
    scope_data_b.data[scope_data_b.write_location++] = data_2;
    scope_data_b.data[scope_data_b.write_location++] = data_3;
    scope_data_b.write_location &= (SCOPE_DATA_SIZE - 1);
    
    if (scope_data_b.write_location == (SCOPE_DATA_SIZE >> 1)) {
      // We just finished filling the first half of the data, send out that data
      scope_data_b.data_x_ready_to_send = 1;
    }
    
    if (scope_data_b.write_location == 0) {
      // We just finished filling the second half of the register, send out that data
      scope_data_b.data_y_ready_to_send = 1;
    }
  } else if ((data_log_id == ETM_CAN_DATA_LOG_REGISTER_HV_VMON_DATA) && scope_data_a.hv_vmon_enabled) {
    // HV VMON data is stored in scope_data_a and scope_data_b
    if (scope_data_a.hv_vmon_buffer_active) {
      // store the data in scope_data_a
      if (scope_data_a.hv_vmon_ready_to_send == 0) {
	// Otherwise the buffer is full already
	scope_data_a.data[scope_data_a.write_location++] = data_0;
	scope_data_a.data[scope_data_a.write_location++] = data_1;
	scope_data_a.data[scope_data_a.write_location++] = data_2;
	scope_data_a.data[scope_data_a.write_location++] = data_3;
	scope_data_a.write_location &= (SCOPE_DATA_SIZE - 1);

	if (scope_data_a.write_location == 0) {
	  // scope_data_a is full
	  _LATG0 = 1;
	  if (scope_data_b.hv_vmon_ready_to_send == 1) {
	    scope_data_b.priority = 1;
	  }
	  scope_data_a.priority = 0;
	  scope_data_a.hv_vmon_ready_to_send = 1;
	  scope_data_a.hv_vmon_buffer_active = 0;
	}
      }
    } else {
      // store the data in scope_data_b
      if (scope_data_b.hv_vmon_ready_to_send == 0) {
	scope_data_b.data[scope_data_b.write_location++] = data_0;
	scope_data_b.data[scope_data_b.write_location++] = data_1;
	scope_data_b.data[scope_data_b.write_location++] = data_2;
	scope_data_b.data[scope_data_b.write_location++] = data_3;
	scope_data_b.write_location &= (SCOPE_DATA_SIZE - 1);
	
	if (scope_data_b.write_location == 0) {
	  // scope_data_b is full
	  _LATG0 = 1;
	  if (scope_data_a.hv_vmon_ready_to_send == 1) {
	    scope_data_a.priority = 1;
	  }
	  scope_data_b.priority = 0;
	  scope_data_b.hv_vmon_ready_to_send = 1;
	  scope_data_a.hv_vmon_buffer_active = 1;
	}
      }
    }
  }
}


static void LocalLogDebugDataFromSlave(unsigned int data_log_id, unsigned int board_id, unsigned int data_3, unsigned int data_2, unsigned int data_1, unsigned int data_0) {
  unsigned int *register_to_write;


  if (data_log_id > 0x37) {
    return;
  }

  if (data_log_id < 0x1C) {
    return;
  }
  
  if (board_id > NUMBER_OF_DATA_MIRRORS) {
    // Not a valid RAM address, this would do bad things to the RAM
    // DPAKER Increment some fault counter;
    debug_data_ecb.can_address_error++;
    return;
  }

  if (board_id == ETM_CAN_ADDR_ETHERNET_BOARD) {
    // This is the ECB, can messages shouldn't overwrite ECB Data
    // DPAKER Increment some fault counter;
    debug_data_ecb.can_address_error++;
    return;
  }

  data_log_id -= 0x1C;
  
  register_to_write = (unsigned int*)&debug_data_slave_mirror;
  register_to_write += data_log_id*4;

  
  *register_to_write = data_3;

  register_to_write++;
  *register_to_write = data_2;

  register_to_write++;
  *register_to_write = data_1;

  register_to_write++;
  *register_to_write = data_0;
  /*
    DPARKER WORK out the register address offset base on the board ID AND Log ID

  */
  
}


void ETMCanMasterClearECBDebug(void) {
  LocalClearDebug();
}


static void LocalClearDebug(void) {
  unsigned int n;
  unsigned int *reset_data_ptr;

  reset_data_ptr = (unsigned int*)&debug_data_ecb;

  for (n = 0; n < (sizeof(debug_data_ecb)>>1); n++) {
    *reset_data_ptr = 0;
    reset_data_ptr++;
  }
  // DPARKER reload any data from initialization????

}


static void LocalUpdateSlaveNotReady(void) {
  unsigned int all_slaves_ready = 0xFFFF;
  unsigned int ignore;
  unsigned int n;
  
  ignore = etm_can_master_boards_to_ignore;
  
  for(n = 0; n < NUMBER_OF_DATA_MIRRORS; n++) {
    if (ignore & 0x0001) {
      // We are ignoring this board
    } else {
      if(local_data_mirror[n].status.control_notice_bits.control_not_ready) {
	all_slaves_ready = 0x0000;
      }
    }
    ignore >>= 1;
  }
  
  // Disable the Can Interrupt
  _C1IE = 0;
  _C2IE = 0;
  
  can_master_all_slaves_ready = all_slaves_ready;
  
  // Reenable the relevant can interrupt
  if (CXEC_ptr == &C1EC) {
    // We are using CAN1.
    _C1IE = 1;
  } else {
    _C2IE = 1;
  }
  
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


void DoCanInterrupt(void) {
  ETMCanMessage can_message;

  debug_data_ecb.CXINTF_max |= *CXINTF_ptr;

  if (*CXRX0CON_ptr & BUFFER_FULL_BIT) {
    /*
      A message has been received in Buffer Zero
    */
    if (!(*CXRX0CON_ptr & FILTER_SELECT_BIT)) {
      // The command was received by Filter 0
      // This is unused at this time
      // Someday it will be used by RTSP response
      debug_data_ecb.can_rx_0_filt_0++;
      
    } else {
      // The commmand was received by Filter 1
      // This is a status message from a slave
      // It should get executed Immediately

      ClrWdt();
      ETMCanRXMessage(&can_message, CXRX0CON_ptr);
      LocalReceiveSlaveStatus(&can_message);
      debug_data_ecb.can_rx_0_filt_1++;
    }
    *CXINTF_ptr &= RX0_INT_FLAG_BIT; // Clear the RX1 Interrupt Flag
  }
  
  if (*CXRX1CON_ptr & BUFFER_FULL_BIT) { 
    /* 
       A message has been recieved in Buffer 1
       This is a data log message
    */
    debug_data_ecb.can_rx_1_filt_2++;
    ETMCanRXMessageBuffer(&etm_can_master_rx_data_log_buffer, CXRX1CON_ptr);
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
    *CXINTF_ptr &= ~ERROR_FLAG_BIT; // Clear the ERR Flag
  } else {
    // FLASH THE CAN LED
    /*
      DPARKER FIX
    if (ETMReadPinLatch(etm_can_master_can_led)) {
      ETMClearPin(etm_can_master_can_led);
    } else {
      ETMSetPin(etm_can_master_can_led);
    }
    */
  }
}



// Functions to be used by the ECB to check status of BOARDS.

unsigned int ETMCanMasterCheckAllBoardsConnected(void) {
  return etm_can_master_all_boards_connected;
}


unsigned int ETMCanMasterCheckAllBoardsConfigured(void) {
  unsigned int ignore;
  unsigned int n;

  // First check that all the boards are connected
  if (etm_can_master_all_boards_connected == 0) {
    return 0;
  }
  ignore = etm_can_master_boards_to_ignore;
  
  for(n = 0; n < NUMBER_OF_DATA_MIRRORS; n++) {
    if ((ignore & 0x0001) == 0) {
      // We are not ignoring this board
      if (local_data_mirror[n].status.control_notice_bits.control_not_configured) {
	return 0;
      }
    }
    ignore >>= 1;
  }
  return 0xFFFF;
}


unsigned int ETMCanMasterCheckSlaveFault(void) {
  unsigned int ignore;
  unsigned int n;
  
  // First check that all the boards are connected
  if (etm_can_master_all_boards_connected == 0) {
    return 0xFFFF;
  }
  ignore = etm_can_master_boards_to_ignore;
  
  for(n = 0; n < NUMBER_OF_DATA_MIRRORS; n++) {
    if ((ignore & 0x0001) == 0) {
      // We are not ignoring this board
      if (*(unsigned int*)&local_data_mirror[n].status.fault_bits) {
	return 0xFFFF;
      }
    }
    ignore >>= 1;
  }
  return 0x0000;
}



unsigned int ETMCanMasterCheckSlaveConfigured(unsigned int board_id) {
  unsigned int ignore;

  if (board_id >= NUMBER_OF_DATA_MIRRORS) {
    return 0x0000;
  }

  ignore = etm_can_master_boards_to_ignore;
  ignore >>= board_id;
  if ((ignore & 0x0001) == 0) {
    // We are not ignoring this board
    if (local_data_mirror[board_id].status.control_notice_bits.control_not_configured) {
      return 0;
    }
    if (local_data_mirror[board_id].connection_timeout) {
      return 0;
    }
  }
  return 0xFFFF;
}



unsigned int ETMCanMasterCheckAllSlavesReady(void) {
  // This needs to be fast
  return can_master_all_slaves_ready;
}


unsigned int ETMCanMasterCheckSlaveReady(unsigned int board_id) {
  if (board_id >= NUMBER_OF_DATA_MIRRORS) {
    return 0x0000;
  }
  
  if ((etm_can_master_boards_to_ignore >> board_id) & 0x0001) {
    // we are ignoring this board
    return 0xFFFF;
  }
  
  if (local_data_mirror[board_id].status.control_notice_bits.control_not_ready) {
    return 0x0000;
  }

  return 0xFFFF;
}

unsigned int ETMCanMasterReturnSlaveStatusBit(unsigned int bit_select, unsigned int default_value) {
  unsigned int board_id;
  unsigned int register_id;
  unsigned int bit_id;
  unsigned int data;


  bit_id      = bit_select & 0xF;
  bit_select  >>= 4;
  register_id = bit_select & 0xF;
  bit_select  >>= 4;
  board_id    = bit_select & 0xF;

  if (board_id >= NUMBER_OF_DATA_MIRRORS) {
    return default_value;
  }
  
  if ((etm_can_master_boards_to_ignore >> board_id) & 0x0001) {
    // we are ignoring this board
    return default_value;
  }

  if (register_id >= 4) {
    // status register is only 4 words, this is an error
    return default_value;
  }
  
  data = *(unsigned int*)(&local_data_mirror[board_id] + register_id);
  if ((data >> bit_id) & 0x0001) {
    // the bit was set
    return 0xFFFF;
  }

  return 0x0000;
}



void ETMCanMasterSetScaledMagnetronHeaterCurrent(unsigned int scaled_current_setting) {
  etm_can_magnetron_heater_scaled_heater_current = scaled_current_setting;
}


void ETMCanMasterSelectScopeDataSourceGeneric(unsigned int scope_a_data_source, unsigned int scope_b_data_source) {
  // DPARKER, use this command to select the scope data sources

  scope_a_settings = scope_a_data_source;
  scope_b_settings = scope_b_data_source;
  scope_hv_vmon_settings = 0xFFFF;
  
  scope_data_a.data_x_ready_to_send = 0;
  scope_data_a.data_y_ready_to_send = 0;
  scope_data_a.hv_vmon_enabled = 0;
  scope_data_a.write_location = 0;

  scope_data_b.data_x_ready_to_send = 0;
  scope_data_b.data_y_ready_to_send = 0;
  scope_data_b.hv_vmon_enabled = 0;
  scope_data_b.write_location = 0;
  
}

void ETMCanMasterSelectScopeDataSourceHVVmon(unsigned int hv_vmon_source) {
  scope_a_settings = 0xFFFF;
  scope_b_settings = 0xFFFF;
  scope_hv_vmon_settings = hv_vmon_source;

  scope_data_a.data_x_ready_to_send = 0;
  scope_data_a.data_y_ready_to_send = 0;
  scope_data_a.hv_vmon_enabled = 1;
  scope_data_a.hv_vmon_buffer_active = 1;
  scope_data_a.hv_vmon_ready_to_send = 0;
  scope_data_a.write_location = 0;

  scope_data_b.data_x_ready_to_send = 0;
  scope_data_b.data_y_ready_to_send = 0;
  scope_data_b.hv_vmon_enabled = 1;
  scope_data_b.hv_vmon_buffer_active = 1;
  scope_data_b.hv_vmon_ready_to_send = 0;
  scope_data_b.write_location = 0;
}
