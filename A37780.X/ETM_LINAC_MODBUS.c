#include <xc.h>
#include "TCPmodbus.h"
#include "A37780.h"
#include "ETM_TICK.h"
#include "ETM_LINAC_MODBUS.h"


#include "ETM_LINAC_COM.h"


#include "TCPmodbus.h"

#include <string.h>
#include "ETM_IO_PORTS.h"  //DPARKER Fix this
#include "ETM_LINAC_COM.h"


unsigned int password_seed;
unsigned int etm_password;


unsigned int sync_time_10ms_units;

typedef struct {
  unsigned int event_time;   // This is the lower 16 bit of the second counter, GUI will have to re-align the higher 16 bits
  unsigned int event_id;     // This tells what the event was
  //unsigned int event_data_a; // Additional Data about the event
  //unsigned int event_data_b; // Additional Data about the event
} TYPE_EVENT;


#define EVENT_LOG_SIZE 64
typedef struct {
  TYPE_EVENT event_data[EVENT_LOG_SIZE];  // DPARKER TEST IF WE CAN DROP THIS TO 32
  unsigned int write_index;
  unsigned int read_index;
} TYPE_EVENT_LOG;

TYPE_EVENT_LOG event_log;

//TYPE_PULSE_ENTRY pulse_log_data_buffer_a[PULSE_LOG_BUFFER_SIZE];
//TYPE_PULSE_ENTRY pulse_log_data_buffer_b[PULSE_LOG_BUFFER_SIZE];


void SendToEventLog(unsigned int log_id) {
  event_log.event_data[event_log.write_index].event_time   = sync_time_10ms_units;
  event_log.event_data[event_log.write_index].event_id     = log_id;
  //event_log.event_data[event_log.write_index].event_data_a = 0;
  //event_log.event_data[event_log.write_index].event_data_b = 0;
  event_log.write_index++;
  event_log.write_index &= (EVENT_LOG_SIZE-1);
  if (event_log.write_index == event_log.read_index) {
    // The event log is full, over write th old events
    event_log.read_index++;
    event_log.read_index &= (EVENT_LOG_SIZE-1);
  }
}



static void AddMessageFromGUI(unsigned char * buffer_ptr);
static unsigned int NewMessageInEventLog(void);
static unsigned int EventLogMessageSize(void);
static unsigned char GetNextSendIndex(void);
static void PrepareTXMessage(ETMModbusTXData *tx_data, unsigned char data_type);

static unsigned int GeneratePassword(unsigned int password_seed);

enum {
  MODBUS_WR_CYCLE_START,
  MODBUS_WR_HVLAMBDA, 	
  MODBUS_WR_ION_PUMP,
  MODBUS_WR_AFC,
  MODBUS_WR_COOLING,
  MODBUS_WR_HTR_MAGNET,
  MODBUS_WR_GUN_DRIVER,
  MODBUS_WR_MAGNETRON_CURRENT,
  MODBUS_WR_TARGET_CURRENT,
  MODBUS_WR_DOSE_MONITOR,
  MODBUS_WR_PFN_BOARD,
  MODBUS_WR_ETHERNET,
  MODBUS_WR_DEBUG_DATA,
  MODBUS_WR_EVENTS,
  MODBUS_WR_CYCLE_STOP,
  MODBUS_WR_PULSE_LOG,
  MODBUS_WR_SCOPE_A,
  MODBUS_WR_SCOPE_B,
  MODBUS_WR_SCOPE_HV,
  MODBUS_WR_SCOPE_MAGNETRON_CURRENT,
  MODBUS_RD_COMMAND_DETAIL,
};


#define ETH_GUI_MESSAGE_BUFFER_SIZE   16
ETMEthernetMessageFromGUI    eth_message_from_GUI[ETH_GUI_MESSAGE_BUFFER_SIZE];


static unsigned char         last_index_sent = 0;  // DPARKER why is this global
static unsigned char         modbus_command_request = 0;  /* how many commands from GUI */
static unsigned char         eth_message_from_GUI_put_index;
static unsigned char         eth_message_from_GUI_get_index;

// DPARKER work on high speed logging commands
static unsigned char         pulse_log_buffer_select;
static unsigned char         pulse_log_ready_to_send = 0;

// This is used to time the "standard" ethernet messages at 1 per 100mS
unsigned long timer_write_holding_var;

#define HEADER_LENGTH_CHAR 12


static void AddMessageFromGUI(unsigned char * buffer_ptr) {
  
  if (((eth_message_from_GUI_put_index + 1) & 0xF) == eth_message_from_GUI_get_index) {
    // The command buffer is full
    // Drop this message??
    // DPARKER what to do here
    // DPARKER ADD DEBUGGING INFORMATION HERE
    return;
  }
  
  eth_message_from_GUI[eth_message_from_GUI_put_index].index = (*buffer_ptr << 8) | *(buffer_ptr + 1);
  eth_message_from_GUI[eth_message_from_GUI_put_index].data_3 = (*(buffer_ptr + 2) << 8) | *(buffer_ptr + 3);
  eth_message_from_GUI[eth_message_from_GUI_put_index].data_2 = (*(buffer_ptr + 4) << 8) | *(buffer_ptr + 5);
  eth_message_from_GUI[eth_message_from_GUI_put_index].data_1 = (*(buffer_ptr + 6) << 8) | *(buffer_ptr + 7);
  eth_message_from_GUI[eth_message_from_GUI_put_index].data_0 = (*(buffer_ptr + 8) << 8) | *(buffer_ptr + 9);
  
  eth_message_from_GUI_put_index++;
  eth_message_from_GUI_put_index &= 0xF;
}


/****************************************************************************
  Function:
    ETMEthernetMessageFromGUI GetNextMessage(void)

  Input:
    pointer to data
    
  Description:
  Remarks:
    None
***************************************************************************/
ETMEthernetMessageFromGUI GetNextMessageFromGUI(void) {
  ETMEthernetMessageFromGUI command;
  
  if (eth_message_from_GUI_put_index == eth_message_from_GUI_get_index)  {
    // The message queue is empty
    command.index = 0xFFFF;
  } else {
    command = eth_message_from_GUI[eth_message_from_GUI_get_index]; 
    eth_message_from_GUI_get_index++;
    eth_message_from_GUI_get_index &= 0xF;
  }
  
  return (command);    
}



static unsigned int NewMessageInEventLog(void) {
  if (event_log.read_index == event_log.write_index) {
    return 0;
  }
  return 1;
}


static unsigned int EventLogMessageSize(void) {
  unsigned int events_to_send = 0;

  if (event_log.read_index > event_log.write_index) {
    events_to_send = EVENT_LOG_SIZE - event_log.read_index; 
  } else {
    events_to_send = event_log.write_index - event_log.read_index;
  }
  if (events_to_send >= (EVENT_LOG_SIZE >> 1)) {
    // Max of half of event Log Size
    events_to_send = EVENT_LOG_SIZE >> 1;
  }	
  
  // Update the read_index
  event_log.read_index += events_to_send;
  event_log.read_index &= (EVENT_LOG_SIZE - 1);

  return (events_to_send << 2);
}


// DPARKER - figure out pulse data
void SendPulseData(unsigned int buffer_select) {
  pulse_log_buffer_select = buffer_select;
  pulse_log_ready_to_send = 1;
}


static unsigned char GetNextSendIndex(void) {
  static unsigned char scheduled_modbus_message_counter = 0;

  scheduled_modbus_message_counter++;
  if (scheduled_modbus_message_counter >= MODBUS_WR_CYCLE_STOP) {
    scheduled_modbus_message_counter = MODBUS_WR_CYCLE_START;
    scheduled_modbus_message_counter++;
  }
  
  return scheduled_modbus_message_counter;
}


#define SIZE_BOARD_MIRROR    (sizeof(ETMCanBoardData) - 6) // 72
#define SIZE_DEBUG_DATA      sizeof(ETMCanBoardDebuggingData)
#define SIZE_ECB_DATA        sizeof(TYPE_ECB_DATA)

static void PrepareTXMessage(ETMModbusTXData *tx_data, unsigned char data_type) {
  // DPARKER what is pulse_index for
  static unsigned char pulse_index = 0;        // index for eash tracking
  static unsigned transaction_number = 0; // Index for each transaction


  tx_data->data_length = 0;
  tx_data->tx_ready = 0;
  
  tx_data->header_length = HEADER_LENGTH_CHAR;
  

  switch (data_type)
    {
    case MODBUS_WR_HVLAMBDA:
      tx_data->data_ptr = (unsigned char *)&local_data_mirror[ETM_CAN_ADDR_HV_LAMBDA_BOARD];
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;

    case MODBUS_WR_ION_PUMP:
      tx_data->data_ptr = (unsigned char *)&local_data_mirror[ETM_CAN_ADDR_ION_PUMP_BOARD];
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;
    
    case MODBUS_WR_AFC:
      tx_data->data_ptr = (unsigned char *)&local_data_mirror[ETM_CAN_ADDR_AFC_CONTROL_BOARD];
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;
      
    case MODBUS_WR_COOLING:
      tx_data->data_ptr = (unsigned char *)&local_data_mirror[ETM_CAN_ADDR_COOLING_INTERFACE_BOARD];
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;
    
    case MODBUS_WR_HTR_MAGNET:
      tx_data->data_ptr = (unsigned char *)&local_data_mirror[ETM_CAN_ADDR_HEATER_MAGNET_BOARD];
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;

    case MODBUS_WR_GUN_DRIVER:
      tx_data->data_ptr = (unsigned char *)&local_data_mirror[ETM_CAN_ADDR_GUN_DRIVER_BOARD];
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;
    
    case MODBUS_WR_MAGNETRON_CURRENT:
      tx_data->data_ptr = (unsigned char *)&local_data_mirror[ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD];
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;

    case MODBUS_WR_TARGET_CURRENT:
      tx_data->data_ptr = (unsigned char *)&local_data_mirror[ETM_CAN_ADDR_TARGET_CURRENT_BOARD];
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;

    case MODBUS_WR_DOSE_MONITOR:
      tx_data->data_ptr = (unsigned char *)&local_data_mirror[ETM_CAN_ADDR_TARGET_CURRENT_BOARD];
      tx_data->data_length = 0;
      tx_data->tx_ready = 0;
      break;

    case MODBUS_WR_PFN_BOARD:
      tx_data->data_ptr = (unsigned char *)&local_data_mirror[ETM_CAN_ADDR_TARGET_CURRENT_BOARD];
      tx_data->data_length = 0;
      tx_data->tx_ready = 0;
      break;
      
    case MODBUS_WR_ETHERNET:
      tx_data->data_ptr = (unsigned char *)&ecb_data;
      tx_data->data_length = SIZE_ECB_DATA;
      tx_data->tx_ready = 1;
      break;
    
    case MODBUS_WR_DEBUG_DATA:
      if (etm_can_active_debugging_board_id == ETM_CAN_ADDR_ETHERNET_BOARD) {
	tx_data->data_ptr = (unsigned char *)&debug_data_ecb;
      } else {
	tx_data->data_ptr = (unsigned char *)&debug_data_slave_mirror;
      }
      tx_data->data_length = SIZE_DEBUG_DATA;
      tx_data->tx_ready = 1;
      break;
      
      // DPARKER WILL THIS TRANSMIT WRAP PROPERLY????
    case MODBUS_WR_EVENTS:
      tx_data->tx_ready = 0;
      if (NewMessageInEventLog()) {
	tx_data->data_ptr = (unsigned char *)&event_log.event_data[event_log.read_index];
	tx_data->data_length = EventLogMessageSize();
	tx_data->tx_ready = 1;
      }
      break;

    case MODBUS_WR_PULSE_LOG:
      // DPARKER - Test the pulse log
      // DPARKER - I Don't think that pulse index is needed
      pulse_index++;  // overflows at 255
      if (pulse_log.data_a_ready_to_send) {
	tx_data->data_ptr = (unsigned char *)&pulse_log.data_a;
	pulse_log.data_a_ready_to_send = 0;
      } else {
	tx_data->data_ptr = (unsigned char *)&pulse_log.data_b;
	pulse_log.data_b_ready_to_send = 0;
      }
      tx_data->data_length = PULSE_LOG_BUFFER_ENTRIES * sizeof(TYPE_PULSE_DATA);
      tx_data->tx_ready = 1;
      break;

    case MODBUS_WR_SCOPE_A:
      tx_data->data_length = SCOPE_DATA_SIZE;
      if (scope_data_a.data_x_ready_to_send) {
	tx_data->data_ptr = (unsigned char *)&scope_data_a.data[0];
	scope_data_a.data_x_ready_to_send = 0;
      } else {
	tx_data->data_ptr = (unsigned char *)&scope_data_a.data[SCOPE_DATA_SIZE>>1];
	scope_data_a.data_y_ready_to_send = 0;
      }
      tx_data->tx_ready = 1;
      break;

    case MODBUS_WR_SCOPE_B:
      tx_data->data_length = SCOPE_DATA_SIZE;
      if (scope_data_b.data_x_ready_to_send) {
	tx_data->data_ptr = (unsigned char *)&scope_data_b.data[0];
	scope_data_b.data_x_ready_to_send = 0;
      } else {
	tx_data->data_ptr = (unsigned char *)&scope_data_b.data[SCOPE_DATA_SIZE>>1];
	scope_data_b.data_y_ready_to_send = 0;
      }
      tx_data->tx_ready = 1;
      break;

    case MODBUS_WR_SCOPE_HV:
      _LATG0 = 0;
      _LATC15 = 1;
      tx_data->data_length = SCOPE_DATA_SIZE << 1;

      if (scope_data_a.hv_vmon_ready_to_send && scope_data_b.hv_vmon_ready_to_send) {
	if (scope_data_a.priority) {
	  tx_data->data_ptr = (unsigned char *)&scope_data_a.data[0];
	  scope_data_a.hv_vmon_ready_to_send = 0;
	} else {
	  tx_data->data_ptr = (unsigned char *)&scope_data_b.data[0];
	  scope_data_b.hv_vmon_ready_to_send = 0;
	}
      } else {
	if (scope_data_a.hv_vmon_ready_to_send) {
	  tx_data->data_ptr = (unsigned char *)&scope_data_a.data[0];
	  scope_data_a.hv_vmon_ready_to_send = 0;
	} else {
	  tx_data->data_ptr = (unsigned char *)&scope_data_b.data[0];
	  scope_data_b.hv_vmon_ready_to_send = 0;
	}
      }
      tx_data->tx_ready = 1;
      break;

    case MODBUS_WR_SCOPE_MAGNETRON_CURRENT:
      tx_data->data_length = 80;
      tx_data->data_ptr = (unsigned char *)&scope_data_magnetron_current.pulse_data[0];
      tx_data->tx_ready = 1;
      break;

    case MODBUS_RD_COMMAND_DETAIL:
      tx_data->data_ptr = (unsigned char *)&debug_data_ecb;  // Dummy data location so that we don't crash the processor if this gets executed for some reason
      tx_data->data_length = 0;
      tx_data->tx_ready = 1;
      break;
    
      default: // move to the next for now, ignore some boards
      tx_data->data_length = 0;
      tx_data->data_ptr = (unsigned char *)&debug_data_ecb; // DUMMY LOCATION
      tx_data->tx_ready = 0;
      break;

    }


  if (tx_data->tx_ready) {

    // DPARKER - ADD some ethernet Debugging Info
    
    transaction_number++;
    last_index_sent = data_type;
        
   // Prepare the header message
    tx_data->header_data[0] = (transaction_number >> 8) & 0xff;	    // transaction hi byte
    tx_data->header_data[1] = transaction_number & 0xff;	    // transaction lo byte
    tx_data->header_data[2] = 0;	                            // protocol hi - 0 
    tx_data->header_data[3] = 0;	                            // protocol lo - 0
    tx_data->header_data[4] = ((tx_data->data_length + HEADER_LENGTH_CHAR - 8) >> 8);              // This is the length of data remaining in the message
    tx_data->header_data[5] = (tx_data->data_length + HEADER_LENGTH_CHAR - 8);                       // This is the length of data remaining in the message
    tx_data->header_data[6] = data_type;                              // Unit Identifier - What Type of Data is this 
    if (data_type == MODBUS_RD_COMMAND_DETAIL) {
      tx_data->header_data[7] = 0x3;                                  // function code 0x03 = Read Multiple Holding Registers, 
    } else {
      tx_data->header_data[7] = 0x10;                                 // function code 0x10 = Write Multiple Holding Registers, 
    }
    // DATA STARTS HERE - HOW SHOULD THIS BE FORMATED FOR GENERIC ETM MESSAGES
    tx_data->header_data[8] = (pulse_index >> 8) & 0xff;           // pulse index high word
    tx_data->header_data[9] = pulse_index & 0xff;                  // pulse index low word
    tx_data->header_data[10] = 0;                                  // unused at this time
    tx_data->header_data[11] = 0;                                  // unused at this time
    
    
    /*
    // This header data is not sent out
    tx_data->header_data[10] = 0;//tx_data->data_length >> 9;                     // msg length in words hi
    tx_data->header_data[11] = 0;//tx_data->data_length >> 1;                     // msg length in words lo
    tx_data->header_data[12] = (tx_data->data_length +15) & 0xff;                   // data length in bytes // DPARKER is this used???
    tx_data->header_data[13] = (pulse_index >> 8) & 0xff;           // pulse index high word
    tx_data->header_data[14] = pulse_index & 0xff;                  // pulse index low word
    
    
    if (data_type == MODBUS_RD_COMMAND_DETAIL) {
      tx_data->header_data[9] = 0;//MODBUS_RD_COMMAND_DETAIL;
      tx_data->header_data[10] = 0;                     // msg length in words hi
      tx_data->header_data[11] = 4;                     // msg length in words lo
      tx_data->header_data[12] = 0;                   // data length in bytes // DPARKER is this used???
    }
    */
  }
}



void ETMModbusApplicationSpecificTXData(ETMModbusTXData* tx_data_to_send) {
  //ETMModbusTXData data_to_send;
  unsigned char send_message;
  unsigned char modbus_tx_index;
  // See if there are any high speed messages to be sent

  send_message = 0;
  if (_LATA6) {
    _LATA6 = 0;
  } else {
    _LATA6 = 1;
  }
    
  if (pulse_log.data_a_ready_to_send || pulse_log.data_b_ready_to_send) {
    modbus_tx_index = MODBUS_WR_PULSE_LOG;
    send_message = 1;
    //pulse_log_ready_to_send = 0;
  } else if (0) {
    // FUTURE Event log counter is greater than 32
  } else if (scope_data_a.data_x_ready_to_send || scope_data_a.data_y_ready_to_send) {
    // Transmit scope data a
    modbus_tx_index = MODBUS_WR_SCOPE_A;
    send_message = 1;
  } else if (scope_data_b.data_x_ready_to_send || scope_data_b.data_y_ready_to_send) {
    // Transmit scope data a
    modbus_tx_index = MODBUS_WR_SCOPE_B;
    send_message = 1;
  } else if (scope_data_a.hv_vmon_ready_to_send || scope_data_b.hv_vmon_ready_to_send) {
    // Transmit HV VMon data
    modbus_tx_index = MODBUS_WR_SCOPE_HV;
    send_message = 1;
  } else if (scope_data_magnetron_current.data_status == SCOPE_DATA_FULL) {
    modbus_command_request = 0;
    // DPARKER Transmit the scope data to the GUI
    modbus_tx_index = MODBUS_WR_SCOPE_MAGNETRON_CURRENT;
    send_message = 1;
    scope_data_magnetron_current.data_status = SCOPE_DATA_EMPTY;
  } else if (modbus_command_request) {
    modbus_tx_index = MODBUS_RD_COMMAND_DETAIL;
    send_message = 1;
    modbus_command_request = 0;
  } else {
    // Execute regularly scheduled command - No need to check to see if they were recieved we will resend them again soon enough
    if (ETMTickRunOnceEveryNMilliseconds(100, &timer_write_holding_var)) {
      // 100ms has passed - Send the next Message
      modbus_tx_index = GetNextSendIndex();
      send_message = 1;
    }
  }


  if (send_message) { 
    PrepareTXMessage(tx_data_to_send, modbus_tx_index);
    if (modbus_tx_index != MODBUS_WR_EVENTS) {
      //ETMTCPModbusWaitForResponse();  // Event log is not repeatable so no need to wait for response
    }
  }
  
  //return data_to_send;
}




void ETMModbusApplicationSpecificRXData(unsigned char data_RX[]) {
  
  if (data_RX[6] != last_index_sent) {
    // does not match the sent command
    // DPARKER - increment some sort of error count
    return;
  }

  if (last_index_sent == MODBUS_RD_COMMAND_DETAIL) {
    AddMessageFromGUI(&data_RX[12]);
  } else { 
    /* write commands return command count in the reference field */
    modbus_command_request = (data_RX[8] << 8) | data_RX[9];
  }

  sync_time_10ms_units = (data_RX[8] << 8) + data_RX[11];

  if (password_seed == 0) {
    password_seed = sync_time_10ms_units;
    etm_password = GeneratePassword(password_seed);
  }
  
  etm_can_active_debugging_board_id = data_RX[10];
  if (etm_can_active_debugging_board_id >= NUMBER_OF_DATA_MIRRORS ) {
    etm_can_active_debugging_board_id = ETM_CAN_ADDR_ETHERNET_BOARD;
  }
}



// DPARKER Remove this after can module has been updates
unsigned int SendCalibrationDataToGUI(unsigned int index, unsigned int scale, unsigned int offset) {
  return (0xffff);
}


void ETMLinacModbusUpdate(void) {
  ETMTCPModbusTask();
  _LATC15 = 0;

}


void ETMLinacModbusInitialize(void) {
  IPCONFIG ip_config;
  TYPE_ENCx24J600_CONFIG ENCx24J600_config;
  

  // DPARKER Load this from EEPROM or USE DEFAULT????
  ip_config.remote_ip_addr = 0x0F46A8C0;  // 192.168. 70. 15
  ip_config.ip_addr        = 0x6346A8C0;  // 192.168. 70. 99
  ip_config.mask           = 0x00FFFFFF;  // 255.255.255.  0
  ip_config.gate           = 0x00000000;  //   0.  0.  0.  0
  ip_config.dns            = 0x00000000;  //   0.  0.  0.  0
  ip_config.mac_addr[0]    = 0x00; 
  ip_config.mac_addr[1]    = 0x50; 
  ip_config.mac_addr[2]    = 0xC2; 
  ip_config.mac_addr[3]    = 0xB4; 
  ip_config.mac_addr[4]    = 0x20; 
  ip_config.mac_addr[5]    = 0x00;        //00-50-C2-B4-20-00 
  //ip_config.net_bios_name  = "ETMBoard Test";
  strcpy(ip_config.net_bios_name, "ETMBoard Test");

  ENCx24J600_config.cable_select_pin = _PIN_RG3;
  //                                              ENCx24J600_config.reset_pin = _PIN_RA15;
  ENCx24J600_config.spi_port = TCPMODBUS_USE_SPI_PORT_1;

  // DPARKER make this part of the system configuration
  if (ETMTickNotInitialized()) {
    ETMTickInitialize(FCY_CLK, ETM_TICK_USE_TIMER_1);  
  }

  eth_message_from_GUI_put_index = 0;
  eth_message_from_GUI_get_index = 0; 

  ETMTCPModbusENCx24J600Initialize(&ENCx24J600_config);

  #define CONNECTION_TIMEOUT_MILLISECONDS  5000
  #define RESPONSE_TIMEOUT_MILLISECONDS     500

  ETMTCPModbusInitialize(&ip_config, CONNECTION_TIMEOUT_MILLISECONDS, RESPONSE_TIMEOUT_MILLISECONDS);

  password_seed = 0;
  
}

static unsigned int GeneratePassword(unsigned int password_seed) {
  unsigned int temp;

  temp = password_seed ^ 0x4450;
  return ETMCRCModbus(&temp, 2);
}
