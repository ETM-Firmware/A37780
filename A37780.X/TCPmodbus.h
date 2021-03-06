#ifndef __TCP_MODBUS_H
#define __TCP_MODBUS_H


typedef struct {
  unsigned long remote_ip_addr;
  unsigned long ip_addr;
  unsigned long mask;
  unsigned long gate;
  unsigned long dns;
  unsigned char mac_addr[6];
  char          net_bios_name[16];
} IPCONFIG;

typedef struct {
  unsigned long cable_select_pin;
  unsigned int  spi_port;
} TYPE_ENCx24J600_CONFIG;
#define TCPMODBUS_USE_SPI_PORT_1         1
#define TCPMODBUS_USE_SPI_PORT_2         2


typedef struct {
  unsigned char header_data[24];        // Max header size is 24 bytes
  unsigned char *data_ptr;
  unsigned char tx_ready;
  unsigned char  header_length;
  unsigned int  data_length;
} ETMModbusTXData;


void ETMTCPModbusENCx24J600Initialize(TYPE_ENCx24J600_CONFIG* ENCx24J600_config);
/*
  This is called to initialize the ENC28J60 hardware
*/


void ETMTCPModbusInitialize(IPCONFIG* ip_config, unsigned int connection_timeout_milliseconds, unsigned int response_timeout_milliseconds);
/*
  This is called to initialize the TCP Modbus module
*/


void ETMTCPModbusTask(void);
/*
  This must be called occassionaly to execute the TCP client state machine
*/


void ETMModbusApplicationSpecificTXData(ETMModbusTXData* tx_data_to_send);
/*
  This function must be defined in the user application file.
  It is used to generate the TX data

  Every time the "ETMTCPModbusTask" is called it will run through the TCP client
  If the client is ready to send a message it will call this function
  
  If tx_ready is set when the function returns and there is (header_length + data_length) available in the socket,
  The message will be sent.
*/


#define MAX_RX_SIZE    48                                          // This is the maximum size of recieved data (including header)
void ETMModbusApplicationSpecificRXData(unsigned char data_RX[]);
/*
  This function must be defined in the user application file
  It is used to process the recieved data

  When a message is recieved durring a "ETMTCPModbusTask" cycle, the TCP Client will call this function with the recieved data
  It is the responsiblity of the Application to process that data.
*/


#define ERROR_COUNT_SM_PROCESS_RESPONSE_TIMEOUT 0
#define ERROR_COUNT_SM_SOCKET_OBTAINED_TIMEOUT  1
#define ERROR_SM_PROCESS_RESPONSE_TIMEOUT_ID    2
#define COUNT_SM_SOCKET_OBTAINED_MSG_TX         3
#define COUNT_SM_PROCESS_RESPONSE_MSG_RX        4
#define ERROR_COUNT_SM_DISCONNECT               5
unsigned int ETMTCPModbusGetErrorInfo(unsigned char error);
/*
  This is used to access the debugging variables availabel for the TCPmodbus module
*/

#endif
