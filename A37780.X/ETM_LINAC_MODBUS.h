#ifndef __ETM_LINAC_MODBUS_H
#define __ETM_LINAC_MODBUS_H


typedef struct {
  unsigned int index ;                  // command index
  unsigned int data_3;
  unsigned int data_2;
  unsigned int data_1;
  unsigned int data_0;
} ETMEthernetMessageFromGUI;


ETMEthernetMessageFromGUI GetNextMessageFromGUI(void);

#define SEND_BUFFER_A            1
#define SEND_BUFFER_B            0

void SendPulseData(unsigned int buffer_select);

void ETMLinacModbusUpdate(void);

void ETMLinacModbusInitialize(void);

void SendToEventLog(unsigned int event_type);

//void ETMLinacModusGetActiveDebuggingID(void);

#endif
