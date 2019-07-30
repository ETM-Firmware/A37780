typedef struct {
  unsigned char pulse_count;
  unsigned char status_bits;
  unsigned int  time_data;   // DPARKER TIME DATA.  Some part of ETM TICK  (strech to 32 bits???)
  unsigned int  data_0;      // Trigger info
  unsigned int  data_1;      // Trigger info
  unsigned int  data_2;      // HVPS - Pre-Pulse Voltage
  unsigned int  data_3;      // AFC Current Position
  unsigned int  data_4;      // AFC Analog Readings (16 bit reverse power or 2x 8 bit)
  unsigned int  data_5;      // Magnetron Current Reading
  unsigned int  data_6;      // Magnetron Current Integral Reading
  unsigned int  data_7;      // Target Current Reading
  unsigned int  data_8;      // Target Current Integral Reading
  unsigned int  data_9;      // Gun Driver Something?
  unsigned int  data_10;     // Gun Driver Something
  unsigned int  data_11;     // Spare??
  unsigned int  data_12;     // Spare??
  unsigned int  data_13;     // Spare??
} TYPE_PULSE_ENTRY;

#define PULSE_LOG_BUFFER_SIZE 16
extern TYPE_PULSE_ENTRY pulse_log_data_buffer_a[PULSE_LOG_BUFFER_SIZE];
extern TYPE_PULSE_ENTRY pulse_log_data_buffer_b[PULSE_LOG_BUFFER_SIZE];
