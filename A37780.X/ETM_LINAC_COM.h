#ifndef __ETM_LINAC_COM_H
#define __ETM_LINAC_COM_H


typedef struct {
  unsigned int  data_a[128];
  unsigned int  data_b[128];
  unsigned      data_a_ready_to_send:1;
  unsigned      data_a_sent:1;
  unsigned      data_b_ready_to_send:1;
  unsigned      data_b_sent:1;
  unsigned      unused:4;
  unsigned char scope_type;
} TYPE_SCOPE_DATA;






typedef struct {
  unsigned char pulse_count_a;
  unsigned char status_bits_a_and_b;
  unsigned long trigger_time_a;

  unsigned int  gun_trigger_width_a;
  unsigned int  gun_trigger_start_a;
  unsigned int  gun_trigger_width_b;
  unsigned int  gun_trigger_start_b;
  
  unsigned int  hvps_eoc_a;
  unsigned int  hvps_spare_a;
  unsigned int  hvps_eoc_b;
  unsigned int  hvps_spare_b;

  unsigned int  afc_current_position_a;
  unsigned int  afc_reverse_reading_a;
  unsigned int  afc_current_position_b;
  unsigned int  afc_reverse_reading_b;

  unsigned int  magnetron_current_sample_a;
  unsigned int  magnetron_current_integral_a;
  unsigned int  magnetron_current_sample_b;
  unsigned int  magnetron_current_integral_b;

  unsigned int  target_current_sample_a;
  unsigned int  target_current_integral_a;
  unsigned int  target_current_sample_b;
  unsigned int  target_current_integral_b;

  unsigned int  gun_driver_data_0_a;
  unsigned int  gun_driver_data_1_a;
  unsigned int  gun_driver_data_0_b;
  unsigned int  gun_driver_data_1_b;

} TYPE_PULSE_DATA;
// Size 27 Words

#define PULSE_LOG_BUFFER_ENTRIES 8

typedef struct {
  TYPE_PULSE_DATA data_a[PULSE_LOG_BUFFER_ENTRIES];
  TYPE_PULSE_DATA data_b[PULSE_LOG_BUFFER_ENTRIES];
  unsigned        data_a_ready_to_send:1;
  unsigned        data_b_ready_to_send:1;
  unsigned        unused:6;
  unsigned char   spare;
} TYPE_PULSE_LOG;




extern TYPE_SCOPE_DATA scope_data;
extern TYPE_PULSE_LOG  pulse_log;



#endif
