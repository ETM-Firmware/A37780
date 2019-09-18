#ifndef __ETM_LINAC_COM_H
#define __ETM_LINAC_COM_H

extern unsigned int etm_can_active_debugging_board_id;



#define SCOPE_DATA_SIZE  256

typedef struct {
  unsigned int  data[SCOPE_DATA_SIZE];
  unsigned      data_x_ready_to_send:1;
  unsigned      data_y_ready_to_send:1;
  unsigned      hv_vmon_enabled:1;
  unsigned      hv_vmon_buffer_active:1;
  unsigned      hv_vmon_ready_to_send:1;
  unsigned      priority:1;
  unsigned      unused:2;
  unsigned char write_location;
} TYPE_SCOPE_DATA;


typedef struct {
  unsigned int pulse_data[40];
  unsigned int data_status;  // 0 = empty,  20 = full
} TYPE_SCOPE_PULSE_DATA;


#define SCOPE_DATA_FULL    20
#define SCOPE_DATA_EMPTY   0
#define SCOPE_DATA_FILLING 10

extern TYPE_SCOPE_PULSE_DATA scope_data_magnetron_current;
extern TYPE_SCOPE_PULSE_DATA scope_data_target_current;

extern TYPE_SCOPE_DATA scope_data_a;
extern TYPE_SCOPE_DATA scope_data_b;



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
