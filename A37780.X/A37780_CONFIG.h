#ifndef __A36507_CONFIG_H
#define __A36507_CONFIG_H



#define __SYSTEM_CONFIGURATION_2_5_MEV
//#define __SYSTEM_CONFIGURATION_6_MEV_MEDICAL


#ifdef __SYSTEM_CONFIGURATION_2_5_MEV
#define SOFTWARE_DASH_NUMBER                 251

#define DRIVE_UP_TIMEOUT                     1000 // 10 Seconds
#define MINIMUM_FAULT_HOLD_TIME              50   // .5 Second
#define FAULT_RESET_HOLD_TIME                200  // 2 Second
#define MAX_DRIVE_UP_FAULTS                  5    // Attempts to bring up high voltage before latching fault
#define MAX_HV_ON_FAULTS                     10   // Attempts to reset faults with HV on (but Xray Off) before latching 

#define MAGNETRON_HEATER_WARM_UP_TIME        300 //20 //300   // 5 minutes
#define THYRATRON_WARM_UP_TIME               900 //40 //900   // 15 minutes
#define GUN_DRIVER_HEATER_WARM_UP_TIME       300 //20 //300   // 5 minutes

#define GUN_HEATER_HOLDOFF_AT_STARTUP        500   // 5 seconds
#define GUN_HEATER_ADDITONAL_HOLDOFF_COLD   2500   // 25 seconds


#define DEFAULT_REMOTE_IP_ADDRESS                            0x0F46A8C0  // 192.168.70.15
#define DEFAULT_IP_ADDRESS                                   0x6346A8C0  // 192.168.70.99

#define MAX_SF6_REFILL_PULSES_IN_BOTTLE                             2100



#define DEFAULT_HV_LAMBDA_SET_POINT                               14000
#define DEFAULT_ELECTROMAGNET_CURRENT                             15200
#define DEFAULT_GUN_DRIVER_PULSE_TOP                               8000
#define DEFAULT_GUN_DRIVER_CATHODE_VOLTAGE                        20000
#define DEFAULT_SPARE_TRIGGER                                         0
#define DEFAULT_PULSE_SYNC_AFC_SAMPLE_DELAY                         200
#define DEFAULT_GUN_START_MIN_DOSE                                  150
#define DEFAULT_GUN_START_MAX_DOSE                                  100
#define DEFAULT_GUN_STOP_MIN_DOSE                                   160
#define DEFAULT_GUN_STOP_MAX_DOSE                                   200
#define DEFAULT_AFC_HOME_POSITION                                 18000
#define DEFAULT_PRF                                                 500 
  

#define DEFAULT_MAGNETRON_HEATER_CURRENT                          10600
#define DEFAULT_GUN_DRIVER_HEATER_CURRENT                          1450
#define DEFAULT_HVPS_TRIGGER_START                                    0
#define DEFAULT_HVPS_TRIGGER_STOP                                     0
#define DEFAULT_TRIGGER_PFN                                           0
#define DEFAULT_TRIGGER_MAGNETRON_AND_TARGET_CURRENT_START          200
#define DEFAULT_TRIGGER_MAGNETRON_AND_TARGET_CURRENT_STOP             0
#define DEFAULT_X_RAY_ON_TIME                                       100
#define DEFAULT_GUN_BIAS_VOLTAGE                                      0
#define DEFAULT_AFT_CONTROL_VOLTAGE                                2000


#endif




#ifdef __SYSTEM_CONFIGURATION_6_MEV_MEDICAL
#define SOFTWARE_DASH_NUMBER                 100


#define DRIVE_UP_TIMEOUT                     1000 // 10 Seconds
#define MINIMUM_FAULT_HOLD_TIME              50   // .5 Second
#define FAULT_RESET_HOLD_TIME                200  // 2 Second
#define MAX_DRIVE_UP_FAULTS                  5    // Attempts to bring up high voltage before latching fault
#define MAX_HV_ON_FAULTS                     10   // Attempts to reset faults with HV on (but Xray Off) before latching 

#define MAGNETRON_HEATER_WARM_UP_TIME        300   // 5 minutes
#define THYRATRON_WARM_UP_TIME               900   // 15 minutes
#define GUN_DRIVER_HEATER_WARM_UP_TIME       300   // 5 minutes

#define GUN_HEATER_HOLDOFF_AT_STARTUP        500   // 5 seconds
#define GUN_HEATER_ADDITONAL_HOLDOFF_COLD   2500   // 25 seconds


#define DEFAULT_REMOTE_IP_ADDRESS                            0x0F46A8C0  // 192.168.70.15
#define DEFAULT_IP_ADDRESS                                   0x6346A8C0  // 192.168.70.99

#define MAX_SF6_REFILL_PULSES_IN_BOTTLE                             48300



#define DEFAULT_HV_LAMBDA_SET_POINT                               14000
#define DEFAULT_ELECTROMAGNET_CURRENT                             15200
#define DEFAULT_GUN_DRIVER_PULSE_TOP                               8000
#define DEFAULT_GUN_DRIVER_CATHODE_VOLTAGE                        20000
#define DEFAULT_SPARE_TRIGGER                                         0
#define DEFAULT_PULSE_SYNC_AFC_SAMPLE_DELAY                         200
#define DEFAULT_GUN_START_MIN_DOSE                                  150
#define DEFAULT_GUN_START_MAX_DOSE                                  100
#define DEFAULT_GUN_STOP_MIN_DOSE                                   160
#define DEFAULT_GUN_STOP_MAX_DOSE                                   200
#define DEFAULT_AFC_HOME_POSITION                                 18000
#define DEFAULT_PRF                                                 500 
  

#define DEFAULT_MAGNETRON_HEATER_CURRENT                          10600
#define DEFAULT_GUN_DRIVER_HEATER_CURRENT                          1450
#define DEFAULT_HVPS_TRIGGER_START                                    0
#define DEFAULT_HVPS_TRIGGER_STOP                                     0
#define DEFAULT_TRIGGER_PFN                                           0
#define DEFAULT_TRIGGER_MAGNETRON_AND_TARGET_CURRENT_START          200
#define DEFAULT_TRIGGER_MAGNETRON_AND_TARGET_CURRENT_STOP           400
#define DEFAULT_X_RAY_ON_TIME                                       100
#define DEFAULT_GUN_BIAS_VOLTAGE                                  40000
#define DEFAULT_AFT_CONTROL_VOLTAGE                                2000


#endif








#endif
