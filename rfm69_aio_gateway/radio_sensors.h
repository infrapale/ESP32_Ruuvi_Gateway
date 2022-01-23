#ifndef __RADIO_SENSOR_H__
#define __RADIO_SENSOR_H__
#include "Arduino.h"
#define NBR_RADIO_SENSORS 6


typedef struct{
    String name;
    String zone;
    String sensor;
    float value;  
    String unit;
    uint8_t nbr_decimals;
    bool   updated;
} sensor_entry_st;

void parse_msg(char *rad_msg);
void print_radio_sensors(void);
void radio_sensors_get_name(uint8_t indx, char *full_name);
float *radio_sensors_get_value_ptr(uint8_t indx);
bool  *radio_sensors_get_updated_ptr(uint8_t indx);


#endif
