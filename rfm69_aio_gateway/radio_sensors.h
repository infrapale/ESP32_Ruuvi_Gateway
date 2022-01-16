#ifndef __RADIO_SENSOR_H__
#define __RADIO_SENSOR_H__
#include "Arduino.h"
#define NBR_COLLECTED_SENSORS 6


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


#endif
