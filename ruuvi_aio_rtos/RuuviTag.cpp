
#include <Arduino.h>
#include "RuuviTag.h"
#include "helpers.h"





RuuviTag::RuuviTag(void)
{
    memset(ruuvi,0x00,sizeof(ruuvi));
    nbr_of = 0;
    _active_indx = -1;

}

int8_t RuuviTag::add(String mac_address, const char *loc)
{
    if (_active_indx < 0 )
    {
        _active_indx = 0;
        nbr_of = 1;
    }
    else
    {
        if  (_active_indx < MAX_NBR_RUUVI_TAG -1 ) 
        {
            _active_indx++;
            nbr_of++;
        }
        else
        {
            _active_indx = - 1;
        }
    }

    if (_active_indx >= 0) 
    {
        ruuvi[_active_indx].mac_addr = mac_address;
        strncpy(ruuvi[_active_indx].location, loc, RUUVI_LOCATION_LEN);
    }
    return  _active_indx;
}


String *RuuviTag::get_addr(uint8_t indx)
{
    if (indx < nbr_of)
    {
        return &ruuvi[indx].mac_addr;  
    }
    else
    {
        return &undef_addr;
    }
}

char *RuuviTag::get_location(uint8_t indx)
{
    if (indx < nbr_of)
    {
        return ruuvi[indx].location;  
    }
    else
    {
        return ruuvi[0].location;
    }
}


/** 
 *  Decodes RUUVI raw data and arranges it in an array
 *  Decoding depends on the RuuviTag data version 
 *  @param[in] mac_addr = mac address from sensor
 *  @param[in] rssi = BLR signal strength
 */ 
void RuuviTag::decode_raw_data(String mac_addr, String raw_data, int rssi)
{
    for (uint8_t indx = 0; indx < nbr_of; indx++)
    {
        if(ruuvi[indx].mac_addr.indexOf(mac_addr)  >= 0 )
        {
            if(raw_data.substring(4, 6) == "03")
            {   
                ruuvi[indx].temperature = hexadecimalToDecimal(raw_data.substring(8, 10));
                ruuvi[indx].temp_fp = (float) ruuvi[indx].temperature + (float) hexadecimalToDecimal(raw_data.substring(10, 12)) / 100.0;
                ruuvi[indx].humidity = hexadecimalToDecimal(raw_data.substring(6, 8));
                //ruuvi[indx].pressure = hexadecimalToDecimal(raw_data.substring(12, 16))-50000;
        
                //ax = hexadecimalToDecimal(raw_data.substring(18, 22));
                //ay = hexadecimalToDecimal(raw_data.substring(22, 26));
                //az = hexadecimalToDecimal(raw_data.substring(26, 30));     
        
                //if(ax > 0xF000){ax = ax - (1 << 16);}
                //if(ay > 0xF000){ay = ay - (1 << 16);}
                //if (az > 0xF000){az = az - (1 << 16);}
        
                //ruuvi[indx].voltage_power = hexadecimalToDecimal(raw_data.substring(30, 34));
                //ruuvi[indx].voltage = (int)((voltage_power & 0x0b1111111111100000) >> 5) + 1600;
                //ruuvi[indx].power = (int)(voltage_power & 0x0b0000000000011111) - 40;
        
                ruuvi[indx].rssi = rssi;
        
                //ruuvi[indx].movement = hexadecimalToDecimal(raw_data.substring(34, 36));
                //ruuvi[indx].measurement = hexadecimalToDecimal(raw_data.substring(36, 40));
                ruuvi[indx].updated = true;
            }
            if(raw_data.substring(4, 6) == "05")
            {
                ruuvi[indx].temperature = hexadecimalToDecimal(raw_data.substring(6, 10));
                ruuvi[indx].temp_fp = (float) ruuvi[indx].temperature * 0.005;
                ruuvi[indx].humidity = hexadecimalToDecimal(raw_data.substring(10, 14)) *0.000025;
                ruuvi[indx].updated = true;        
            }

        
        }
    }
}


void decode(uint8_t indx, String hex_data, int rssi){
    /*
    Serial.print("decodeRuuvi ("); Serial.print(indx);
    Serial.print("): "); Serial.print(hex_data);
    Serial.print(" "); Serial.print(hex_data.substring(4, 6)); 
    Serial.println("");
    */
/*
    */
}
