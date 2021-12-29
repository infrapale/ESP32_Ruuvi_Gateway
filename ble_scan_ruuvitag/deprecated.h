    for (uint8_t i = 0; i< RUUVI_NBR_OF; i++)
    {
        ruuvi[i].updated = 0;
    }



String payload;
//Converts decoded RUUVI data into uBeac JSON format
void uBeacRuuvi()
{
    //Serial.print("uBeacRuuvi..");
    payload = "[{\"id\": \"MyRUUVI\", \"sensors\": [{\"id\": \"Temperature\", \"value\": $temperature$}, {\"id\": \"Humidity\", \"value\": $humidity$}, "
                     "{\"id\": \"Pressure\", \"value\": $pressure$}, {\"id\": \"Acceleration\", \"value\": {\"ax\": $ax$,\"ay\": $ay$,\"az\": $az$,}}, "
                     "{\"id\": \"Voltage Power\", \"value\": $voltage_power$}, {\"id\": \"Voltage\", \"value\": $voltage$}, {\"id\": \"Power\", \"value\": $power$}, "
                     "{\"id\": \"RSSI\", \"value\": $rssi$}, {\"id\": \"Movement Counter\", \"value\": $movement$}, {\"id\": \"Measurement Sequence\", \"value\": $measurement$}]}]";
  
    payload.replace("$temperature$",String(temp));
    payload.replace("$humidity$",String(hum));  
    payload.replace("$pressure$",String(pressure/100));
    payload.replace("$ax$",String(ax*9.81/1000));
    payload.replace("$ay$",String(ay*9.81/1000));
    payload.replace("$az$",String(az*9.81/1000));
    payload.replace("$voltage_power$",String(voltage_power));
    payload.replace("$voltage$",String(voltage));
    payload.replace("$power$",String(power));
    payload.replace("$rssi$",String(rssi_ruuvi));
    payload.replace("$movement$",String(movement));
    payload.replace("$measurement$",String(measurement));
    //Serial.println(payload);
  
}




    switch(mqtt_tx_state)
    {
        case 0:
            if (ruuvi[0].updated)
            {    
                ruuvi[0].ada_mqtt_publ = &home_sauna_temp;
                if (ruuvi[0].ada_mqtt_publ-> publish(ruuvi[0].temp_fp))
                //if (home_id_temp.publish(ruuvi[0].temp_fp))  
                {
                    Serial.println(ruuvi[0].temp_fp,1);
                }
                else    
                {
                    Serial.println(F("Failed"));
                }
                ruuvi[0].updated = false;
             }   
             break;
        case 1:
            if (ruuvi[1].updated)
            {   
                Serial.print("Home Outdoor Temperature: ");
                if (home_od_temp.publish(ruuvi[1].temp_fp))    
                {
                    Serial.println(ruuvi[1].temp_fp,1);
                }
                else
                {
                    Serial.println(F("Failed"));
                }
                ruuvi[1].updated = false;
            }
            break;
        case 2:
            if (ruuvi[2].updated)
            {   
                Serial.print("Home Sauna Temperature: ");
                if (home_sauna_temp.publish(ruuvi[2].temp_fp))    
                {
                    Serial.println(ruuvi[2].temp_fp,1);
                }
                else
                {
                    Serial.println(F("Failed"));
                }
                ruuvi[2].updated = false;
            }
            break;
        default:
            break;
    }
