/**
 * https://github.com/ruuvi/ruuvi-sensor-protocols/blob/master/dataformat_03.md
 * https://tutorial.cytron.io/2020/01/15/send-sensors-data-to-adafruit-io-using-esp32/
 */
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEAddress.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <esp_task_wdt.h>
#define   WDT_TIMEOUT 10

#define  VILLA_ASTRID
//#define  H_MOKKULA
//#define PIRPANA
#include "secrets.h"
//#include <HTTPClient.h>
#include "config.h"
#include "helpers.h"

#define NBR_SENSORS     3
#define CAPTION_LEN     40
#define MAC_ADDR_LEN    18


WiFiClient client;

typedef enum
{
    VALUE_TYPE_UNDEFINED = 0,
    VALUE_TYPE_FLOAT,
} value_type_et;

typedef enum
{
    RUUVI_HOME_INDOOR = 0,
    RUUVI_HOME_SAUNA,
    RUUVI_HOME_OUTDOOR,
    RUUVI_NBR_OF
} ruuvi_et;

typedef struct
{
    Adafruit_MQTT_Publish *ada_mqtt_publ;
    char     caption[CAPTION_LEN];
    //char     mac_addr[MAC_ADDR_LEN];
    value_type_et  value_type;
    void     *data_ptr;
    bool     *updated_ptr;   
} sensor_st;

typedef struct
{
    float    temp_fp;
    uint16_t temperature;
    uint16_t humidity;
    uint16_t pressure;
    int16_t  acc[3];
    uint16_t voltage_power;
    uint16_t voltage;
    uint16_t power;
    uint16_t rssi;
    uint16_t movement;
    uint16_t measurement;
    bool     updated;
    
} ruuvi_tag_data_st;

int scanTime = 5; //In seconds
BLEScan* pBLEScan;

ruuvi_tag_data_st ruuvi[RUUVI_NBR_OF];
uint8_t  mqtt_tx_state = 0;
int temp, hum, pressure, ax, ay, az, voltage_power, voltage, power, rssi_ruuvi, movement, measurement;
float temp_fp;
String payload;
//String MAC_add = "ed:9a:ab:c6:30:72"; //All the identified MAC addresses will go in one String
String MAC_addr[RUUVI_NBR_OF] =
{
    "e6:2c:8d:db:22:35",  // RUUVI_HOME_INDOOR
    "ea:78:e2:12:36:f8",  // RUUVI_HOME_OUTDOOR
    "ed:9a:ab:c6:30:72"   // RUUVI_HOME_SAUNA
};

const char* ssid     = WIFI_SSID;              //Main Router      
const char* password = WIFI_PASS;            //Main Router Password
//const char* url = "UBEAC GATEWAY URL HERE"; 

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, IO_USERNAME, IO_KEY);
// Adafruit_MQTT_Subscribe ledControl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ledControl");
Adafruit_MQTT_Publish home_id_temp = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.esp32test-temp");
Adafruit_MQTT_Publish home_od_temp = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.tampere-ruuvi-outdoor-temp");
Adafruit_MQTT_Publish home_sauna_temp = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.tampere-ruuvi-sauna-temp");


sensor_st sensor[NBR_SENSORS]= 
{
    { 
        &home_id_temp, "Home Indoor Temperature", VALUE_TYPE_FLOAT, 
        &ruuvi[RUUVI_HOME_INDOOR].temp_fp, &ruuvi[RUUVI_HOME_INDOOR].updated 
    },
    { 
        &home_od_temp, "Home Outdoor Temperature", VALUE_TYPE_FLOAT, 
        &ruuvi[RUUVI_HOME_OUTDOOR].temp_fp, &ruuvi[RUUVI_HOME_OUTDOOR].updated 
    },
    { 
        &home_sauna_temp, "Home Sauna Temperature", VALUE_TYPE_FLOAT, 
        &ruuvi[RUUVI_HOME_SAUNA].temp_fp, &ruuvi[RUUVI_HOME_SAUNA].updated 
    }
};


//Decodes RUUVI raw data and arranges it in an array
void decodeRuuvi(uint8_t indx, String hex_data, int rssi){
    Serial.print("decodeRuuvi (");
    Serial.print(indx);
    Serial.print("): ");
    Serial.print(hex_data);
    //Serial.println(hex_data.substring(4, 6));
    /**
     *  9904034F1545C8DFFF52004E03E90B59
     *  9904
     *  03
     *  4F
     *  15 45
     *  C8DF
     *  FF52004E03E90B59
     */
    Serial.print(" "); Serial.print(hex_data.substring(4, 6)); 
    if(hex_data.substring(4, 6) == "03")
    {   
        ruuvi[indx].temperature = hexadecimalToDecimal(hex_data.substring(8, 10));
        ruuvi[indx].temp_fp = (float) ruuvi[indx].temperature + (float) hexadecimalToDecimal(hex_data.substring(10, 12)) / 100.0;
        ruuvi[indx].humidity = hexadecimalToDecimal(hex_data.substring(6, 8));
        //ruuvi[indx].pressure = hexadecimalToDecimal(hex_data.substring(12, 16))-50000;

        //ax = hexadecimalToDecimal(hex_data.substring(18, 22));
        //ay = hexadecimalToDecimal(hex_data.substring(22, 26));
        //az = hexadecimalToDecimal(hex_data.substring(26, 30));     

        //if(ax > 0xF000){ax = ax - (1 << 16);}
        //if(ay > 0xF000){ay = ay - (1 << 16);}
        //if (az > 0xF000){az = az - (1 << 16);}

        //ruuvi[indx].voltage_power = hexadecimalToDecimal(hex_data.substring(30, 34));
        //ruuvi[indx].voltage = (int)((voltage_power & 0x0b1111111111100000) >> 5) + 1600;
        //ruuvi[indx].power = (int)(voltage_power & 0x0b0000000000011111) - 40;

        //ruuvi[indx].rssi = rssi;

        //ruuvi[indx].movement = hexadecimalToDecimal(hex_data.substring(34, 36));
        //ruuvi[indx].measurement = hexadecimalToDecimal(hex_data.substring(36, 40));
        ruuvi[indx].updated = true;
    }
    if(hex_data.substring(4, 6) == "05")
    {
        ruuvi[indx].temperature = hexadecimalToDecimal(hex_data.substring(6, 10));
        ruuvi[indx].temp_fp = (float) ruuvi[indx].temperature * 0.005;
        ruuvi[indx].humidity = hexadecimalToDecimal(hex_data.substring(10, 14)) *0.000025;
        ruuvi[indx].updated = true;
         
    }
    
     Serial.println("..done");
}

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

//Class that scans for BLE devices
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks 
{
    void onResult(BLEAdvertisedDevice advertisedDevice) 
    {
        //Scans for specific BLE MAC addresses 
        //Serial.println(advertisedDevice.getAddress().toString().c_str());
        esp_task_wdt_reset();
        for (uint8_t i = 0; i < RUUVI_NBR_OF; i++)
        {
            if(MAC_addr[i].indexOf(advertisedDevice.getAddress().toString().c_str()) >= 0) //If the scanned MAC address is in the identified MAC address String
            {   
                Serial.println(advertisedDevice.getAddress().toString().c_str());
                String raw_data = String(BLEUtils::buildHexData(nullptr, (uint8_t*)advertisedDevice.getManufacturerData().data(), advertisedDevice.getManufacturerData().length()));
                raw_data.toUpperCase();
                decodeRuuvi(i, raw_data, advertisedDevice.getRSSI());
                //uBeacRuuvi();
                // Serial.print("raw: "); Serial.println(raw_data);
                break;
            }            
        }
    }
};

void setup() 
{
    Serial.begin(115200);
    
    //Connect to Local WiFi
    delay(4000);   //Delay needed before calling the WiFi.begin

    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    //esp_task_wdt_add(NULL); //add current thread to WDT watch
    WiFi.begin(ssid, password); 
   
    while (WiFi.status() != WL_CONNECTED)  //Check for the connection
    {
        delay(1000);
        Serial.println("Connecting to WiFi..");
    }
    for (uint8_t i = 0; i< RUUVI_NBR_OF; i++)
    {
        ruuvi[i].updated = 0;
    }
 
    Serial.println("Connected to the WiFi network");

    //BLE scanning
    Serial.println("Scanning...");


    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);  // less or equal setInterval value
  
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        Serial.print(".");
    }
    
}


void MQTT_connect(void)
{
    int8_t ret;
  
    // Stop if already connected.
    if (mqtt.connected()) 
    {
        return;
    }
  
    Serial.print("Connecting to MQTT… ");
  
    uint8_t retries = 3;
  
    while ((ret = mqtt.connect()) != 0) 
    { 
        // connect will return 0 for connected
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println("Retrying MQTT connection in 5 seconds…");
        mqtt.disconnect();
    
        delay(5000);
        if (--retries == 0) {
            // basically die and wait for WDT to reset me
            while (1);
        }
    } 
    Serial.println("MQTT Connected!");
}

void loop() 
{
    Serial.print("Loop ..MQTT Connect...");
    MQTT_connect();
    BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
    
    esp_task_wdt_reset();
    Serial.print("  mqtt_tx_state = "); Serial.print(mqtt_tx_state); Serial.print("  ");
    
    if (sensor[mqtt_tx_state]->updated)
    {
        sensor[mqtt_tx_state]->ada_mqtt_publ->publish();
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
    ++mqtt_tx_state;
    if(mqtt_tx_state > 2) mqtt_tx_state = 0;
    
    Serial.println(" clearResults...");
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
    delay(10000);
}
