/**
 * @file ble_scan_ruuvitag.ino
 * 
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

/// SSID Definitions
#define  VILLA_ASTRID
//#define  H_MOKKULA
//#define PIRPANA
#include "secrets.h"
//#include <HTTPClient.h>
#include "config.h"
#include "helpers.h"

#define NBR_SENSORS               4       ///< Number of sensor values
#define CAPTION_LEN               40      ///< Length of value name
#define MAC_ADDR_LEN              18      ///< Length of the BLE MAC address string
#define MQTT_UPDATE_INTERVAL_ms   60000   ///< MQTT update interval, one value per time


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
    float     *data_ptr;
    bool     *updated_ptr;   
} sensor_st;

typedef struct
{
    float    temp_fp;
    uint16_t temperature;
    float    humidity;
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

ruuvi_tag_data_st ruuvi[RUUVI_NBR_OF] = {0};
uint8_t  sensor_indx = 0;
//int temp, hum, pressure, ax, ay, az, voltage_power, voltage, power, rssi_ruuvi, movement, measurement;


String MAC_addr[RUUVI_NBR_OF] =
{
    "e6:2c:8d:db:22:35",  // RUUVI_HOME_INDOOR
    "ea:78:e2:12:36:f8",  // RUUVI_HOME_OUTDOOR
    "ed:9a:ab:c6:30:72"   // RUUVI_HOME_SAUNA
};

const char* ssid     = WIFI_SSID;            //Main Router      
const char* password = WIFI_PASS;            //Main Router Password

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, IO_USERNAME, IO_KEY);
// Adafruit_MQTT_Subscribe ledControl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ledControl");
Adafruit_MQTT_Publish home_id_temp = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.esp32test-temp");
Adafruit_MQTT_Publish home_od_temp = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.tampere-ruuvi-outdoor-temp");
Adafruit_MQTT_Publish home_sauna_temp = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.tampere-ruuvi-sauna-temp");
Adafruit_MQTT_Publish home_id_hum = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.tampere-ruuvi-indoor-hum");

/// Sensor Caption and Pointers
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
    },
    { 
        &home_id_hum, "Home Indoor Humidity", VALUE_TYPE_FLOAT, 
        &ruuvi[RUUVI_HOME_INDOOR].humidity, &ruuvi[RUUVI_HOME_INDOOR].updated 
    }
};


/** 
 *  Decodes RUUVI raw data and arranges it in an array
 *  Decoding depends on the RuuviTag data version 
 *  @param[in] indx,  RuuviTag index
 *  @param[in] hex_data = raw data from the sensor
 *  @param[in] rssi = BLR signal strength
 */
void decodeRuuvi(uint8_t indx, String hex_data, int rssi){
    /*
    Serial.print("decodeRuuvi ("); Serial.print(indx);
    Serial.print("): "); Serial.print(hex_data);
    Serial.print(" "); Serial.print(hex_data.substring(4, 6)); 
    Serial.println("");
    */
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
}

/**
 * Class that scans for BLE devices
 */
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
                // Serial.println(advertisedDevice.getAddress().toString().c_str());
                String raw_data = String(BLEUtils::buildHexData(nullptr, (uint8_t*)advertisedDevice.getManufacturerData().data(), advertisedDevice.getManufacturerData().length()));
                raw_data.toUpperCase();
                decodeRuuvi(i, raw_data, advertisedDevice.getRSSI());
                // Serial.print("raw: "); Serial.println(raw_data);
                break;
            }            
        }
    }
};

/**
 * Connect to MQTT service
 * Skip if already connected
 * Restart (WDT) if the connection is failing
 * 
 */
void MQTT_connect(void)
{
    int8_t ret;
  
    // Stop if already connected.
    if (!mqtt.connected()) 
    {
        //Serial.println("Connecting to MQTT… ");      
        uint8_t retries = 3;
      
        while ((ret = mqtt.connect()) != 0) 
        { 
            // connect will return 0 for connected
            Serial.println(mqtt.connectErrorString(ret));
            Serial.println("Retrying MQTT connection in 5 seconds…");
            mqtt.disconnect();
        
            delay(5000);
            if (--retries == 0) 
            {
                Serial.println("Retry limit reached -> WDT reset…");
                // basically die and wait for WDT to reset me
                while (1);
            }
        } 
        Serial.println("MQTT Connected!");
    }
}

/**
 * Arduino setup() function
 */

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

/**
 * Arduino loop() function
 */
void loop() 
{
    MQTT_connect();
    BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
    
    esp_task_wdt_reset();
    Serial.print("sensor_indx = "); Serial.print(sensor_indx); Serial.print("  ");
    
    if (*sensor[sensor_indx].updated_ptr)
    {
        Serial.print(sensor[sensor_indx].caption); Serial.print("  ");
        Serial.print(*sensor[sensor_indx].data_ptr);
        sensor[sensor_indx].ada_mqtt_publ->publish(*sensor[sensor_indx].data_ptr);      
    }
    if(++sensor_indx >= NBR_SENSORS) sensor_indx = 0;
    
    Serial.println("");
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
    delay(MQTT_UPDATE_INTERVAL_ms);
}
