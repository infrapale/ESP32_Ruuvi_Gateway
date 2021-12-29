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

#include "RuuviTag.h"
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


typedef struct
{
    Adafruit_MQTT_Publish *ada_mqtt_publ;
    char          caption[CAPTION_LEN];
    value_type_et value_type;
    float         *data_ptr;
    bool          *updated_ptr;   
} sensor_st;


int scanTime = 5; //In seconds
BLEScan* pBLEScan;

RuuviTag  ruuvi_tag;
uint8_t  sensor_indx = 0;
const char* ssid     = WIFI_SSID;            //Main Router      
const char* password = WIFI_PASS;            //Main Router Password

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, IO_USERNAME, IO_KEY);
// Adafruit_MQTT_Subscribe ledControl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ledControl");
Adafruit_MQTT_Publish home_id_temp = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.esp32test-temp");
Adafruit_MQTT_Publish home_od_temp = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.tampere-ruuvi-outdoor-temp");
Adafruit_MQTT_Publish home_sauna_temp = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.tampere-ruuvi-sauna-temp");
Adafruit_MQTT_Publish home_id_hum = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.tampere-ruuvi-indoor-hum");

/// Sensor MQTT pointers, other data will be initialized in the setup function
sensor_st sensor[NBR_SENSORS]= 
{
    { &home_id_temp, "", VALUE_TYPE_FLOAT, NULL,NULL},
    { &home_sauna_temp, "", VALUE_TYPE_FLOAT, NULL,NULL}, 
    { &home_od_temp, "", VALUE_TYPE_FLOAT, NULL,NULL}, 
    { &home_id_hum, "", VALUE_TYPE_FLOAT, NULL,NULL}
};


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

        String mac_addr = advertisedDevice.getAddress().toString().c_str();
        String raw_data = String(BLEUtils::buildHexData(nullptr, (uint8_t*)advertisedDevice.getManufacturerData().data(), advertisedDevice.getManufacturerData().length()));
        raw_data.toUpperCase();

        ruuvi_tag.decode_raw_data(mac_addr, raw_data, advertisedDevice.getRSSI());           
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

    /// Refine ruuvi tag data
    ruuvi_tag.add(String("e6:2c:8d:db:22:35"),"home_indoor");
    ruuvi_tag.add(String("ea:78:e2:12:36:f8"),"home_sauna");
    ruuvi_tag.add(String("ed:9a:ab:c6:30:72"),"home_outdoor");

    /// Refine sensors
    strncpy(sensor[0].caption,ruuvi_tag.ruuvi[0].location,CAPTION_LEN-RUUVI_LOCATION_LEN);
    strcat(sensor[0].caption, "_temperature");

    strncpy(sensor[1].caption,ruuvi_tag.ruuvi[1].location,CAPTION_LEN-RUUVI_LOCATION_LEN);
    strcat(sensor[1].caption, "_temperature");
    
    strncpy(sensor[2].caption,ruuvi_tag.ruuvi[2].location,CAPTION_LEN-RUUVI_LOCATION_LEN);
    strcat(sensor[2].caption, "_temperature");
    
    strncpy(sensor[3].caption,ruuvi_tag.ruuvi[0].location,CAPTION_LEN-RUUVI_LOCATION_LEN);
    strcat(sensor[3].caption, "_humidity");
    
    sensor[0].data_ptr = &ruuvi_tag.ruuvi[0].temp_fp;
    sensor[1].data_ptr = &ruuvi_tag.ruuvi[1].temp_fp;
    sensor[2].data_ptr = &ruuvi_tag.ruuvi[2].temp_fp;
    sensor[3].data_ptr = &ruuvi_tag.ruuvi[0].humidity;

    sensor[0].updated_ptr = &ruuvi_tag.ruuvi[0].updated;
    sensor[1].updated_ptr = &ruuvi_tag.ruuvi[1].updated;
    sensor[2].updated_ptr = &ruuvi_tag.ruuvi[2].updated;
    sensor[3].updated_ptr = &ruuvi_tag.ruuvi[0].updated;
      
    /**
     * Setup BLE scanning
     */
    Serial.println("Scanning...");
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(1000);
    pBLEScan->setWindow(990);  // less or equal setInterval value
  
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
