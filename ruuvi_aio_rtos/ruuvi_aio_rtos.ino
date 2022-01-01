/**
 * RuuviTag Adafruit IO Gateway
 * @author TomHöglund 2021
 * 
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html
 * https://github.com/adafruit/Adafruit_MQTT_Library/blob/master/Adafruit_MQTT.h
 * 
 */
 
#include <Wire.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEAddress.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <esp_task_wdt.h>


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

#define WDT_TIMEOUT           10
#define ARDUINO_RUNNING_CORE  1
#define LED_RED               26
#define LED_GREEN             32
#define LED_YELLOW            33
#define LED_BLUE              25

int scanTime = 5; //In seconds
BLEScan* pBLEScan;

const char* ssid     = WIFI_SSID;            //Main Router      
const char* password = WIFI_PASS;            //Main Router Password


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



void StartTasks(void);
static SemaphoreHandle_t sema_wifi_avail;
static SemaphoreHandle_t sema_mqtt_avail;

 
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, IO_USERNAME, IO_KEY);
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

RuuviTag  ruuvi_tag;

extern bool         loopTaskWDTEnabled;
static TaskHandle_t htask;


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
       
        digitalWrite(LED_BLUE,HIGH);         
    }
    
};


void setup() 
{
    BaseType_t rc;
    esp_err_t er;

    htask = xTaskGetCurrentTaskHandle();
    loopTaskWDTEnabled = true;
    
    delay(4000);
    Serial.begin(115200);   

    er = esp_task_wdt_status(htask);
    assert(er == ESP_ERR_NOT_FOUND);
    
    if ( er == ESP_ERR_NOT_FOUND ) {
        er = esp_task_wdt_init(10,true);
        assert(er == ESP_OK);
        er = esp_task_wdt_add(htask);
        assert(er == ESP_OK);
        printf("Task is subscribed to TWDT.\n");
    }
    
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED,LOW);
    pinMode(LED_GREEN, OUTPUT);
    digitalWrite(LED_GREEN,LOW);
    pinMode(LED_YELLOW, OUTPUT);
    digitalWrite(LED_YELLOW,LOW);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_BLUE,LOW);

    Serial.println("Setup BLE Scanning...");
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);  // less or equal setInterval value

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
      

 
    sema_wifi_avail = xSemaphoreCreateBinary();
    sema_mqtt_avail = xSemaphoreCreateBinary();

    assert(sema_wifi_avail);
    assert(sema_mqtt_avail);
    rc = xSemaphoreGive(sema_wifi_avail);
    rc = xSemaphoreGive(sema_mqtt_avail);
    StartTasks();
}

void loop() {
    esp_err_t er;
    er = esp_task_wdt_add(nullptr);
    er = esp_task_wdt_status(htask);
    assert(er == ESP_OK);
    esp_task_wdt_reset();
    vTaskDelay(1000);
}

void StartTasks(void){
    BaseType_t rc;

    rc = xSemaphoreTake(sema_wifi_avail,portMAX_DELAY);
    assert(rc == pdPASS);
    rc = xSemaphoreTake(sema_mqtt_avail,portMAX_DELAY);
    assert(rc == pdPASS);

    
    xTaskCreatePinnedToCore(
       TaskConnectWiFi
        ,  "TaskConnectWiFi" 
        ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);
    
  
    xTaskCreatePinnedToCore(
       TaskConnectMqtt
        ,  "TaskConnectMqtt" 
        ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);


     xTaskCreatePinnedToCore(
       TaskScanBle
        ,  "TaskScanBle" 
        ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);
    
    /*
    xTaskCreatePinnedToCore(
       TaskSendMqtt
        ,  "TaskSendMqtt" 
        ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);
    
    xTaskCreatePinnedToCore(
       TaskReadBme680
        ,  "TaskSendMqtt" 
        ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);
    
      */
}

void TaskConnectWiFi( void *pvParameters ){
    uint8_t state;
    int8_t  ret;
    uint8_t retries = 6;
    BaseType_t rc;
    esp_err_t er;

    state = 0;
    er = esp_task_wdt_add(nullptr);
    assert(er == ESP_OK);
    
    for (;;)
    {
        //printf("WiFi state: %d\n", state);
        switch(state)
        {
            case 0:   // initial
                if (WiFi.status() != WL_CONNECTED){
                    Serial.println("Connecting WiFi");       
                    WiFi.begin(ssid, password); 
                    retries = 6;
                    state++;
                }
                else state = 2;
                vTaskDelay(1000);
                break;
            case 1:   // Check for the connection
                if (WiFi.status() != WL_CONNECTED) {
                    digitalWrite(LED_YELLOW,LOW);
                    vTaskDelay(1000);
                    if (--retries == 0) state = 3;
                    else Serial.println("Waiting for WiFi"); 
                }
                else {
                    digitalWrite(LED_YELLOW,HIGH);
                    Serial.println("Connected to WiFi");
                    rc = xSemaphoreGive(sema_wifi_avail);
                    state = 2;
                }             
                esp_task_wdt_reset();
                vTaskDelay(1000);
                break;
            case 2:   // 
                if (WiFi.status() != WL_CONNECTED) state = 0;
                esp_task_wdt_reset();
                vTaskDelay(1000);
                break;
            case 3:   // 
                Serial.println("WiFi Retry limit reached -> WDT reset…");
                vTaskDelay(1000);  //        while (1); 
                break;
        }
    }
}



void TaskConnectMqtt( void *pvParameters ){
    uint8_t state;
    int8_t ret;
    uint8_t retries = 3;
    BaseType_t rc;
    esp_err_t er;
    uint8_t   wifi_timeout;
    uint8_t   mqtt_timeout;
    uint8_t   sensor_indx;
    uint8_t   interval_sec;
    
    state = 0;
    er = esp_task_wdt_add(nullptr);
    assert(er == ESP_OK);
    
    for (;;)
    { 
        printf("MQTT state: %d\n", state);
        switch(state) {
            case 0: // initial
                digitalWrite(LED_GREEN,LOW);
                wifi_timeout = 10;
                mqtt_timeout = 10;
                sensor_indx  = 0;
                state++;
                break;
            case 1: // waiting for WiFi
                rc = xSemaphoreTake(sema_wifi_avail, 1000);
                if(rc == pdPASS) state++;
                else {
                    if(--wifi_timeout == 0) state = 6;
                }
                esp_task_wdt_reset();
                vTaskDelay(1000);
                break;
            case 2: // WiFi is available
                Serial.println("Connecting to MQTT… ");              
                ret = mqtt.connect();
                if (ret != 0) {    // connect will return 0 for connected
                    Serial.println(mqtt.connectErrorString(ret));
                    Serial.println("Retrying MQTT connection…");
                    mqtt.disconnect();          
                    if (--mqtt_timeout == 0) state = 6;
                }
                else {
                    state = 3;
                    Serial.println("MQTT Connected!"); 
                    rc = xSemaphoreGive(sema_mqtt_avail);
                    digitalWrite(LED_GREEN,HIGH);  
                }
                esp_task_wdt_reset();
                vTaskDelay(1000);
                break;


            case 3: // MQTT is connected
                if (mqtt.connected()){
                    if (*sensor[sensor_indx].updated_ptr)
                    {
                        printf("%s %f\n",sensor[sensor_indx].caption,*sensor[sensor_indx].data_ptr);
                        sensor[sensor_indx].ada_mqtt_publ->publish(*sensor[sensor_indx].data_ptr);      
                    }
                    if(++sensor_indx >= NBR_SENSORS) sensor_indx = 0;
                    state++;
                    interval_sec = 15;

                } else {
                    interval_sec = 15;
                    state++;
                }
                esp_task_wdt_reset();
                vTaskDelay(1000);
                break;

            case 4: // Release WiFI and MQTT
                mqtt.disconnect();  
                rc = xSemaphoreGive(sema_wifi_avail);  
                state++;
                break;

            case 5: //
                if(--interval_sec == 0) state = 1;
                esp_task_wdt_reset();
                vTaskDelay(1000);          
                break;

            case 6: //
                Serial.println("Retry limit reached -> WDT reset…");
                rc = xSemaphoreGive(sema_wifi_avail);
                rc = xSemaphoreGive(sema_mqtt_avail);
                vTaskDelay(10000);
                break;
        }
    }
}

void TaskScanBle( void *pvParameters ){
    BaseType_t rc;
    esp_err_t er;
    uint8_t   state = 0;
    BLEScanResults foundDevices;
    
    er = esp_task_wdt_add(nullptr);
    assert(er == ESP_OK);

    for (;;)
    {   
        printf("Scan BLE state: %d\n", state);
        switch(state) {
            case 0:   // Initial state
                state++;
                break; 
            case 1:   // BLE Scan
                foundDevices = pBLEScan->start(scanTime, false);          
                if (digitalRead(LED_BLUE) == HIGH)
                    digitalWrite(LED_BLUE,LOW);
                else    
                    digitalWrite(LED_BLUE,HIGH);    
                esp_task_wdt_reset();
                state++;
                vTaskDelay(4000);           
                break; 
            case 2:   // Clear BLE results
                pBLEScan->clearResults(); 
                esp_task_wdt_reset();
                state--;
                vTaskDelay(1000);
                break; 
        }     
    }
}

/*
void TaskSendMqtt( void *pvParameters ){
    
    for (;;)
    {   
        digitalWrite(LED_BLUE,HIGH);
        vTaskDelay(1000);
        digitalWrite(LED_BLUE,LOW);
        vTaskDelay(1000);
    }
}

void TaskReadBme680( void *pvParameters ){
    
    for (;;)
    {   
        digitalWrite(LED_RED,HIGH);
        vTaskDelay(1100);
        digitalWrite(LED_RED,LOW);
        vTaskDelay(1100);
    }
}
*/
