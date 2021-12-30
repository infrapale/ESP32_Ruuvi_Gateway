/**
 * RuuviTag Adafruit IO Gateway
 * @author TomHöglund 2021
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

//#include "RuuviTag.h"
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
void TaskReadLight( void *pvParameters );

static SemaphoreHandle_t wifi_avail;
 
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, IO_USERNAME, IO_KEY);


void setup() 
{
    BaseType_t rc;
    delay(4000);
    Serial.begin(115200);   
    
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED,HIGH);
    pinMode(LED_GREEN, OUTPUT);
    digitalWrite(LED_GREEN,HIGH);
    pinMode(LED_YELLOW, OUTPUT);
    digitalWrite(LED_YELLOW,HIGH);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_BLUE,HIGH);
 
    wifi_avail = xSemaphoreCreateBinary();
    assert(wifi_avail);
    rc = xSemaphoreGive(wifi_avail);
    StartTasks();
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

void StartTasks(void){
    BaseType_t rc;

    rc = xSemaphoreTake(wifi_avail,portMAX_DELAY);
    assert(rc == pdPASS);

    
    xTaskCreatePinnedToCore(
       TaskConnectWiFi
        ,  "TaskSendMqtt" 
        ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);
    
   
    xTaskCreatePinnedToCore(
       TaskConnectMqtt
        ,  "TaskSendMqtt" 
        ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
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
    int8_t ret;
    uint8_t retries = 6;
    BaseType_t rc;
    
    for (;;)
    {
        Serial.println("Connecting WiFi");
        Serial.println(ssid);
        Serial.println(password);
        
        WiFi.begin(ssid, password); 
   
        while (WiFi.status() != WL_CONNECTED)  //Check for the connection
        {
            digitalWrite(LED_YELLOW,LOW);
            vTaskDelay(1000);
            if (--retries == 0) 
            {
                Serial.println("WiFi Retry limit reached -> WDT reset…");
                while (1); 
            }
            else
            {
                Serial.println("Waiting for WiFi");              
            }
             
         }
         digitalWrite(LED_YELLOW,HIGH);
         Serial.println("Connected to WiFi");
         rc = xSemaphoreGive(wifi_avail);

         retries = 3;
         while (WiFi.status() == WL_CONNECTED) 
         {
            vTaskDelay(10000);
         }
         rc = xSemaphoreTake(wifi_avail,portMAX_DELAY);
         assert(rc == pdPASS);
    }
}



void TaskConnectMqtt( void *pvParameters ){
    int8_t ret;
    uint8_t retries = 3;
    BaseType_t rc;

    
    for (;;)
    {
        rc = xSemaphoreTake(wifi_avail,portMAX_DELAY);
        assert(rc == pdPASS);
        if (!mqtt.connected()) 
        {
            //Serial.println("Connecting to MQTT… ");              
            ret = mqtt.connect();
            if (ret != 0)    // connect will return 0 for connected
            {            
                Serial.println(mqtt.connectErrorString(ret));
                Serial.println("Retrying MQTT connection in 5 seconds…");
                mqtt.disconnect();          
                if (--retries == 0) 
                {
                    Serial.println("Retry limit reached -> WDT reset…");
                    // basically die and wait for WDT to reset me
                    while (1);
                }
                vTaskDelay(5000);

            }
            else
            {
                Serial.println("MQTT Connected!"); 
                retries = 3;               
            }
        }
        rc = xSemaphoreGive(wifi_avail);
        if (digitalRead(LED_GREEN) == HIGH)
            digitalWrite(LED_GREEN,LOW);
        else    
            digitalWrite(LED_GREEN,HIGH);
    }
}

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
