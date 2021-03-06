/**
 * RFM69 Adafruit IO Gateway
 * @author TomHöglund 2022
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
//#define  VILLA_ASTRID
//#define  LILLA_ASTRID
#define  H_MOKKULA
//#define PIRPANA
#include "secrets.h"

#include "config.h"
#include "helpers.h"
#include "radio_sensors.h"
#include "app_defs.h"

#define CAPTION_LEN               40      ///< Length of value name
#define MAC_ADDR_LEN              18      ///< Length of the BLE MAC address string
#define MQTT_UPDATE_INTERVAL_ms   60000   ///< MQTT update interval, one value per time

#define WDT_TIMEOUT           10
//#define ARDUINO_RUNNING_CORE  1
#define LED_RED               26
#define LED_GREEN             32
#define LED_YELLOW            33
#define LED_BLUE              25

#define RFM69_I2C_ADDR    0x20
#define RFM69_RESET       0x01
#define RFM69_CLR_RX      0x02
#define RFM69_CLR_TX      0x03
#define RFM69_GET_ID      0x05
#define RFM69_SEND_MSG    0x10
#define RFM69_TX_DATA     0x11
#define RFM69_RX_AVAIL    0x40
#define RFM69_RX_LOAD_MSG 0x41
#define RFM69_RX_RD_MSG1  0x42
#define RFM69_RX_RD_MSG2  0x43
#define RFM69_RX_RD_LEN   0x44
#define RFM69_TX_FREE     0x50

#define I2C_MAX_MEG_LEN   32
#define RFM69_MSG_LEN     64


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
    Adafruit_MQTT_Publish *aio_mqtt_publ;
    char          caption[CAPTION_LEN];
    value_type_et value_type;
    float         *data_ptr;
    bool          *updated_ptr;   
} sensor_st;



void StartTasks(void);
static SemaphoreHandle_t sema_wifi_avail;
static SemaphoreHandle_t sema_mqtt_avail;

 
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, IO_USERNAME, IO_KEY);
Adafruit_MQTT_Publish va_piha_temp1_feed = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/villaastrid.piha-temp1");
Adafruit_MQTT_Publish va_piha_temp2_feed = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/villaastrid.piha-temp2");
Adafruit_MQTT_Publish va_piha_hum_feed = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/villaastrid.piha-hum");
Adafruit_MQTT_Publish va_piha_light1_feed = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/villaastrid.piha-light1");
Adafruit_MQTT_Publish va_piha_light2_feed = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/villaastrid.piha-light2");
Adafruit_MQTT_Publish va_piha_pmb_feed = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/villaastrid.piha-pmb");

/// Sensor MQTT pointers, other data will be initialized in the setup function
sensor_st sensor[NBR_RADIO_SENSORS];

extern bool         loopTaskWDTEnabled;
static TaskHandle_t htask;




void setup() 
{
    BaseType_t rc;
    esp_err_t er;
    char feed_name[80];

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

    /// Define sensors
    //memset(sensor,0x00,sizeof(sensor));
    printf("Defining sensors: ");
    for (uint8_t indx = 0; indx < NBR_RADIO_SENSORS; indx++){
        radio_sensors_get_name(indx, sensor[indx].caption);
        sensor[indx].value_type = VALUE_TYPE_FLOAT;
        sensor[indx].data_ptr = radio_sensors_get_value_ptr(indx);
        sensor[indx].updated_ptr = radio_sensors_get_updated_ptr(indx);
        *sensor[indx].updated_ptr = false;
        printf("-%d",indx);
    }
    printf("-done\n");
    
        
    sensor[0].aio_mqtt_publ = &va_piha_temp1_feed;
    sensor[1].aio_mqtt_publ = &va_piha_temp2_feed;
    sensor[2].aio_mqtt_publ = &va_piha_hum_feed;
    sensor[3].aio_mqtt_publ = &va_piha_light1_feed;
    sensor[4].aio_mqtt_publ = &va_piha_light2_feed;
    sensor[5].aio_mqtt_publ = &va_piha_pmb_feed;

    
    for (uint8_t indx = 0; indx < NBR_RADIO_SENSORS; indx++){
        printf("%s\n",  sensor[indx].caption);
    }
    
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
       TaskReadRadio433
        ,  "TaskReadRadio433" 
        ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);
         

    /*
     xTaskCreatePinnedToCore(
       TaskScanBle
        ,  "TaskScanBle" 
        ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);
    */
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
                    printf("Connecting WiFi: %s %s\n",ssid, password);       
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
                sensor_indx  = 0;
                state++;
                break;
            case 1: //Re-run MQTT action
                wifi_timeout = 10;
                mqtt_timeout = 10;
                state++;
                break;
                
            case 2: // waiting for WiFi
                rc = xSemaphoreTake(sema_wifi_avail, 1000);
                if(rc == pdPASS) state++;
                else {
                    if(--wifi_timeout == 0) state = 99;
                }
                esp_task_wdt_reset();
                vTaskDelay(1000);
                break;
            case 3: // WiFi is available
                if (mqtt.connected()){
                    printf("MQTT was already connected\n");
                    state++;
                }
                else {                
                    printf("Connecting to MQTT…\n ");  
                    if (WiFi.status() != WL_CONNECTED)
                    {
                        printf("WiFi is not connected\n ");  
                        state = 99;  //restart
                    }
                    else{
                        ret = mqtt.connect();
                        if (ret != 0) {    // connect will return 0 for connected
                            printf("%s\n",mqtt.connectErrorString(ret));
                            printf("Retrying MQTT connection…\n");
                            mqtt.disconnect();          
                            if (--mqtt_timeout == 0) state = 6;
                        }
                        else {
                            esp_task_wdt_reset();
                            vTaskDelay(100);
                            state++;
                        }
                    }
                }
                
            case 4: // MQTT is connected
                printf("MQTT is Connected!\n"); 
                rc = xSemaphoreGive(sema_mqtt_avail);
                digitalWrite(LED_GREEN,HIGH);
                esp_task_wdt_reset();
                state++;
                vTaskDelay(100);
                break;
            case 5: // MQTT actions
                if (mqtt.connected()){
                    if (*sensor[sensor_indx].updated_ptr)
                    {
                        printf("%s %f\n",sensor[sensor_indx].caption,*sensor[sensor_indx].data_ptr);
                        //sensor[sensor_indx].ada_mqtt_publ->publish(*sensor[sensor_indx].data_ptr);      
                    }
                    if(++sensor_indx >= NBR_RADIO_SENSORS) sensor_indx = 0;
                    state++;
                    interval_sec = 15;

                } else {
                    interval_sec = 15;
                    state = 99;
                }
                esp_task_wdt_reset();
                vTaskDelay(1000);
                break;

            case 6: // Release WiFI and MQTT
                mqtt.disconnect();  
                rc = xSemaphoreGive(sema_wifi_avail);  
                state++;
                break;

            case 7: //Wait for next MQTT update
                if(--interval_sec == 0) state = 1;
                esp_task_wdt_reset();
                vTaskDelay(1000);          
                break;

            case 99: //
                printf("Retry limit reached -> WDT reset…\n");
                rc = xSemaphoreGive(sema_wifi_avail);
                rc = xSemaphoreGive(sema_mqtt_avail);
                vTaskDelay(10000);
                break;
            default:
                printf("Fatal error: incorrect MQTT state -> WDT reset…\n");
                vTaskDelay(100);
                state = 99;
                break;           
        }
    }
}



void TaskReadRadio433( void *pvParameters ){
    uint8_t     state = 0;
    uint8_t     n = 0;
    uint8_t     msg_len;
    char        json_msg[RFM69_MSG_LEN];
    uint8_t     indx;
    BaseType_t  rc;
    esp_err_t   er;
    
    er = esp_task_wdt_add(nullptr);
    Wire.begin();        // join i2c bus (address optional for master)
    
    for (;;)
    {   
        printf("Read Radio 433: %d\n", state);
        switch(state) {
            case 0:
                Wire.beginTransmission( (uint8_t)RFM69_I2C_ADDR);
                Wire.write(byte(RFM69_RESET)); 
                Wire.endTransmission();
                esp_task_wdt_reset();
                vTaskDelay(2000);
                state++;
                break;

            case 1:
                Wire.beginTransmission( (uint8_t)RFM69_I2C_ADDR);
                Wire.write(byte(RFM69_RX_AVAIL)); 
                Wire.endTransmission();
                Wire.requestFrom(RFM69_I2C_ADDR, 1);    // request 1 bytes from peripheral
                while (Wire.available()) { // peripheral may send less than requested
                    n = Wire.read(); // receive a byte as character
                    printf("Number of messages %d\n",n);
                }   
                if (n> 0){
                    state++;
                     vTaskDelay(100);
                }
                else {
                     vTaskDelay(2000);
                }
                esp_task_wdt_reset();
                break;

            case 2:
                Wire.beginTransmission( (uint8_t)RFM69_I2C_ADDR);
                Wire.write(byte(RFM69_RX_LOAD_MSG)); 
                Wire.write(byte(RFM69_RX_RD_LEN)); 
                Wire.endTransmission();
                Wire.requestFrom(RFM69_I2C_ADDR, 1);    // request 1 bytes from peripheral
                while (Wire.available()) { // peripheral may send less than requested
                    msg_len = Wire.read(); // receive a byte as character
                    printf("msg_len = %d\n",msg_len);
                }   
                if (msg_len > 0){
                    state++;
                    vTaskDelay(100);
                }
                else {
                    state = 0;
                    vTaskDelay(2000);
                }
                esp_task_wdt_reset();
                break;

            case 3:         
                Wire.beginTransmission( (uint8_t)RFM69_I2C_ADDR);
                Wire.write(byte(RFM69_RX_RD_MSG1)); 
                Wire.endTransmission();
                indx = 0;
                json_msg[0] = 0x00;
            
                Wire.requestFrom(RFM69_I2C_ADDR, I2C_MAX_MEG_LEN);    // request first half message
                while (Wire.available()) { // peripheral may send less than requested
                    char c = Wire.read(); // receive a byte as character
                    if (indx < RFM69_MSG_LEN) json_msg[indx++] = c;
                    printf("%c",c);
                }

                Wire.beginTransmission( (uint8_t)RFM69_I2C_ADDR);
                Wire.write(byte(RFM69_RX_RD_MSG2)); 
                Wire.endTransmission();
            
                Wire.requestFrom(RFM69_I2C_ADDR, I2C_MAX_MEG_LEN);    // request second half message
                while (Wire.available()) { // peripheral may send less than requested
                    char c = Wire.read(); // receive a byte as character
                    if (indx < RFM69_MSG_LEN) json_msg[indx++] = c;
                    printf("%c",c);
                }   
                printf("\n"); 
                parse_msg(json_msg);
                print_radio_sensors();
  
                state++;
                vTaskDelay(50);
                esp_task_wdt_reset();
                break;
            case 4:
                state = 1;
                vTaskDelay(1000);
                esp_task_wdt_reset();
                break;
        }
        digitalWrite(LED_BLUE,HIGH);
        vTaskDelay(1000);
        digitalWrite(LED_BLUE,LOW);
        vTaskDelay(1000);
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
