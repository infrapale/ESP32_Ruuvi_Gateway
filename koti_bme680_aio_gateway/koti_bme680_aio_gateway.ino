/**
 * RuuviTag Adafruit IO Gateway
 * @author TomHöglund 2021
 * 
 * https://github.com/ruuvi/ruuvi-sensor-protocols/blob/master/dataformat_03.md
 * https://tutorial.cytron.io/2020/01/15/send-sensors-data-to-adafruit-io-using-esp32/
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html
 * https://github.com/adafruit/Adafruit_MQTT_Library/blob/master/Adafruit_MQTT.h
 * 
 */
 
#include <Wire.h>
#include <SPI.h>
//#include <BLEDevice.h>
///#include <BLEUtils.h>
//#include <BLEScan.h>
//#include <BLEAdvertisedDevice.h>
//#include <BLEAddress.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <esp_task_wdt.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
//#include <bme680.h>
//#include <bme680_defs.h>

#define ARDUINO_RUNNING_CORE 1

/// SSID Definitions
//#define  VILLA_ASTRID
//#define  H_MOKKULA
#define PIRPANA
#include "secrets.h"

#include "config.h"
#include "helpers.h"

#define NBR_SENSORS               3       ///< Number of sensor values
#define CAPTION_LEN               40      ///< Length of value name
#define MAC_ADDR_LEN              18      ///< Length of the BLE MAC address string
#define MQTT_UPDATE_INTERVAL_ms   60000   ///< MQTT update interval, one value per time

#define WDT_TIMEOUT           10
//#define ARDUINO_RUNNING_CORE  1

#define LED_YELLOW   33
#define LDR_PIN      34
#define NBR_LDR_RES  5

int scanTime = 5; //In seconds
//BLEScan* pBLEScan;
Adafruit_BME680 bme; // I2C
uint16_t ldr_value[NBR_LDR_RES];
uint8_t  ldr_select_pin[NBR_LDR_RES] = {15,16,17,18,19};

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
    float         value;
    bool          updated;
} sensor_st;



void StartTasks(void);
static SemaphoreHandle_t sema_wifi_avail;
static SemaphoreHandle_t sema_mqtt_avail;

 
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, IO_USERNAME, IO_KEY);
Adafruit_MQTT_Subscribe *subscription;

//  Koti
Adafruit_MQTT_Publish koti_sisa_temp    = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.sisa-temperature");
Adafruit_MQTT_Publish koti_sisa_hum     = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.sisa-temperature");
Adafruit_MQTT_Publish koti_sisa_light   = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.sisa-light");
Adafruit_MQTT_Subscribe at_home_state   = Adafruit_MQTT_Subscribe(&mqtt, IO_USERNAME "/feeds/home-tampere.at-home-state");




/// Sensor MQTT pointers, other data will be initialized in the setup function
sensor_st sensor[NBR_SENSORS]= 
{
    { &koti_sisa_temp, "", VALUE_TYPE_FLOAT, NULL,NULL,0.0,false}, 
    { &koti_sisa_hum, "", VALUE_TYPE_FLOAT, NULL,NULL,0.0,false}, 
    { &koti_sisa_light, "", VALUE_TYPE_FLOAT, NULL,NULL,0.0,false}, 
};


extern bool         loopTaskWDTEnabled;
static TaskHandle_t htask;


void at_home_state_callback(char *data, uint16_t len)
{
    Serial.print("at home state: ");
    Serial.println(data);
    if ((strcmp(data,"On") == 0))
    {
        digitalWrite(LED_YELLOW,HIGH);
    }
    else
    {
        digitalWrite(LED_YELLOW,LOW);
    }
}

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
    
    TaskHandle_t h = xTaskGetIdleTaskHandle();
    er = esp_task_wdt_status(h);
    if ( er != ESP_OK ) 
    {
        er = esp_task_wdt_add(h); // Add Idle task
        assert(er == ESP_OK);
    }

    at_home_state.setCallback(at_home_state_callback);
    mqtt.subscribe(&at_home_state);

    pinMode(LED_YELLOW, OUTPUT);
    digitalWrite(LED_YELLOW,LOW);

    pinMode(LDR_PIN,INPUT);
    //select_ldr(0);

    
    for (uint8_t i = 0; i < NBR_SENSORS; i++){
        sensor[i].data_ptr    = &sensor[i].value;
        sensor[i].updated_ptr = &sensor[i].updated;       
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
       TaskReadBme680
        ,  "TaskReadBme680"   // A name just for humans
        ,  8000  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
       TaskReadLight
        ,  "TaskReadLight"   // A name just for humans
        ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);            
    
    
}

static void TaskConnectWiFi( void *pvParameters ){
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
        printf("WiFi state: %d\n", state);
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
                    vTaskDelay(1000);
                    if (--retries == 0) state = 3;
                    else Serial.println("Waiting for WiFi"); 
                }
                else {
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
    uint8_t   sensor_indx = 0;
    uint8_t   interval_sec;
    
    state = 0;
    er = esp_task_wdt_add(nullptr);
    assert(er == ESP_OK);
    mqtt.subscribe(&at_home_state);
    
    for (;;)
    { 
        printf("MQTT state: %d\n", state);
        switch(state) {
            case 0: // initial
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
                if (mqtt.connected())
                {
                    printf("MQTT was already connected\n");
                    state++;
                }
                else 
                {                
                    printf("Connecting to MQTT…\n ");  
                    if (WiFi.status() != WL_CONNECTED)
                    {
                        printf("WiFi is not connected\n ");  
                        state = 99;  //restart
                    }
                    else
                    {
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
                break;
            case 4: // MQTT is connected
                printf("MQTT is Connected!\n"); 
                rc = xSemaphoreGive(sema_mqtt_avail);
                esp_task_wdt_reset();
                state++;
                vTaskDelay(100);
                break;
            case 5: // MQTT actions
                if (mqtt.connected())
                {
                    if (*sensor[sensor_indx].updated_ptr)
                    {
                        printf("%s %f\n",sensor[sensor_indx].caption,*sensor[sensor_indx].data_ptr);
                        sensor[sensor_indx].ada_mqtt_publ->publish(*sensor[sensor_indx].data_ptr);      
                    }
                    if(++sensor_indx >= NBR_SENSORS) sensor_indx = 0;
                    state++;
                    interval_sec = 15;

                } else 
                {
                    interval_sec = 60;
                    state = 99;
                }
                esp_task_wdt_reset();
                vTaskDelay(1000);
                break;

            case 6: 
                rc = xSemaphoreGive(sema_wifi_avail);
                state++;
                break;

            case 7: //Wait for next MQTT update
                if(--interval_sec == 0) state = 1;
                //mqtt.processPackets(1000);
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
                printf("Fatal error: incorrect MQTT state -> WDT reset…%d\n",state);
                vTaskDelay(100);
                state = 99;
                break;           
        }
    }
}

void TaskReadBme680( void *pvParameters ){
    unsigned long endTime;
    esp_err_t er;
    
    er = esp_task_wdt_add(nullptr);
    assert(er == ESP_OK);

    if (!bme.begin(0x77)) {
        while (1){
            printf("Could not find a valid BME680 sensor, check wiring!");
            vTaskDelay(10000);
        }
    }
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    //bme.setGasHeater(320, 150); // 320*C for 150 ms
    bme.setGasHeater(0, 0); // 320*C for 150 ms

    for (;;)
    {   
        endTime = bme.beginReading();
        if (endTime == 0) {
            printf("Failed to begin reading \n");
        } 
        else {
            esp_task_wdt_reset();
            vTaskDelay(1000);     
            if (!bme.endReading()) {
                printf("Failed to complete reading\n");
            }
            sensor[0].value = bme.temperature;
            sensor[0].updated = true;
            sensor[1].value = bme.humidity;
            sensor[1].updated = true;
            
        }
        vTaskDelay(4000);     
    }  
}

void TaskReadLight( void *pvParameters ){
    uint8_t ldr_indx = 0;

    esp_err_t er;
    
    er = esp_task_wdt_add(nullptr);
    assert(er == ESP_OK);

    
    for (;;)
    {    
        ldr_value[ldr_indx] = analogRead(LDR_PIN);
        if (++ldr_indx >= NBR_LDR_RES ) {
            ldr_indx = 0;
        }
        //select_ldr(ldr_indx);
        esp_task_wdt_reset();
 
        sensor[2].value = (float) ldr_value[3];
        sensor[2].updated = true;
        vTaskDelay(2000);       
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
