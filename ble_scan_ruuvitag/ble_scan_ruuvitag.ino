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
#define  H_MOKKULA
#include "secrets.h"
//#include <HTTPClient.h>
#include "config.h"

WiFiClient client;


int scanTime = 5; //In seconds
BLEScan* pBLEScan;
int temp, hum, pressure, ax, ay, az, voltage_power, voltage, power, rssi_ruuvi, movement, measurement;
float temp_fp;
String payload;
String MAC_add = "ed:9a:ab:c6:30:72"; //All the identified MAC addresses will go in one String

const char* ssid     = WIFI_SSID;              //Main Router      
const char* password = WIFI_PASS;            //Main Router Password
const char* url = "UBEAC GATEWAY URL HERE"; 

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, IO_USERNAME, IO_KEY);
// Adafruit_MQTT_Subscribe ledControl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ledControl");
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/home-tampere.esp32test-temp");


//Converts hexadecimal values to decimal values
int hexadecimalToDecimal(String hexVal)
{
    int len = hexVal.length();
    int base = 1;

    int dec_val = 0;

    for (int i = len - 1; i >= 0; i--)
    {
        if (hexVal[i] >= '0' && hexVal[i] <= '9')
        {
            dec_val += (hexVal[i] - 48) * base;

            base = base * 16;
        }
        else if (hexVal[i] >= 'A' && hexVal[i] <= 'F')
        {
            dec_val += (hexVal[i] - 55) * base;

            base = base * 16;
        }
    }
    return dec_val;
}

//Decodes RUUVI raw data and arranges it in an array
void decodeRuuvi(String hex_data, int rssi){
    Serial.println(hex_data);
    Serial.println(hex_data.substring(4, 6));
    /**
     *  9904034F1545C8DFFF52004E03E90B59
     *  9904
     *  03
     *  4F
     *  15 45
     *  C8DF
     *  FF52004E03E90B59
     */
    if(hex_data.substring(4, 6) == "03"){
        temp = hexadecimalToDecimal(hex_data.substring(8, 10));
        temp_fp = (float) temp + (float) hexadecimalToDecimal(hex_data.substring(10, 12)) / 100.0;
        hum = hexadecimalToDecimal(hex_data.substring(6, 8));
        pressure = hexadecimalToDecimal(hex_data.substring(12, 16))-50000;

        //ax = hexadecimalToDecimal(hex_data.substring(18, 22));
        //ay = hexadecimalToDecimal(hex_data.substring(22, 26));
        //az = hexadecimalToDecimal(hex_data.substring(26, 30));     

        if(ax > 0xF000){
          ax = ax - (1 << 16);
        }
        if(ay > 0xF000){
          ay = ay - (1 << 16);
        }
        if (az > 0xF000){
          az = az - (1 << 16);
        }

        voltage_power = hexadecimalToDecimal(hex_data.substring(30, 34));
        voltage = (int)((voltage_power & 0x0b1111111111100000) >> 5) + 1600;
        power = (int)(voltage_power & 0x0b0000000000011111) - 40;

        rssi_ruuvi = rssi;

        movement = hexadecimalToDecimal(hex_data.substring(34, 36));
        measurement = hexadecimalToDecimal(hex_data.substring(36, 40));
    }
}

//Converts decoded RUUVI data into uBeac JSON format
void uBeacRuuvi(){
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
  Serial.println(payload);

}

//Class that scans for BLE devices
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      //Scans for specific BLE MAC addresses 
      Serial.println(advertisedDevice.getAddress().toString().c_str());
      if(MAC_add.indexOf(advertisedDevice.getAddress().toString().c_str()) >= 0){ //If the scanned MAC address is in the identified MAC address String
        String raw_data = String(BLEUtils::buildHexData(nullptr, (uint8_t*)advertisedDevice.getManufacturerData().data(), advertisedDevice.getManufacturerData().length()));
        raw_data.toUpperCase();
        decodeRuuvi(raw_data, advertisedDevice.getRSSI());
        uBeacRuuvi();
        Serial.println(raw_data);
      }  
      
      //  Sends RuuviTag JSON data to IoT clod platform
      if(WiFi.status()== WL_CONNECTED){ 
 
        //HTTPClient http;   
        /*
        http.begin(url);  
        int httpResponseCode = http.POST(payload); 
 
        if(httpResponseCode>0){
          String response = http.getString(); 
          Serial.println(httpResponseCode);
        }
        http.end();
        */
      }else{
        Serial.println("Error in WiFi connection");    
      } 
    }
};

void setup() {
  Serial.begin(115200);
  
  //Connect to Local WiFi
  delay(4000);   //Delay needed before calling the WiFi.begin
 
  WiFi.begin(ssid, password); 
 
  while (WiFi.status() != WL_CONNECTED) { //Check for the connection
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
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

}


void MQTT_connect()
{
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT… ");

  uint8_t retries = 3;

  while ((ret = mqtt.connect()) != 0) { 
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

void loop() {
    MQTT_connect();
    BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
    Serial.println(temp_fp);
 
    if (!temperature.publish(temp_fp)) {
        Serial.println(F("Failed"));
    }
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
    delay(10000);
}
