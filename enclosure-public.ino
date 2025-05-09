#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_adc_cal.h>
#include "DHT.h"
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <queue>

#define REF_VOLTAGE 1108
// --------------------------
#define DHTPIN 27     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);
// --------------------------

#define ONE_WIRE_BUS 14

// OneWire oneWire(ONE_WIRE_BUS);
// DallasTemperature sensors(&oneWire);

// const uint8_t TMP_PIN = 33;
const uint8_t TMP_PIN_1 = 35;
const uint8_t HEATER_PWR_PIN = 32;
int isHeaterOn = 0;
int TEMP_ON = 60 - 10;
int TEMP_OFF = 70 - 10;
esp_adc_cal_characteristics_t *adc_chars = new esp_adc_cal_characteristics_t;

// WIFI
const char* ssid = "wifi";
const char* password = "password";
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

// function declarations
uint16_t avgAnalogRead(uint8_t pin, uint16_t samples = 8);
void makeHAReq(float num1, float num2, float num3, float num4, float num5, int num6);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello World! new one");

  // PIN MODE SET UP
  pinMode(ONE_WIRE_BUS, INPUT);
  // pinMode(TMP_PIN, INPUT);
  pinMode(TMP_PIN_1, INPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(HEATER_PWR_PIN, OUTPUT);

  // ADC 1 CONFIG
  adc1_config_width(ADC_WIDTH_BIT_11);
  adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_0);
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_11, REF_VOLTAGE, adc_chars);

  // sensors.begin();
  dht.begin();
  // adc2_vref_to_gpio(GPIO_NUM_25);

  // WIFI SET UP  
  WiFi.mode(WIFI_STA); //Optional
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");

  while(WiFi.status() != WL_CONNECTED){
      Serial.print(".");
      delay(100);
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus 
  /********************************************************************/
  // DS18B20 
  // sensors.requestTemperatures(); // Send the command to get temperature readings
  // float temp_3 = sensors.getTempCByIndex(0); //DS18B20 sensor

  // You can have more than one DS18B20 on the same bus.  
  // 0 refers to the first IC on the wire 

  // READ ADC
  // float Vtemp_1 = avgAnalogRead(TMP_PIN); delay(1000);
  float Vtemp_2 = avgAnalogRead(TMP_PIN_1); delay(1000);
  // Serial.println(Vtemp_1);
  Serial.println(Vtemp_2);
  
  // READ FROM DHT
  // -- Reading temperature or humidity takes about 250 milliseconds!
  // -- Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // Read T as C (default)

  // -- Check if any reads failed
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
  }
  Serial.println("DHT: " + String(h) + "%, " + String(t) + " C");

  
  // float temp_1 = ( 5.506 - sqrt(30.316036 + 0.00704 * (870.6 - Vtemp_1) ) ) / ( 2 * -0.00176) + 30;
  float temp_2 = ( 5.506 - sqrt(30.316036 + 0.00704 * (870.6 - Vtemp_2) ) ) / ( 2 * -0.00176) + 30; //Eqn from datasheet
  // Serial.println("temp 1 = " + String(temp_1, 1) );
  Serial.println("temp 2 = " + String(temp_2, 1) );
  // Serial.println("temp 3 = " + String(temp_3, 1) );
  Serial.println("heater = " + String(isHeaterOn));
  Serial.println("humidity = " + String(h));
  Serial.println("DHT T = " + String(t, 1));

  // Control Heater
  if( temp_2 > TEMP_OFF ) {digitalWrite(HEATER_PWR_PIN, LOW); isHeaterOn = 0;} // temp_1 > TEMP_OFF ||  || temp_3 > TEMP_OFF
  else if(temp_2 < TEMP_ON) {digitalWrite(HEATER_PWR_PIN, HIGH); isHeaterOn = 1;} // temp_1 < TEMP_ON ||  || temp_3 < TEMP_ON
  
  // MAX TEMP SAFETY SHUTOFF HEATER
  if(temp_2 > 75 || t > 75) { //temp_1 > 75 || || temp_3 > 75
    digitalWrite(HEATER_PWR_PIN, LOW);
    while(true){
      delay(1000);
      Serial.println("Error: Max Temp Reached");
    }
  }

  // MAKE API REQ
  makeHAReq(0, temp_2, 0, h, t, isHeaterOn); //temp_1, temp_3

}

uint16_t avgAnalogRead(uint8_t pin, uint16_t samples) {
  adc1_channel_t chan;
  switch (pin) {
    case 32:
      chan = ADC1_CHANNEL_4; break;
    case 33:
      chan = ADC1_CHANNEL_5; break;
    case 34:
      chan = ADC1_CHANNEL_6; break;
    case 35:
      chan = ADC1_CHANNEL_7; break;
    case 36:
      chan = ADC1_CHANNEL_3; break;
    case 39:
      chan = ADC1_CHANNEL_0; break;
  }
  uint32_t sum = 0;
  for (int x=0; x<samples; x++) {
    sum += adc1_get_raw(chan);
  }
  sum /= samples;
  return esp_adc_cal_raw_to_voltage(sum, adc_chars);
}

void makeHAReq(float num1, float num2, float num3, float num4, float num5, int num6){
  String serverName_1 = "http://10.0.0.1:8123/api/states/input_number.heater_t1";
  String serverName_2 = "http://10.0.0.1:8123/api/states/input_number.heater_t2";
  String serverName_3 = "http://10.0.0.1:8123/api/states/input_number.heater_t3";
  String serverName_4 = "http://10.0.0.1:8123/api/states/input_number.heater_dht_h";
  String serverName_5 = "http://10.0.0.1:8123/api/states/input_number.heater_dht_t";
  String serverName_6 = "http://10.0.0.1:8123/api/states/input_number.isheateron"; // input_number.isheateron

  //Send an HTTP POST request every 10 minutes
  if ((millis() - lastTime) > timerDelay) {
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      HTTPClient http;

      String serverPath = serverName_1;
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());
      http.addHeader("Authorization", "Bearer ------");
      http.addHeader("content-type", "application/json"); // application/x-www-form-urlencoded
      
      // Send HTTP GET request
      String httpRequestData = "{\"state\":\"" + String(num1, 1) + "\"}";           
      int httpResponseCode = http.POST(httpRequestData);
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        // String payload = http.getString();
        // Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();

      // --------------------------------------------------------------------------------------------------- 2
      serverPath = serverName_2;
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());
      http.addHeader("Authorization", "Bearer ------");
      http.addHeader("content-type", "application/json"); // application/x-www-form-urlencoded
      
      // Send HTTP GET request
      httpRequestData = "{\"state\":\"" + String(num2, 1) + "\"}";           
      httpResponseCode = http.POST(httpRequestData);
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        // String payload = http.getString();
        // Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();

      // --------------------------------------------------------------------------------------------------- 3
      serverPath = serverName_3;
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());
      http.addHeader("Authorization", "Bearer ----");
      http.addHeader("content-type", "application/json"); // application/x-www-form-urlencoded
      
      // Send HTTP GET request
      httpRequestData = "{\"state\":\"" + String(num3, 1) + "\"}";           
      httpResponseCode = http.POST(httpRequestData);
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        // String payload = http.getString();
        // Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();

      // --------------------------------------------------------------------------------------------------- 4
      serverPath = serverName_4;
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());
      http.addHeader("Authorization", "Bearer ----");
      http.addHeader("content-type", "application/json"); // application/x-www-form-urlencoded
      
      // Send HTTP GET request
      httpRequestData = "{\"state\":\"" + String(num4, 1) + "\"}";           
      httpResponseCode = http.POST(httpRequestData);
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        // String payload = http.getString();
        // Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();

      // --------------------------------------------------------------------------------------------------- 5
      serverPath = serverName_5;
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());
      http.addHeader("Authorization", "Bearer -----");
      http.addHeader("content-type", "application/json"); // application/x-www-form-urlencoded
      
      // Send HTTP GET request
      httpRequestData = "{\"state\":\"" + String(num5, 1) + "\"}";           
      httpResponseCode = http.POST(httpRequestData);
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        // String payload = http.getString();
        // Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();

      // --------------------------------------------------------------------------------------------------- 6
      serverPath = serverName_6;
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());
      http.addHeader("Authorization", "Bearer -----");
      http.addHeader("content-type", "application/json"); // application/x-www-form-urlencoded
      
      // Send HTTP GET request
      httpRequestData = "{\"state\":\"" + String(num6) + "\"}";           
      httpResponseCode = http.POST(httpRequestData);
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        // String payload = http.getString();
        // Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();

      
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
  
}

// ESP32-D0WDQ6

// https://github.com/espressif/arduino-esp32/issues/1804

