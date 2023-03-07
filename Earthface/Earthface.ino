#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <ArduinoJson.h>
#include "Wire.h"
#include "AS73211.h"
#include <MatrixMath.h>
#include "EEPROM.h"

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define CLK_PIN 34
#define DATA_PIN 13

//#define x_con 0.905
//#define y_con 1
//#define z_con 1.089

#define RED_LED 32
#define GREEN_LED 33
#define BLUE_LED 2

#define N 3
#define slot_PIN 35

#define Battery_enable_pin 15
#define Battery_analog_pin 36

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 5

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
char char_received;
String fromSerial = "";
String jsonData = "";
bool start = true;
bool end = false;

uint8_t red[10], green[10], blue[10];
float x[10], y[10], z[10];
float c_x[10], c_y[10], c_z[10];

// setting PWM properties
const int ledPin = 5;
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
int dutycycle = 100;

bool init_slot = true;

uint16_t X, Y, Z;

mtx_type C[N][N] =
{

};

mtx_type T[N][N] =
{
 
{1.4628067, -0.1840623, -0.2743606},
{-0.5217933, 1.4472381,  0.0677227},
{ 0.0349342, -0.0968930,  1.2884099} 

};
mtx_type v[N];        // This is a row vector
mtx_type w[N];        // result of calibrated xyz matrix
mtx_type r[N];        // result of calibrated srgb  matrix
mtx_type maxVal = 10; // maxValimum random matrix entry range
float val_bfr_conver[3];
float rawSrgb[3];

uint8_t init_count = 0;

unsigned long previousMillis = 0;

unsigned long blue_previousMillis = 0;
int blue_ledState = LOW;

unsigned long red_previousMillis = 0;
int red_ledState = LOW;

unsigned long green_previousMillis = 0;
int green_ledState = LOW;

bool send_data_flag = false;

int R, G, B;

String rx_data;

AS73211 colour(0x74);

int addr;
#define EEPROM_SIZE 162

template <class T>
int EEPROM_writeAnything(int ee, const T &value)
{
  const byte *p = (const byte *)(const void *)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}

template <class T>
int EEPROM_readAnything(int ee, T &value)
{
  byte *p = (byte *)(void *)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return i;
}

void calibration_matrix(uint16_t a, uint16_t b, uint16_t c)
{

  /*
    v[0] = ((double)a*x_con)/1000;   // (double)a/10;
    v[1] = ((double)b*y_con)/1000;   // (double)b/10;
    v[2] = ((double)c*z_con)/1000;  // (double)c/10;
  */

  v[0] = ((double)a / 1000); // (double)a/10;
  v[1] = ((double)b / 1000); // (double)b/10;
  v[2] = ((double)c / 1000); // (double)c/10;

  // Serial.printf("\n\n%d,%d,%d\n\n",R,G,B);

  Matrix.Multiply((mtx_type *)C, (mtx_type *)v, N, N, 1, (mtx_type *)w);
  // Matrix.Print((mtx_type*)C, N, N, "C");
  // Matrix.Print((mtx_type*)v, N, 1, "v");
  // Matrix.Print((mtx_type*)w, N, 1, "w");

//  Serial.printf("\n\n v = %f,%f,%f\n\n", v[0], v[1], v[2]);
}

float adj(float C)
{
  if (abs(C) < 0.0031308) // abs(C)
  {
    return 3294.6 * C; // return 12.92 * C; //return 3294.6 * C;
  }

  return 269.025 * pow(abs(C), 0.41666) - 14.025; // return 1.055 * pow(C, 0.41666) - 0.055; // return 269.025 * pow(C, 0.41666) - 14.025;
}

void xyz_to_rgb_conversion()
{

  Matrix.Multiply((mtx_type *)T, (mtx_type *)w, N, N, 1, (mtx_type *)r);

  R = adj(r[0]);
  G = adj(r[1]);
  B = adj(r[2]);

  if (R < 0)
    R = 0;
  if (G < 0)
    G = 0;
  if (B < 0)
    B = 0;

  if (R > 255)
    R = 255;
  if (G > 255)
    G = 255;
  if (B > 255)
    B = 255;

 // Serial.printf("\n\n%d,%d,%d\n\n", R, G, B);
}

void colorsensor_Data(uint16_t temp)
{

  for (int t = 0; t < 2; t++)
  {
    X = colour.color6_readData(_COLOR6_MREG_MEASUREMENT_X_CHANNEL);
    Y = colour.color6_readData(_COLOR6_MREG_MEASUREMENT_Y_CHANNEL);
    Z = colour.color6_readData(_COLOR6_MREG_MEASUREMENT_Z_CHANNEL);
    delay(1000);
  }

  //X = ((double)X * x_con); // (double)a/10;
  //Y = ((double)Y * y_con); // (double)b/10;
  //Z = ((double)Z * z_con); // (double)c/10;

  delay(10);

  // Serial.print(x);
  // Serial.print("\t");
  // Serial.print(y);
  // Serial.print("\t");
  // Serial.print(z);
  // Serial.println();

  x[temp] = (float)X / 10;
  y[temp] = (float)Y / 10;
  z[temp] = (float)Z / 10;

  calibration_matrix(X, Y, Z);

  c_x[temp] = (float)w[0];
  c_y[temp] = (float)w[1];
  c_z[temp] = (float)w[2];

  xyz_to_rgb_conversion();

  red[temp] = R;
  green[temp] = G;
  blue[temp] = B;
}

int battery_value = 0;

void readJsonData()
{
  StaticJsonBuffer<2150> jsonBuffer;
  JsonObject &root = jsonBuffer.createObject();

  root["type"] = "testdata";
  root["number"] = 1;
  root["testId"] = "TEST001";

  // Create the "analog" array
  JsonArray &value1 = root.createNestedArray("rgbValues");

  JsonArray &v1rgb1 = value1.createNestedArray();
  v1rgb1.add(red[0]);
  v1rgb1.add(green[0]);
  v1rgb1.add(blue[0]);

  JsonArray &v1rgb2 = value1.createNestedArray();
  v1rgb2.add(red[1]);
  v1rgb2.add(green[1]);
  v1rgb2.add(blue[1]);

  JsonArray &v1rgb3 = value1.createNestedArray();
  v1rgb3.add(red[2]);
  v1rgb3.add(green[2]);
  v1rgb3.add(blue[2]);

  JsonArray &v1rgb4 = value1.createNestedArray();
  v1rgb4.add(red[3]);
  v1rgb4.add(green[3]);
  v1rgb4.add(blue[3]);

  JsonArray &v1rgb5 = value1.createNestedArray();
  v1rgb5.add(red[4]);
  v1rgb5.add(green[4]);
  v1rgb5.add(blue[4]);

  JsonArray &v1rgb6 = value1.createNestedArray();
  v1rgb6.add(red[5]);
  v1rgb6.add(green[5]);
  v1rgb6.add(blue[5]);

  JsonArray &v1rgb7 = value1.createNestedArray();
  v1rgb7.add(red[6]);
  v1rgb7.add(green[6]);
  v1rgb7.add(blue[6]);

  JsonArray &v1rgb8 = value1.createNestedArray();
  v1rgb8.add(red[7]);
  v1rgb8.add(green[7]);
  v1rgb8.add(blue[7]);

  JsonArray &v1rgb9 = value1.createNestedArray();
  v1rgb9.add(red[8]);
  v1rgb9.add(green[8]);
  v1rgb9.add(blue[8]);

  JsonArray &v1rgb10 = value1.createNestedArray();
  v1rgb10.add(red[9]);
  v1rgb10.add(green[9]);
  v1rgb10.add(blue[9]);

  JsonArray &rawValues = root.createNestedArray("rawValues");

  JsonArray &rv1 = rawValues.createNestedArray();
  rv1.add(x[0]);
  rv1.add(y[0]), rv1.add(z[0]);

  JsonArray &rv2 = rawValues.createNestedArray();
  rv2.add(x[1]);
  rv2.add(y[1]), rv2.add(z[1]);

  JsonArray &rv3 = rawValues.createNestedArray();
  rv3.add(x[2]);
  rv3.add(y[2]), rv3.add(z[2]);

  JsonArray &rv4 = rawValues.createNestedArray();
  rv4.add(x[3]);
  rv4.add(y[3]), rv4.add(z[3]);

  JsonArray &rv5 = rawValues.createNestedArray();
  rv5.add(x[4]);
  rv5.add(y[4]), rv5.add(z[4]);

  JsonArray &rv6 = rawValues.createNestedArray();
  rv6.add(x[5]);
  rv6.add(y[5]), rv6.add(z[5]);

  JsonArray &rv7 = rawValues.createNestedArray();
  rv7.add(x[6]);
  rv7.add(y[6]), rv7.add(z[6]);

  JsonArray &rv8 = rawValues.createNestedArray();
  rv8.add(x[7]);
  rv8.add(y[7]), rv8.add(z[7]);

  JsonArray &rv9 = rawValues.createNestedArray();
  rv9.add(x[8]);
  rv9.add(y[8]), rv9.add(z[8]);

  JsonArray &rv10 = rawValues.createNestedArray();
  rv10.add(x[9]);
  rv10.add(y[9]), rv10.add(z[9]);

  JsonArray &cldValues = root.createNestedArray("calibratedValues");

  JsonArray &cv1 = cldValues.createNestedArray();
  cv1.add(c_x[0]);
  cv1.add(c_y[0]), cv1.add(c_z[0]);

  JsonArray &cv2 = cldValues.createNestedArray();
  cv2.add(c_x[1]);
  cv2.add(c_y[1]), cv2.add(c_z[1]);

  JsonArray &cv3 = cldValues.createNestedArray();
  cv3.add(c_x[2]);
  cv3.add(c_y[2]), cv3.add(c_z[2]);

  JsonArray &cv4 = cldValues.createNestedArray();
  cv4.add(c_x[3]);
  cv4.add(c_y[3]), cv4.add(c_z[3]);

  JsonArray &cv5 = cldValues.createNestedArray();
  cv5.add(c_x[4]);
  cv5.add(c_y[4]), cv5.add(c_z[4]);

  JsonArray &cv6 = cldValues.createNestedArray();
  cv6.add(c_x[5]);
  cv6.add(c_y[5]), cv6.add(c_z[5]);

  JsonArray &cv7 = cldValues.createNestedArray();
  cv7.add(c_x[6]);
  cv7.add(c_y[6]), cv7.add(c_z[6]);

  JsonArray &cv8 = cldValues.createNestedArray();
  cv8.add(c_x[7]);
  cv8.add(c_y[7]), cv8.add(c_z[7]);

  JsonArray &cv9 = cldValues.createNestedArray();
  cv9.add(c_x[8]);
  cv9.add(c_y[8]), cv9.add(c_z[8]);

  JsonArray &cv10 = cldValues.createNestedArray();
  cv10.add(c_x[9]);
  cv10.add(c_y[9]), cv10.add(c_z[9]);

  root["batt"] = battery_value;

  root["@"] = root.measureLength();

  root.printTo(jsonData);

  // Serial.println(jsonData);
}

void check_battery_status()
{


  for (int i = 0; i < 5; i++)
  {
    digitalWrite(Battery_enable_pin, HIGH);
    battery_value = battery_value + analogRead(Battery_analog_pin);
    digitalWrite(Battery_enable_pin, LOW);
  }

  //Serial.println(battery_value/5);
  battery_value /= 5;  

  if (battery_value < 2000)
  {
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    delay(500);

    //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    delay(10);

    Serial.println("Going to sleep now...");
    delay(1000);
    digitalWrite(RED_LED, LOW);
    //esp_deep_sleep_start();
  }
}

void scan_data()
{
  static uint16_t state = 0, counter = 0, rcounter = 0;
  static uint8_t slot = 0;

  if ((slot == 0) && (init_slot == true))
  {
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    delay(250);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    Serial.println(slot);
    colorsensor_Data(slot);
    delay(500);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, HIGH);
    init_slot = false;
  }

  delayMicroseconds(10); // Simulate doing somehing else as well.

  state = (state << 1) | digitalRead(CLK_PIN) | 0xe000;

  if (state == 0xf000)
  {
    state = 0x0000;
    if (digitalRead(DATA_PIN))
    {
      // red[counter] = counter;
      // green[counter] = counter;
      // blue[counter] = counter;

      // else
      //   for(int i=1;i<=10;i++)
      //    Serial.printf("%u  %u  %u\n", red[i],green[i],blue[i]);

      if (counter < 5)
      {
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        counter++;
        rcounter = 0;
        init_slot = false;
      }

      if (counter >= 5)
      {
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        delay(250);
        counter = 0;
        slot++;
        Serial.println(slot);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        delay(750);
        colorsensor_Data(slot);
        delay(250);
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(BLUE_LED, HIGH);
      }

      if (slot == 9)
      {
        if (init_slot == true)
          slot = 0;
        end = true;
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        for (int i = 0; i < 10; i++)
          Serial.printf("%u  %u  %u\n", red[i], green[i], blue[i]);
      }
    }
    else
    {

      /////////////////////////////////////////
      if (counter > 0)
      {
        counter--;
        rcounter = 6;
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
      }

      if (rcounter < 5)
      {
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        rcounter++;
        // counter++;
      }

      if (rcounter == 5)
      {
        // start = true;
        // end = false;
        rcounter = 0;
        counter = 0;
        --slot;
        Serial.println(slot);
        colorsensor_Data(slot);
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(BLUE_LED, HIGH);

        // Serial.println("loop3 ");
      }

      ////////////////////////////////////////
    }
    // Serial.println(counter);
  }

  // state = (state << 1) | digitalRead(CLK_PIN) | 0xe000;

  // if ((state == 0xf000) || (slot == 0))
  // {
  //   state = 0x0000;
  //   if  ((digitalRead(DATA_PIN)) && (init_count !=3))
  //   //if  (digitalRead(DATA_PIN))
  //   {
  //     Serial.println("loop1");
  //     if (counter > 0){
  //       counter--;
  //     Serial.println("loop2");
  //     }

  //     if (counter == 0){
  //       //start = true;
  //       //end = false;
  //       --slot;
  //       digitalWrite(BLUE_LED, HIGH);
  //       Serial.println("loop3 ");
  //     }

  //     if(slot < 0)
  //     slot = 0;

  //   }
  //   else
  //   {
  //     if (counter < 5)
  //     {
  //       //start = false;
  //       digitalWrite(BLUE_LED, LOW);
  //       digitalWrite(GREEN_LED, HIGH);
  //       counter++;
  //       init_slot = false;
  //       // red[counter] = counter;
  //       // green[counter] = counter;
  //       // blue[counter] = counter;
  //     }

  //     if((counter == 5) || (slot == 0)){
  //       counter = 0;
  //       Serial.println(slot);
  //       colorsensor_Data(slot);
  //       slot++;
  //       delay(500);
  //       digitalWrite(GREEN_LED, LOW);
  //       digitalWrite(BLUE_LED, HIGH);

  //     }

  //     if(slot == 10)
  //     {
  //       if(init_slot == true)
  //         slot = 0;
  //       end = true;
  //       digitalWrite(BLUE_LED, LOW);
  //       digitalWrite(GREEN_LED, LOW);
  //       for (int i = 0; i < 10; i++)
  //         Serial.printf("%u  %u  %u\n", red[i], green[i], blue[i]);
  //     }
  //   }
  //   //Serial.println(counter);
  // }
}

void blink_blueLED()
{

  unsigned long blue_currentMillis = millis();

  if (blue_currentMillis - blue_previousMillis >= 500)
  {
    // save the last time you blinked the LED
    blue_previousMillis = blue_currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (blue_ledState == LOW)
    {
      blue_ledState = HIGH;
    }
    else
    {
      blue_ledState = LOW;
      init_count++;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(BLUE_LED, blue_ledState);
  }
  if (init_count == 3)
    start = false;
}

void blink_greenLED()
{
  unsigned long green_currentMillis = millis();

  if (green_currentMillis - green_previousMillis >= 500)
  {
    // save the last time you blinked the LED
    green_previousMillis = green_currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (green_ledState == LOW)
    {
      green_ledState = HIGH;
    }
    else
    {
      green_ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(GREEN_LED, green_ledState);
  }
}

void blink_redLED()
{
  unsigned long red_currentMillis = millis();

  if (red_currentMillis - red_previousMillis >= 500)
  {
    // save the last time you blinked the LED
    red_previousMillis = red_currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (red_ledState == LOW)
    {
      red_ledState = HIGH;
    }
    else
    {
      red_ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(RED_LED, red_ledState);
  }
}

void send_ble_data()
{
  if (deviceConnected)
  {
    // Serial.println("Device Connected");

    if (send_data_flag)
    {
      Serial.println(fromSerial);
      int j = 0;
      readJsonData();
      Serial.println(jsonData);
      Serial.println(jsonData.length());
      std::string send_data = "";

      while (jsonData.length() >= j)
      {
        for (int i = 0; i < 20; i++, j++)
          send_data += jsonData[j];

        // Serial.println(j);
        // Serial.println(send_data.length());
        pTxCharacteristic->setValue(send_data); // Notify fromSerial.
        pTxCharacteristic->notify();
        send_data = "";
        delay(35);
      }
      // size_t len = jsonData.length();
      // send_data = (String)len;
      // pTxCharacteristic->setValue(send_data); // Notify fromSerial.
      // pTxCharacteristic->notify();
      send_data = "";
      jsonData = "";
      fromSerial = "";
      send_data_flag = false;
      init_slot = true;
      end = false;
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
    }
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    // Serial.println("Device Disconnected");
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}

void write_calib_to_EEPROM(int ee_address)
{

  EEPROM_writeAnything(ee_address, C[0][0]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 8, C[0][1]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 16, C[0][2]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 24, C[1][0]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 32, C[1][1]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 40, C[1][2]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 48, C[2][0]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 56, C[2][1]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 64, C[2][2]);
  EEPROM.commit();
  delay(100);
}

void write_conv_to_EEPROM(int ee_address)
{

  EEPROM_writeAnything(ee_address, T[0][0]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 8, T[0][1]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 16, T[0][2]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 24, T[1][0]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 32, T[1][1]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 40, T[1][2]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 48, T[2][0]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 56, T[2][1]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 64, T[2][2]);
  EEPROM.commit();
  delay(100);

  EEPROM_writeAnything(ee_address + 72, dutycycle);
  EEPROM.commit();
  delay(100);
}

void read_calib_from_EEPROM(int ee_address)
{
  EEPROM_readAnything(ee_address, C[0][0]);
  EEPROM_readAnything(ee_address + 8, C[0][1]);
  EEPROM_readAnything(ee_address + 16, C[0][2]);
  EEPROM_readAnything(ee_address + 24, C[1][0]);
  EEPROM_readAnything(ee_address + 32, C[1][1]);
  EEPROM_readAnything(ee_address + 40, C[1][2]);
  EEPROM_readAnything(ee_address + 48, C[2][0]);
  EEPROM_readAnything(ee_address + 56, C[2][1]);
  EEPROM_readAnything(ee_address + 64, C[2][2]);

 /* for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
      Serial.println(C[i][j]);
  }*/
}

void read_conv_from_EEPROM(int ee_address)
{
  EEPROM_readAnything(ee_address, T[0][0]);
  EEPROM_readAnything(ee_address + 8, T[0][1]);
  EEPROM_readAnything(ee_address + 16, T[0][2]);
  EEPROM_readAnything(ee_address + 24, T[1][0]);
  EEPROM_readAnything(ee_address + 32, T[1][1]);
  EEPROM_readAnything(ee_address + 40, T[1][2]);
  EEPROM_readAnything(ee_address + 48, T[2][0]);
  EEPROM_readAnything(ee_address + 56, T[2][1]);
  EEPROM_readAnything(ee_address + 64, T[2][2]);

  // for(int i=0; i<3 ;i++){
  //   for(int j=0;j<3;j++)
  //   Serial.println(T[i][j]);
  // }

  // EEPROM_readAnything(ee_address + 72, dutycycle);
  // Serial.println(dutycycle);
}

void parse_json_data(String in_data)
{

  int lengthString = in_data.length();
  char buf[lengthString + 1];
  in_data.toCharArray(buf, lengthString + 1);

  int bufLength = sizeof(buf);
  Serial.print(F("JSON buf response length: "));
  Serial.println(bufLength);

  StaticJsonBuffer<500> jsonStringBuffer;
  JsonObject &jsonString = jsonStringBuffer.parseObject(buf);

  if (!jsonString.success())
  {
    Serial.println(F("Failed to parse responseBody"));
  }

  C[0][0] = (jsonString["calibration_values"][0]);
  C[0][1] = (jsonString["calibration_values"][1]);
  C[0][2] = (jsonString["calibration_values"][2]);
  C[1][0] = (jsonString["calibration_values"][3]);
  C[1][1] = (jsonString["calibration_values"][4]);
  C[1][2] = (jsonString["calibration_values"][5]);
  C[2][0] = (jsonString["calibration_values"][6]);
  C[2][1] = (jsonString["calibration_values"][7]);
  C[2][2] = (jsonString["calibration_values"][8]);
  /*
    T[0][0] = (jsonString["conversion_values"][0]);
    T[0][1] = (jsonString["conversion_values"][1]);
    T[0][2] = (jsonString["conversion_values"][2]);
    T[1][0] = (jsonString["conversion_values"][3]);
    T[1][1] = (jsonString["conversion_values"][4]);
    T[1][2] = (jsonString["conversion_values"][5]);
    T[2][0] = (jsonString["conversion_values"][6]);
    T[2][1] = (jsonString["conversion_values"][7]);
    T[2][2] = (jsonString["conversion_values"][8]);

    dutycycle = (jsonString["conversion_values"][9]);
  */

  // store to EEPROM
  if (C[0][0] > 0)
  {
    write_calib_to_EEPROM(0);
    read_calib_from_EEPROM(0);
  }

  /*
    if(T[0][0] > 0){
    write_conv_to_EEPROM(72);
    //read_conv_from_EEPROM(72);
    }
  */
}

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
      deviceConnected = false;
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0)
      {
        for (int i = 0; i < rxValue.length(); i++)
          rx_data += rxValue[i];
      }
      Serial.println(rx_data);
      if (rx_data.indexOf("send") >= 0)
      {
        send_data_flag = true;
        rx_data = "";
      }
      else if (rx_data.indexOf("notok") >= 0)
      {
        send_data_flag = true;
        send_ble_data();
        rx_data = "";
      }
      else if (rx_data.indexOf("ok") >= 0)
      {
        rx_data = "";
      }
      else if (rx_data.indexOf("]}") >= 0)
      {
        parse_json_data(rx_data);
        rx_data = "";
      }
    }
};

void init_BLE()
{
  // Create the BLE Device
  BLEDevice::init("EarthFace");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY);
  // pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE);

  pServer->setCallbacks(new MyServerCallbacks());
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");
}

void setup()
{
  // put your setup code here, to run once:

  Serial.begin(115200);


  Serial.println("Begin");



  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(Battery_enable_pin, OUTPUT);
  pinMode(slot_PIN, INPUT);

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 0);

  /////BATTERY//////////


  check_battery_status();


  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
  delay(100);
  digitalWrite(BLUE_LED, LOW);

    init_BLE();
    delay(500);

  Wire.begin(26, 27); // sda= GPIO_21 /scl= GPIO_22
  colour.begin();

  delay(500);

  pinMode(CLK_PIN, INPUT);
  pinMode(DATA_PIN, INPUT);

  // assign to array
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM");
    delay(1000000);
  }

  read_calib_from_EEPROM(0);
  // read_conv_from_EEPROM(72);

  ledcWrite(ledChannel, dutycycle);

  ///////////SLOT SENSOR////////////////////////

    while (digitalRead(slot_PIN) == HIGH)
    {
      blink_redLED();
    }
  
    digitalWrite(RED_LED, LOW);
  
  
    while (start == true && end == false)
    {
      blink_blueLED();
      //Serial.println("EarthFace");
    }
  
    while (digitalRead(slot_PIN) == LOW);


  ///////////////////////////////////////////

  Serial.println("EarthFace");
  scan_data();
}



  int slot_analog = 0;

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 10000)
  {
    previousMillis = currentMillis;
     check_battery_status();
  }




//  if (start == true && end == false)
//  {
//    for (int j = 0; j < 10; j++) {
//      slot_analog = slot_analog + analogRead(slot_PIN);
//      delay(5);
//    }
//
//    if (slot_analog / 10 < 2900) {
//      blink_redLED();
//      
//    }
//    else if (slot_analog / 10 > 2900) {
//      blink_blueLED();
//    }
//    Serial.println(slot_analog/10);
//    slot_analog = 0;
//    
//  }
//
//
//
//  if (start == false && end == false) {
//    if(slot_analog == 0){
//       for (int j = 0; j < 10; j++) {
//        slot_analog = slot_analog + analogRead(slot_PIN);
//        delay(5);
//      }
//      Serial.println(slot_analog/10);
//    }
//    
//    if(slot_analog/10 >  2900){
//       slot_analog = 0;
//    }
//    else{
//      scan_data();
//       slot_analog = 1;
//    }
//  }
    

  

  if (start == false && end == true)
  {
    blink_greenLED();
    send_ble_data();
  }

    scan_data();


}