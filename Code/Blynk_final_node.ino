#include <SDS011.h>
#include "DHT.h"
#include <Wire.h>
#include <SPI.h>
// #include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <MQUnifiedsensor.h>
// #include <TinyGPSPlus.h>
// #include <SoftwareSerial.h>

/*Own*/
// #define BLYNK_TEMPLATE_ID "TMPLS7Iw-kCl"
// #define BLYNK_TEMPLATE_NAME "CCIoT"
// #define BLYNK_AUTH_TOKEN "3rDaGqYaAaZJOZ5WNluFMdAMcG-e3zsd"

/*Remote Labs SPCRC*/
#define BLYNK_TEMPLATE_ID "TMPLnMkAGIqH"
#define BLYNK_TEMPLATE_NAME "tictoc Device"
#define BLYNK_AUTH_TOKEN "5_q80He-rAnVZ7XL1e9VfSEGm-PNHS0F"

#define BLYNK_PRINT Serial
BlynkTimer timer;

// char ssid[] = "DESKTOP-JTEB0MU 7708";
// char pass[] = "12345678";

// char ssid[] = "motorola edge 20 pro_4872";
// char pass[] = "Password";

char ssid[] = "vivo_bk";
char pass[] = "wifibrahad";

// char ssid[] = "redmi";
// char pass[] = "redmi123";

// static const int RXPin = 3, TXPin = 1;
// static const uint32_t GPSBaud = 9600;

// // The TinyGPSPlus object
// TinyGPSPlus gps;
// // The serial connection to the GPS device
// SoftwareSerial ss(RXPin, TXPin);

#ifdef ESP32
HardwareSerial port(2);
#endif

#define DHTTYPE DHT22
#define DHTPIN 26

/************************Hardware Related Macros************************************/
#define Board ("ESP-32")  // Wemos ESP-32 or other board, whatever have ESP32 core.

//https://www.amazon.com/HiLetgo-ESP-WROOM-32-Development-Microcontroller-Integrated/dp/B0718T232Z (Although Amazon shows ESP-WROOM-32 ESP32 ESP-32S, the board is the ESP-WROOM-32D)
#define Pin (33)  //check the esp32-wroom-32d.jpg image on ESP32 folder

/***********************Software Related Macros************************************/
#define Type ("MQ-7")             //MQ7 or other MQ Sensor, if change this verify your a and b values.
#define Voltage_Resolution (3.3)  // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define ADC_Bit_Resolution (12)   // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define RatioMQ7CleanAir (27.5)   //RS / R0 = 27.5 ppm
/*****************************Globals***********************************************/
MQUnifiedsensor MQ7(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
// Adafruit_BMP280 bmp;  // I2C
DHT dht(DHTPIN, DHTTYPE);
SDS011 my_sds;

float temp_p10, temp_p25, T_BMP, P_BMP, t, h, CO_data, p10, p25;
// float lat,lng;

//Get the button value
// BLYNK_WRITE(V6) {
//   data_read_switch = param.asInt();
// }

float AQIcalc() {
  float p10aqi, p25aqi, COaqi, AQI;

  // p2.5 AQI
  if (p25 <= 30)
    p25aqi = p25 * 50/30;
  else if (p25 <= 60)
    p25aqi = 50 + ((p25 - 30) * 50/30);
  else if (p25 <= 90)
    p25aqi = 100 + ((p25 - 60) * 100/30);
  else if (p25 <= 120)
    p25aqi = 200 + ((p25 - 90) * 100/30);
  else if (p25 <= 250)
    p25aqi = 300 + ((p25 - 120) * 100/30);
  else if (p25 > 250)
    p25aqi = 400 + ((p25 - 250) * 100/30);
  else
    p25aqi = 0;

  // p10 AQI
  if (p10 <= 100)
    p10aqi = p10;
  else if (p10 <= 250)
    p10aqi = 100 + ((p10 - 100) * 100/150);
  else if (p10 <= 350)
    p10aqi = 200 + (p10 - 250);
  else if (p10 <= 430)
    p10aqi = 300 + ((p10 - 350) * 100/80);
  else if (p10 > 430)
    p10aqi = 400 + ((p10 - 430) * 100/80);
  else
    p10aqi = 0;

  // CO AQI
  if (CO_data <= 1)
    COaqi = CO_data * 50/1;
  else if (CO_data <= 2)
    COaqi = 50 + ((CO_data - 1) * 50/1);
  else if (CO_data <= 10)
    COaqi = 100 + ((CO_data - 2) * 100/8);
  else if (CO_data <= 17)
    COaqi = 200 + ((CO_data - 10) * 100/7);
  else if (CO_data <= 34)
    COaqi = 300 + ((CO_data - 17) * 100/17);
  else if (CO_data > 34)
    COaqi = 400 + ((CO_data - 34) * 100/17);
  else
    COaqi = 0;

  AQI = p25aqi;
  if (AQI < p10aqi)
    AQI = p10aqi;
  if (AQI < COaqi)
    AQI = COaqi;

  return AQI;
}

void myTimer() {
  // This function describes what will happen with each timer tick
  // if (data_read_switch) {
  // Blynk.beginGroup();
  // // Blynk.virtualWrite(V0, T_BMP);
  // Blynk.virtualWrite(V5, t);
  // Blynk.virtualWrite(V1, h);
  // Blynk.virtualWrite(V2, p25);
  // Blynk.virtualWrite(V3, p10);
  // // Blynk.virtualWrite(V4, P_BMP);
  // Blynk.virtualWrite(V7, CO_data);
  // Blynk.endGroup();
  // }

  // if (millis() > 5000 && gps.charsProcessed() < 10)
  // {
  //   Serial.println(F("No GPS detected: check wiring."));
  //   // while(true);
  // }

  float AQI = AQIcalc();

  Blynk.beginGroup();
    Blynk.virtualWrite(V0, p25);
    Blynk.virtualWrite(V1, p10);
    Blynk.virtualWrite(V2, CO_data);
    Blynk.virtualWrite(V3, h);
    Blynk.virtualWrite(V4, t);
    // Blynk.virtualWrite(V5, lng, lat);
    Blynk.virtualWrite(V6, AQI);
  Blynk.endGroup();

  Serial.println("P2.5: " + String(p25) + " ug/m³");
  Serial.println("P10: " + String(p10) + " ug/m³");

  // Serial.print(F("(From BMP) Temperature ="));
  // Serial.print(String(T_BMP) + " °C");
  // Serial.print(F(" Pressure = "));
  // Serial.println(String(P_BMP) + " hPa");

  // // Serial.print(F("Approx altitude = "));
  // // Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  // // Serial.println(" m");

  Serial.print(F("(From DHT) Temperature: "));
  Serial.print(t);
  Serial.print(" °C");
  Serial.print(F(" Humidity: "));
  Serial.print(h);
  Serial.println(" %");

  Serial.print(F("CO: "));
  Serial.print(CO_data);  // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  Serial.println(" PPM");
  
  Serial.print(F("AQI: "));
  Serial.println(AQI);
}

void setup() {
  Serial.begin(115200);
  
  // // GPS Sensor
  // ss.begin(GPSBaud);

  // BMP Sensor
  // while (!Serial) delay(100);  // wait for native usb
  // Serial.println(F("BMP280 test"));
  // unsigned status;
  // status = bmp.begin(0x76);
  // if (!status) {
  //   Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
  //                    "try a different address!"));
  //   Serial.print("SensorID was: 0x");
  //   Serial.println(bmp.sensorID(), 16);
  //   Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
  //   Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  //   Serial.print("        ID of 0x60 represents a BME 280.\n");
  //   Serial.print("        ID of 0x61 represents a BME 680.\n");
  //   while (1) delay(10);
  // }
  // bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  //                 Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  //                 Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  //                 Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  //                 Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // SDS Sensor
  Serial.println(F("SDS011 test"));
  my_sds.begin(&port);

  // DHT Sensor
  Serial.println(F("DHT22 test"));
  dht.begin();

  //MQ7 Sensor
  //Set math model to calculate the PPM concentration and the value of constants
  MQ7.setRegressionMethod(1);  //_PPM =  a*ratio^b
  MQ7.setA(99.042);
  MQ7.setB(-1.518);  // Configure the equation to calculate CO concentration value

  /*
    Exponential regression:
  GAS     | a      | b
  H2      | 69.014  | -1.374
  LPG     | 700000000 | -7.703
  CH4     | 60000000000000 | -10.54
  CO      | 99.042 | -1.518
  Alcohol | 40000000000000000 | -12.35
  */

  /*****************************  MQ Init ********************************************/
  //Remarks: Configure the pin of arduino as input.
  /************************************************************************************/
  MQ7.init();

  /* 
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ7.setRL(10);
  */
  /*****************************  MQ CAlibration ********************************************/
  // Explanation:
  // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
  // and on clean air (Calibration conditions), setting up R0 value.
  // We recomend executing this routine only on setup in laboratory conditions.
  // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ7.update();  // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ7.calibrate(RatioMQ7CleanAir);
    Serial.print(".");
  }
  MQ7.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0)) { Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); }
  if (calcR0 == 0) { Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); }

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(2000L, myTimer);
}

// void displayInfo()
// {
//   Serial.print(F("Location: ")); 
//   if (gps.location.isValid())
//   {
//     lat = gps.location.lat();
//     Serial.print(lat, 6);
//     Serial.print(F(","));
//     lng = gps.location.lng();
//     Serial.print(lng, 6);
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F("  Date/Time: "));
//   if (gps.date.isValid())
//   {
//     Serial.print(gps.date.month());
//     Serial.print(F("/"));
//     Serial.print(gps.date.day());
//     Serial.print(F("/"));
//     Serial.print(gps.date.year());
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F(" "));
//   if (gps.time.isValid())
//   {
//     if (gps.time.hour() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.hour());
//     Serial.print(F(":"));
//     if (gps.time.minute() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.minute());
//     Serial.print(F(":"));
//     if (gps.time.second() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.second());
//     Serial.print(F("."));
//     if (gps.time.centisecond() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.centisecond());
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.println();
// }

void loop() {
  // // BMP Sensor
  // T_BMP = bmp.readTemperature();
  // P_BMP = bmp.readPressure() / 100;  //standardizing to hPa

  // // GPS Sensor
  // if (millis() > 5000 && gps.charsProcessed() < 10)
  // {
  //   Serial.println(F("No GPS detected: check wiring."));
  //   // while(true);
  // }

  // while (ss.available() > 0)
  //   if (gps.encode(ss.read()))
  //     displayInfo();

  // SDS Sensor
  my_sds.read(&temp_p25, &temp_p10);
  p25 = (1.7635 * temp_p25) + 2.8795;
  p10 = (2.1558 * temp_p10) + 16.618;

  // DHT Sensor
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
  // Check if any reads failed and exit early (to try again).
  // if (isnan(h)) {
  //   Serial.println(F("Failed to read from DHT sensor!"));
  // }

  // MQ7 Sensor
  MQ7.update();  // Update data, the arduino will read the voltage from the analog pin
  //MQ7.serialDebug(); // Will print the table on the serial port
  CO_data = MQ7.readSensor();

  Blynk.run();
  timer.run();

  //  delay(2000);
}