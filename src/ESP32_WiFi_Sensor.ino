// Web
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>
#include <.env>

#include "time.h"

// Sensors
#include <SparkFunCCS811.h> //Click here to get the library: http://librarymanager/All#SparkFun_CCS811
#include <Adafruit_BMP280.h>
#include <SparkFun_Si7021_Breakout_Library.h>

// Other
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//*************************************************************************

// ---- WLAN ----
// included by .env file
// const char *AP_SSID = "?";
// const char *AP_PWD = "?";

// ---- Sensor ----
#define CCS811_ADDR 0x5A //Alternate I2C Address
#define PRESSURE_ARRAY_SIZE 40

//*************************************************************************

//*************************************************************************

// ---- Internet ----
WiFiMulti wifiMulti;

WiFiUDP UDP; // Create an instance of the WiFiUDP class to send and receive

IPAddress timeServerIP; // time.nist.gov NTP server address
const char *NTPServerName = "time.nist.gov";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte NTPBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

// ---- Sensor ----
uint8_t testvariable = 0;

float pressure = 0;
uint16_t pressure_high = 0;
uint16_t pressure_low = 0;
float pressure_back = 0;
uint16_t humidity = 0;
float temperature = 0;
uint16_t carbondioxide = 0;
uint16_t organic = 0;

float pressurearray[PRESSURE_ARRAY_SIZE + 1];
float pressure_mw = 0;
uint8_t counter = 0;

CCS811 myCCS811(CCS811_ADDR);

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

//Create Instance of SI7021 temp and humidity sensor
Weather Si7021;

// ---- NTP ----
unsigned long intervalNTP = 60000; // Request NTP time every minute
unsigned long prevNTP = 0;
unsigned long lastNTPResponse = millis();
uint32_t timeUNIX = 0;

unsigned long prevActualTime = 0;

// ---- general ----
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//*************************************************************************
void startWiFi()
{                                   // Try to connect to some given access points. Then wait for a connection
  wifiMulti.addAP(AP_SSID, AP_PWD); // add Wi-Fi networks you want to connect to
  // wifiMulti.addAP("ssid_from_AP_2", "your_password_for_AP_2");
  // wifiMulti.addAP("ssid_from_AP_3", "your_password_for_AP_3");

  Serial.println("Connecing...");
  printDisplay("Connecing...", "", INADDR_NONE, "");

  while (wifiMulti.run() != WL_CONNECTED)
  { // Wait for the Wi-Fi to connect
    delay(250);
    Serial.print('.');
  }
  Serial.println("\r\n");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID()); // Tell us what network we're connected to
  Serial.print("IP address:\t");
  Serial.print(WiFi.localIP()); // Send the IP address of the ESP8266 to the computer
  Serial.println("\r\n");

  printDisplay("Connected to", String(WiFi.SSID()), IPAddress(WiFi.localIP()), "");
}

//*************************************************************************

void setup()
{
  Serial.begin(115200);

  Wire.begin(); // Inialize I2C Hardware

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) // Address 0x3C for 128x64
  {
    Serial.println(F("SSD1306 allocation failed"));
  }
  else
  {
    // Clear the buffer
    display.clearDisplay();
    display.setTextColor(WHITE);
  }

  Si7021.begin(); // Initialize the I2C humiditysensors and ping them

  //This begins the CCS811 sensor and prints error status of .beginWithStatus()
  CCS811Core::CCS811_Status_e returnCode = myCCS811.beginWithStatus();
  Serial.print("CCS811 begin exited with: ");
  Serial.println(myCCS811.statusString(returnCode));

  if (bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID) == false) // ToDo
  {
    Serial.println(F("[ BMP280 returned with an error ]"));
    // softReset();
  }

  if (myCCS811.begin() == false)
  {
    Serial.print("[ CCS811 error returned with an error ]");
    // softReset();
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X16,    /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  for (uint8_t n = 0; n < PRESSURE_ARRAY_SIZE; n++)
  {
    pressurearray[n] = 0; // clear the array for average calculation
  }

  delay(1000);
  startWiFi();

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

//*************************************************************************
void postDataToServer()
{
  String message = String("POST - Status: ");
  String message_error = String("POST - Error: ");
  Serial.println("Posting JSON data to server...");
  // Block until we are able to connect to the WiFi access point
  if (wifiMulti.run() == WL_CONNECTED)
  {

    HTTPClient http;

    // http.begin("http://192.168.0.4:3000/sensor");
    // http.begin("http://192.168.188.205:3000/sensor");
    http.begin("http://1368-87-139-68-34.ngrok.io/sensor");
    // http.begin("https://moisturizeme.herokuapp.com/sensor"); // deployed master on heroku
    // http.begin("http://" + String(IPAddress(WiFi.localIP())) + ":3000/sensor");
    // IPAddress(WiFi.localIP())

    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<200> doc;
    // Add values in the document
    //
    doc["sensor_id"] = "1";
    doc["time"] = 12345678;
    doc["pressure"] = pressure;
    doc["temperature"] = temperature;
    doc["humidity"] = humidity;
    doc["carbondioxide"] = carbondioxide;
    doc["organic"] = organic;

    // Add an array.
    //
    JsonArray data = doc.createNestedArray("sensorData");
    data.add(42.4242);
    data.add(testvariable++);

    String requestBody;
    serializeJson(doc, requestBody);

    int httpResponseCode = http.POST(requestBody);

    if (httpResponseCode > 0)
    {

      String response = http.getString();

      Serial.println(httpResponseCode);
      Serial.println(response);

      printDisplay("Connected to", String(WiFi.SSID()), IPAddress(WiFi.localIP()), String(message + httpResponseCode));
    }
    else
    {

      printDisplay("Connected to", String(WiFi.SSID()), IPAddress(WiFi.localIP()), String(message_error + httpResponseCode));
      // Serial.printf("Error occurred while sending HTTP POST: %s\n", httpClient.errorToString(statusCode).c_str());
    }
    delay(500);
    printDisplay("Connected to", String(WiFi.SSID()), IPAddress(WiFi.localIP()), String("waiting..."));
  }
}

//*************************************************************************
void ntp_request()
{
  unsigned long currentMillis = millis();

  if (currentMillis - prevNTP > intervalNTP)
  { // If a minute has passed since last NTP request
    prevNTP = currentMillis;
    Serial.println("\r\nSending NTP request ...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  }
}

//*************************************************************************
void calculatePessureAverage()
{
  uint8_t n = 0;

  // **** average calculation
  counter++;
  if (counter > PRESSURE_ARRAY_SIZE)
    counter = PRESSURE_ARRAY_SIZE;

  pressurearray[n++] = pressure;
  if (n >= PRESSURE_ARRAY_SIZE)
    n = 0;

  pressure_mw = 0;
  for (uint8_t i = 0; i <= PRESSURE_ARRAY_SIZE; i++)
  {
    pressure_mw += pressurearray[i];
  }

  pressure_mw /= counter; // calculate the average
  // **** end of average calculation
}

//*************************************************************************
void perform_measurements(void)
{
  temperature = Si7021.getTemp();
  humidity = Si7021.getRH();

  if (myCCS811.dataAvailable())
  {
    myCCS811.setEnvironmentalData((float)humidity, temperature);

    //Calling this function updates the global tVOC and eCO2 variables
    myCCS811.readAlgorithmResults();

    carbondioxide = myCCS811.getCO2();
    organic = myCCS811.getTVOC();

    //printUART();
  }

  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  pressure = pressure_event.pressure;
  pressure_high = (uint32_t)pressure;                // from a 1234.56 it contains 1234
  pressure_low = ((pressure - pressure_high) * 100); // from a 1234.56 it contains 56

  calculatePessureAverage();
}

//*************************************************************************
void printUART()
{
  Serial.print(F("Press: "));
  Serial.print((pressure), 2);
  Serial.print(F(" hPa"));
  Serial.print(F(" Temp: "));
  Serial.print(temperature, 1);
  Serial.print(F("°C"));
  Serial.print(F(" RH: "));
  Serial.print(humidity);
  Serial.print(F("%"));
  Serial.print(F(" CO2: "));
  Serial.print(carbondioxide);
  Serial.print(F(" ppm"));
  Serial.print(F(" tVOC: "));
  Serial.print(organic);
  Serial.print(F(" ppb"));

  Serial.println();
}

//*************************************************************************
void printDisplay(char statusLine1[], String statusLine2, IPAddress statusLine3, String statusLine4)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0 * 8);
  display.print(statusLine1);
  display.setCursor(0, 1 * 8);
  display.print(statusLine2);

  display.setCursor(0, 3 * 8);
  display.print("IP-Address:");
  display.setCursor(0, 4 * 8);
  display.print(statusLine3);

  display.setCursor(0, 6 * 8);
  display.print(statusLine4);

  display.display();
}

//*************************************************************************
void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  // Serial.print("Day of week: ");
  // Serial.println(&timeinfo, "%A");
  // Serial.print("Month: ");
  // Serial.println(&timeinfo, "%B");
  // Serial.print("Day of Month: ");
  // Serial.println(&timeinfo, "%d");
  // Serial.print("Year: ");
  // Serial.println(&timeinfo, "%Y");
  // Serial.print("Hour: ");
  // Serial.println(&timeinfo, "%H");
  // Serial.print("Minute: ");
  // Serial.println(&timeinfo, "%M");
  // Serial.print("Second: ");
  // Serial.println(&timeinfo, "%S");

  // Serial.println("Time variables");
  // char timeHour[3];
  // strftime(timeHour,3, "%H", &timeinfo);
  // Serial.println(timeHour);
  // char timeWeekDay[10];
  // strftime(timeWeekDay,10, "%A", &timeinfo);
  // Serial.println(timeWeekDay);
  // Serial.println();
}

//*************************************************************************

void loop()
{
  delay(2000);
  ntp_request();
  printLocalTime();
  perform_measurements();
  postDataToServer();
}