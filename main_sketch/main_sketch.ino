#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <ESP_I2S.h>
#include <FS.h>
#include <Wire.h>
#include <RTClib.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "driver/gpio.h"

// --- I2S MIC GLOBAL DEFAULTS and VARIABLES ---
const int I2S_BCK_PIN = 4;
const int I2S_WS_PIN = 5;
const int I2S_SD_PIN = 6;
const unsigned int SAMPLE_RATE = 32000;
const int WAVE_HEADER_SIZE = 44;
I2SClass i2s;

// --- Recording constraints ---
const unsigned long recordingTimeLimit = 1200000; //20 minute files is sweet spot 
bool isRecording = false;
unsigned long recordingStartTime = 0;

// --- RTC MODULE GLOBAL DEFAULTS and VARIABLES ---
const int I2C_SDA_PIN = 8;
const int I2C_SCL_PIN = 9;
#define BME280_I2C_ADDR 0x76

// --- DAILY WAKEUP WINDOWS (in 24-hour format) --
const int START_1_HR = 4;  // Window 1
const int START_1_MIN = 30;
const int STOP_1_HR = 11;    // Window 1 End
const int STOP_1_MIN = 0;
const int START_2_HR = 11;  // Window 2 Start
const int START_2_MIN = 10;
const int STOP_2_HR = 20;   // Window 2 End
const int STOP_2_MIN = 30;

// --- SD CARD MODULE GLOBAL DEFAULTS and VARIABLES ---
const int SD_CS_PIN = 10;
const int SD_MOSI_PIN = 11;
const int SD_MISO_PIN = 13;
const int SD_CLK_PIN = 12; //also know as SCK pin 
// -- for blinking yellow light for disc error when recording --
unsigned long startDiscError = 0;
const unsigned long intDiscError = 10000;

// --- GPS GLOBAL DEFAULTS and Variables ---
const int RXD1 = 16;  // Connect to the TX pin of the GPS!!!! reversed!!
const int TXD1 = 15;  // Connect to the RX pin of the GPS!!!! reversed!!
const double DEFAULT_LAT = 0.0;
const double DEFAULT_LNG =  0.0;
double globalLat = DEFAULT_LAT;
double globalLng = DEFAULT_LNG;
bool hasValidGpsFix = false;
const unsigned long GPS_SETUP_TIMEOUT_MS =  900000;   // 15 minutes max wait in setup (900K ms)
const int MOSFET_GATE_PIN  = 1;
//TIMEZONE
RTC_DATA_ATTR int savedTimezoneOffsetHours = 0; 
RTC_DATA_ATTR bool timezoneKnown = false;

// --- LED PIN DEFINITIONS ---
const int LED_REC = 14; // blue led for recording or listening
const int LED_DISC = 17; // yellow led for SD disc status full or not ready
// -- used in startup disc check --
int ledStateRecording = LOW; 
// -- used in recording long blink when sampling--
const unsigned long LED_BLINK_PERIOD_MS = 3000;  // time between flashes
const unsigned long LED_PULSE_MS = 40;           // how long each flash lasts
unsigned long prevMillisBlnk = 0;   
unsigned long lastRecBlinkTime = 0;
unsigned long lastGPSBlinkTime = 0;
bool gpsLedState = false;
bool ledStateRec = false;

// --- BLUETOOTH BLE ---
const char* const SERVICE_UUID = "fdcff45e-438b-4a62-acf6-dbd852aae4b1";
const char* const CHARACTERISTIC_UUID = "cf28c230-d88e-4e6e-8a2e-0efc4d8ec072";
BLECharacteristic *pCharacteristic = nullptr;
BLEAdvertising *pAdvertising = nullptr;
const unsigned long BLE_REFRESH_INTERVAL = 30000; // 30 seconds
unsigned long lastBleRefresh = 0;

// --- BAT POWER MONITORING ---
float batteryLevel;
const int BAT_PIN = 2;
const float DIVIDER_RATIO = 1.365;  // 4.05V (meter) / 3.000V (ADC reading)

// --- EEPROM CONSTANTS for storing device name---
const int MAX_STRING_LENGTH = 16; 
const int eepromAddress = 0;
const char* DEFAULT_DEVICE_NAME = "name_tbd_aww"; 

// --- FILE LOGGING ---
String currentFileName; // Determined dynamically on boot/rotation

// -- INSTANTIATE GLOBAL FILE NAME AND RTC --
File file;
File file1;
char filename[92];
RTC_DS3231 rtc;
Adafruit_BME280 bme;
bool bmeFound = false;
TinyGPSPlus gps;
// Forward Declarations
String readStringFromEEPROM(int address);
void writeStringToEEPROM(int address, String data);
void logMessage(const String &message);
void startRecording();
void stopRecording();
void appendAudioToSD();
void writeWavHeader(File &file, int sampleRate, int bitsPerSample, int channels, int dataSize);
void syncSystemClockFromRTC();

void setLocalTimezone(float longitude, int year, int month, int day) {
    char tzString[32];
    if (longitude > 0) {
        // Philippines: UTC+8. 
        // Note: POSIX TZ format is inverted for positive offsets! UTC+8 is written as -8.
        strcpy(tzString, "PHT-8");
        savedTimezoneOffsetHours = 8;
    } else {
        // Eastern Time Zone (US). Handles EDT/EST if date logic is added.
        // For simplicity, a basic static check or standard POSIX string can be used:
        // "EST5EDT,M3.2.0,M11.1.0" handles EST (UTC-5) and EDT (UTC-4) automatically via standard C lib!
        strcpy(tzString, "EST5EDT,M3.2.0,M11.1.0");
        savedTimezoneOffsetHours = -4; // Base offset
    }
    setenv("TZ", tzString, 1);
    tzset();
    timezoneKnown = true;
    Serial.printf("Timezone configured to: %s\n", tzString);
    logMessage(String("Timezone configured to: ") + tzString);
}

void syncSystemTimeWithGPS() {
    struct tm t;
    t.tm_year = gps.date.year() - 1900;
    t.tm_mon = gps.date.month() - 1;
    t.tm_mday = gps.date.day();
    t.tm_hour = gps.time.hour();
    t.tm_min = gps.time.minute();
    t.tm_sec = gps.time.second();
    t.tm_isdst = 0; // GPS is pure UTC, no DST

    // Universal way to convert UTC struct tm to Unix epoch without timegm()
    // We temporarily clear TZ to "UTC", run mktime, then let tzset() restore it later.
    char *old_tz = getenv("TZ");
    std::string saved_tz = old_tz ? old_tz : "";
    
    setenv("TZ", "UTC0", 1);
    tzset();
    time_t utc_epoch = mktime(&t);

    // Restore your original local timezone configuration
    if (!saved_tz.empty()) {
        setenv("TZ", saved_tz.c_str(), 1);
    } else {
        unsetenv("TZ");
    }
    tzset();
    
    // Apply the pure UTC epoch to the internal ESP32 system time
    struct timeval tv = { .tv_sec = utc_epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);

    // Convert that exact same epoch timestamp into LOCAL time components
    // This utilizes your restored local "TZ" environment variable automatically!
    struct tm *local_tm = localtime(&utc_epoch);

    // Update the DS3231 RTC with the adjusted LOCAL time
    rtc.adjust(DateTime(
        local_tm->tm_year + 1900, 
        local_tm->tm_mon + 1, 
        local_tm->tm_mday, 
        local_tm->tm_hour, 
        local_tm->tm_min, 
        local_tm->tm_sec
    ));
    
    Serial.println("System clock (UTC) and RTC (Local) successfully synced!");
    logMessage("System clock (UTC) and RTC (Local) successfully synced! ");
}

void printLocalTime() {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        Serial.println("Failed to obtain time");
        logMessage("Failed to obtain time ");
        return;
    }
    char timeBuffer[64];
    strftime(timeBuffer, sizeof(timeBuffer), "%A, %B %d %Y %H:%M:%S", &timeinfo);
    Serial.println(&timeinfo, "Local Time: %A, %B %d %Y %H:%M:%S");
    logMessage("Local Time: " + String(timeBuffer));
}

void logMessage(const String &message) {
  // Open file in Append mode. If it doesn't exist, it creates it automatically.
  file1 = SD.open(currentFileName, FILE_APPEND);
  if (file1) {
    file1.println(message);
    // Explicitly closing updates the file allocation table (FAT) size immediately
    file1.close(); 
  } else {
    Serial.println("CRITICAL: Failed to open file for appending debug data!");
  }
}

uint64_t logStorageStatus() {
  uint64_t totalBytes = SD.totalBytes();
  uint64_t usedBytes = SD.usedBytes();
  uint64_t freeBytes = totalBytes - usedBytes;
  float freeMB = freeBytes / (1024.0 * 1024.0);
  float totalMB = totalBytes / (1024.0 * 1024.0);
  String msg = "SD Storage: " + String(freeMB, 1) + " MB free / " + String(totalMB, 1) + " MB total";
  Serial.println(msg);
  logMessage(msg);
  const float MB_PER_RECORDING =
      (SAMPLE_RATE * 2.0f * (recordingTimeLimit / 1000.0f)) / (1024.0f * 1024.0f);
  if (freeMB < MB_PER_RECORDING * 2) {
    String warn = "WARNING: Low SD space - less than 2 recordings remaining!";
    Serial.println(warn);
    logMessage(warn);
  }
  return freeBytes;
}

int writeFailures = 0;
RTC_DATA_ATTR char lastError[64] = ""; 
bool mountSD() {
  SD.end();
  SPI.end();
  delay(200);
  SPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  return SD.begin(SD_CS_PIN);
}

void syncSystemClockFromRTC() {
  DateTime t = rtc.now();                 // DS3231 holds local time
  setenv("TZ", "UTC0", 1);                // no conversion: system "local" = RTC local
  tzset();
  struct timeval tv = { .tv_sec = (time_t)t.unixtime(), .tv_usec = 0 };
  settimeofday(&tv, NULL);
}

void i2cBusRecovery() {
  Wire.end();
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SCL_PIN, HIGH);
  for (int i = 0; i < 9 && digitalRead(I2C_SDA_PIN) == LOW; i++) {  // clock out a stuck slave
    digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(10);
  }
  pinMode(I2C_SDA_PIN, OUTPUT);                                      // generate a STOP condition
  digitalWrite(I2C_SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(I2C_SDA_PIN, HIGH); delayMicroseconds(10);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
}

bool initRTC() {
  for (int attempt = 1; attempt <= 5; attempt++) {
    i2cBusRecovery();                    // includes Wire.begin()
    delay(50);
    if (rtc.begin()) return true;
    Serial.printf("RTC init attempt %d failed\n", attempt);
    delay(300 * attempt);
  }
  return false;
}

// Returns true only for a plausible time, so a failed read can't pass as a real one
bool readRTC(DateTime &out) {
  for (int i = 0; i < 3; i++) {
    DateTime t = rtc.now();
    if (t.year() >= 2020 && t.year() <= 2099 && t.month() >= 1 && t.month() <= 12 &&
        t.hour() < 24 && t.minute() < 60) {
      out = t;
      return true;
    }
    delay(10);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  pinMode(BAT_PIN, INPUT);
  gpio_pullup_dis((gpio_num_t)BAT_PIN);
  gpio_pulldown_dis((gpio_num_t)BAT_PIN);
  pinMode(LED_REC, OUTPUT);
  pinMode(LED_DISC, OUTPUT);
  digitalWrite(LED_REC, LOW);
  digitalWrite(LED_DISC, LOW);
  pinMode(MOSFET_GATE_PIN, OUTPUT);
  digitalWrite(MOSFET_GATE_PIN, LOW);
  setCpuFrequencyMhz(80);
  
  if (!initRTC()) {
    strlcpy(lastError, "RTC not found in setup", sizeof(lastError));
    for (int i = 0; i < 30; i++) {
      digitalWrite(LED_DISC, i % 2);
      delay(500);
    }
    esp_restart();
  }

  // Initialize BME280 on default Wire
  if (bme.begin(BME280_I2C_ADDR, &Wire)) {
     bmeFound = true;
  } else {
    Serial.println("BME280: NOT found! (Check wiring or try 0x76)");
  }

  //initialize SD keep trying
  bool sdOk = false;
  for (int attempt = 1; attempt <= 5 && !sdOk; attempt++) {
    sdOk = mountSD();
    if (!sdOk) {
      Serial.printf("SD mount attempt %d failed\n", attempt);
      delay(500);
    }
  }
  if (!sdOk) {
    Serial.println("SD Card not Initialized! Blinking, then restarting.");
    strlcpy(lastError, "SD.begin failed in setup", sizeof(lastError));
    unsigned long t = millis();
    while (millis() - t < 30000) {      // blink for 30 s so you can still see the fault
      digitalWrite(LED_DISC, HIGH);
      delay(500);
      digitalWrite(LED_DISC, LOW);
      delay(500);
    }
    esp_restart();
  }

  DateTime now;
  bool haveTime = readRTC(now);
  if (!haveTime && rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    haveTime = readRTC(now);
  }
  if (!haveTime) {
    strlcpy(lastError, "RTC time invalid in setup", sizeof(lastError));
    esp_restart();
  }
  syncSystemClockFromRTC();

  // Initialize Serial1
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  unsigned long setupStart = millis();

  // Convert everything to minutes since midnight for easy comparison
  int currentMinutes = (now.hour() * 60) + now.minute();
  int start1 = (START_1_HR * 60) + START_1_MIN;
  int stop1  = (STOP_1_HR * 60) + STOP_1_MIN;
  int start2 = (START_2_HR * 60) + START_2_MIN;
  int stop2  = (STOP_2_HR * 60) + STOP_2_MIN;

  // Check if we are currently INSIDE either of the two active windows
  bool inWindow1 = (currentMinutes >= start1 && currentMinutes < stop1);
  bool inWindow2 = (currentMinutes >= start2 && currentMinutes < stop2);

  if (!EEPROM.begin(MAX_STRING_LENGTH)) {
    logMessage("Failed to initialize EEPROM ");
    Serial.println("Failed to initialize EEPROM");
    return;
  }

  //one time code to enter the unit id into the EEPROM as a config
  //char charBuffer[MAX_STRING_LENGTH];
  //String myString = "aw-chipbot-03.3";  
  //writeStringToEEPROM(eepromAddress, myString);
  //Serial.println("String saved successfully.");
  //String retrievedString = readStringFromEEPROM(eepromAddress);
  //Serial.print("Retrieved String: ");
  //Serial.println(retrievedString);

  if (inWindow1 || inWindow2) {
    // We are supposed to be awake! Proceed to void loop()
    // Log file work establish filename format with minutes from RTC for initial boot
    char bootLogName[50];
    char deviceName[16];
    strlcpy(deviceName, readStringFromEEPROM(eepromAddress).c_str(), sizeof(deviceName));
    snprintf(bootLogName, sizeof(bootLogName), "/%s_%04d-%02d-%02d-%02d%02d%02d_log.txt", 
            deviceName, now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    currentFileName = String(bootLogName);
    
    batteryLevel = voltageToPercent(readBatteryVoltage());
    Serial.println("Battery Level...");
    Serial.println(String(batteryLevel));
    digitalWrite(MOSFET_GATE_PIN, LOW);
    Serial.println("In active window get GPS coordinates wait max 20 minutes.");
    logMessage("In active window get GPS coordinates wait max 20 minutes.");
    // GET initial GPS coordinates
    while (!hasValidGpsFix && (millis() - setupStart < GPS_SETUP_TIMEOUT_MS)) {
      unsigned long currentMillis = millis();
      if (currentMillis - lastGPSBlinkTime >= 300) {
        lastGPSBlinkTime = currentMillis;
        gpsLedState = !gpsLedState;
        digitalWrite(LED_REC, gpsLedState);
      }
      while (Serial1.available() > 0) {
        char c = Serial1.read();
        if (gps.encode(c)) {
          if (gps.location.isValid()) {
            globalLat = gps.location.lat();
            globalLng = gps.location.lng();
            hasValidGpsFix = true;
            Serial.print("Initial Coordinates: ");
            logMessage("Initial Coordinates: ");
            String latStr = String(globalLat, 6);
            String longStr = String(globalLng, 6);
            Serial.print(latStr);
            logMessage(latStr);
            Serial.println(longStr);
            logMessage(longStr);
            struct tm timeinfo;
            if (getLocalTime(&timeinfo) && timezoneKnown) {
                printLocalTime();
            } else {
                // 1. Read from your GY-NEO6MV2 module here until gps.location.isValid() and gps.time.isValid() are true
                // 2. Once valid:
                 float lon = gps.location.lng();
                 setLocalTimezone(lon, gps.date.year(), gps.date.month(), gps.date.day());
                 syncSystemTimeWithGPS();
                 printLocalTime();
            }
            break; 
          }
        }
      }
      delay(1);
    }
    // If the loop finished but we never got a valid fix, use defaults
    if (!hasValidGpsFix) {
      globalLat = DEFAULT_LAT;
      globalLng = DEFAULT_LNG;
      Serial.println("Could not get a satellite lock in time.");
      logMessage("Could not get a satellite lock in time. ");
    }
    digitalWrite(MOSFET_GATE_PIN, HIGH);
    syncSystemClockFromRTC();

    // Synchronize our timers right as setup finishes
    char latStr[16];
    char lngStr[16];
    dtostrf(globalLat, 1, 6, latStr);
    dtostrf(globalLng, 1, 6, lngStr);
    String coordinatesString = String(latStr) + "," + lngStr;
    String retrievedString = readStringFromEEPROM(eepromAddress);
    if (retrievedString.length() == 0) {
      retrievedString = "esp32_aw_xx";
    }
    BLEDevice::init(retrievedString);
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
    String initCharacteristic = "Coordinates: " + coordinatesString + ", Battery: " + String(batteryLevel);
    pCharacteristic->setValue(initCharacteristic);
    pService->start();
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    // MIN & MAX advertising intervals. 
    // 0x0640 * 0.625 ms = 1000 ms (1 second)
    // Higher interval = lower power, but takes longer for your Android phone to discover.
    pAdvertising->setMinInterval(0x0640); 
    pAdvertising->setMaxInterval(0x0640);
    BLEDevice::startAdvertising();
    Serial.println("Characteristic defined! Now advertising...");
    logMessage("Characteristic defined! Now advertising...");
    // This allows the ESP32-S3 to automatically enter light sleep 
    // between BLE advertising intervals.
    esp_sleep_enable_timer_wakeup(1000000); // Optional: dynamic fallback
    #if CONFIG_PM_ENABLE
      esp_pm_config_esp32s3_t pm_config = {
          .max_freq_mhz = 240,
          .min_freq_mhz = 40,
          .light_sleep_enable = true
      };
      esp_pm_configure(&pm_config);
    #endif

    i2s.setPins(I2S_BCK_PIN, I2S_WS_PIN, -1, I2S_SD_PIN);   // bclk, ws, dout, din
    if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT,
                  I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
      Serial.println("I2S init failed!");
      logMessage("I2S init failed! ");
      while (1);
    }

    Serial.println("System is Ready. Beginning continuous loop recording.");
    logMessage("System is Ready. Beginning continuous loop recording.");

  } else {
      digitalWrite(MOSFET_GATE_PIN, HIGH);
      int minutesToSleep = 0;

      if (currentMinutes < start1) {
        // sleep until Window 1
        minutesToSleep = start1 - currentMinutes;
      } else if (currentMinutes < start2) {
        // We are between Window 1 and Window 2, sleep until Window 2
        minutesToSleep = start2 - currentMinutes;
      } else {
        // It's late night, sleep until Window 1 of the NEXT day
        minutesToSleep = (1440 - currentMinutes) + start1;
      }

      // Safeguard seconds adjustment: subtract current seconds so we wake up exactly on the minute mark
      long secondsToSleep = (minutesToSleep * 60) - now.second();
      if (secondsToSleep <= 0) secondsToSleep = 1; // Prevent negative/zero sleep
      String timestamp = now.timestamp();
      Serial.println(timestamp);
      logMessage(timestamp);
      Serial.printf("Outside active windows. Sleeping for %ld seconds...", secondsToSleep);
      logMessage("Outside active windows. Sleeping for %ld seconds..." + String(secondsToSleep));
      Serial.flush();

      // Configure deep sleep timer (expects microseconds)
      esp_sleep_enable_timer_wakeup((uint64_t)secondsToSleep * 1000000ULL);
      esp_deep_sleep_start();
  }
}

void loop() {  
  unsigned long currentMillis = millis();
  if (currentMillis - lastBleRefresh >= BLE_REFRESH_INTERVAL) {
    lastBleRefresh = currentMillis;

    // Refresh advertising in case the stack silently stopped
    if (pAdvertising != nullptr) {
      pAdvertising->start();
    }
    // Refresh characteristic payload with current readings
    if (pCharacteristic != nullptr) {
      float temp = 0.0, humidity = 0.0;
      if (bmeFound) {
        temp     = bme.readTemperature();
        humidity = bme.readHumidity();
      }
      char latStr[16], lngStr[16];
      dtostrf(globalLat, 1, 6, latStr);
      dtostrf(globalLng, 1, 6, lngStr);
      unsigned long elapsedMin = isRecording ? (currentMillis - recordingStartTime) / 60000 : 0;
      uint64_t totalBytes = SD.totalBytes();
      uint64_t usedBytes = SD.usedBytes();
      float freeMB = (totalBytes - usedBytes) / (1024.0 * 1024.0);
      String updatedPayload = "Coords:" + String(latStr) + "," + String(lngStr) +
                              " | Bat:" + String(batteryLevel, 1) + "%";
      if (bmeFound) {
        updatedPayload += " | Temp:" + String(temp, 1) + "C" +
                          " | Hum:" + String(humidity, 1) + "%";
      }
      updatedPayload += " | Elapsed:" + String(elapsedMin) + "m";
      updatedPayload += " | Free:" + String(freeMB, 0) + "MB";
      pCharacteristic->setValue(updatedPayload.c_str());
      pCharacteristic->notify();
    }
  }

  // --- CONTINUOUS AUDIO TRACK MANAGEMENT ---
  if (!isRecording) {
    startRecording();
    recordingStartTime = millis();
  } else {
    unsigned long elapsed = millis() - recordingStartTime;

    // Has our file cap expired? If so, cycle files.
    if (elapsed >= recordingTimeLimit) {
      stopRecording();
      startRecording();
      recordingStartTime = millis();
    } else {
      // Keep pushing audio frames down to the SD card
      appendAudioToSD();
    }
  }

  if (isRecording) {
    unsigned long ms = millis();
    if (!ledStateRec && ms - lastRecBlinkTime >= LED_BLINK_PERIOD_MS) {
      lastRecBlinkTime = ms;          // start of a flash
      ledStateRec = true;
      digitalWrite(LED_REC, HIGH);
    } else if (ledStateRec && ms - lastRecBlinkTime >= LED_PULSE_MS) {
      ledStateRec = false;            // end of the flash
      digitalWrite(LED_REC, LOW);
    }
  } else {
    // Ensure the LED is completely off when not recording
    digitalWrite(LED_REC, LOW);
    ledStateRec = false;
  }

  // --- ACTIVE WINDOW CHECK (every 10 s, validated RTC read) ---
  static unsigned long lastRtcCheck = 0;
  static uint8_t rtcBadReads = 0;
  if (millis() - lastRtcCheck >= 10000) {
    lastRtcCheck = millis();
    DateTime now;
    if (readRTC(now)) {
      rtcBadReads = 0;
      int currentMinutes = (now.hour() * 60) + now.minute();
      int start1 = (START_1_HR * 60) + START_1_MIN;
      int stop1  = (STOP_1_HR * 60) + STOP_1_MIN;
      int start2 = (START_2_HR * 60) + START_2_MIN;
      int stop2  = (STOP_2_HR * 60) + STOP_2_MIN;

      bool inWindow1 = (currentMinutes >= start1 && currentMinutes < stop1);
      bool inWindow2 = (currentMinutes >= start2 && currentMinutes < stop2);

      // If we drop out of BOTH windows, wrap up the current recording block and sleep
      if (!inWindow1 && !inWindow2) {
        Serial.println("Active window expired! Going to sleep.");
        logMessage("Active window expired! Going to sleep. ");
        if (isRecording) {
          stopRecording();
        }
        digitalWrite(MOSFET_GATE_PIN, HIGH);
        strlcpy(lastError, "window expired", sizeof(lastError));
        Serial.flush();
        esp_restart();
      }
    } else {
      // Bad or implausible RTC read: try to free the I2C bus, restart if it keeps failing
      i2cBusRecovery();
      if (++rtcBadReads >= 4) {
        strlcpy(lastError, "RTC unreadable in loop", sizeof(lastError));
        if (isRecording) stopRecording();
        esp_restart();
      }
    }
  }
}

float readBatteryVoltage() {
  const int SAMPLES = 16;
  uint32_t sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogReadMilliVolts(BAT_PIN);
    delay(2);
  }
  float pinMillivolts = sum / (float)SAMPLES;
  return (pinMillivolts * DIVIDER_RATIO) / 1000.0f;
}

float voltageToPercent(float v) {
  const float V_EMPTY = 3.30f;
  const float V_FULL  = 4.20f;
  float pct = (v - V_EMPTY) / (V_FULL - V_EMPTY) * 100.0f;
  if (pct > 100) pct = 100;
  if (pct < 0) pct = 0;
  return pct;
}

void startRecording() { 
  // Fetch the current date and time from the DS3231
DateTime now;
  if (!readRTC(now)) now = DateTime((uint32_t)(millis() / 1000));  // fallback filename timestamp
  syncSystemClockFromRTC();
  uint32_t pinMV = analogReadMilliVolts(BAT_PIN);
  float batV = readBatteryVoltage();
  batteryLevel = voltageToPercent(batV);
  char batMsg[96];
  snprintf(batMsg, sizeof(batMsg),
           "BAT_PIN=%d raw=%d pin_mV=%u batt_V=%.3f pct=%.0f",
           BAT_PIN, analogRead(BAT_PIN), (unsigned)pinMV, batV, batteryLevel);
  Serial.println(batMsg);
  logMessage(String(batMsg));
  float temp = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  uint64_t totalBytes = SD.totalBytes();
  uint64_t usedBytes = SD.usedBytes();
  float freeMB = (totalBytes - usedBytes) / (1024.0 * 1024.0);
  if (bmeFound) {
    temp     = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    char msgBuffer[112];
    snprintf(msgBuffer, sizeof(msgBuffer),
            "Temp: %.1f C | Hum: %.1f %% | Bat: %.1f%% | Free: %.1f MB\n",
            temp, humidity, batteryLevel, freeMB);
    Serial.print(msgBuffer);
    logMessage(msgBuffer);
  } else {
    Serial.println("BME not found");
  }
  if (pCharacteristic != nullptr) {
    char latStr[16], lngStr[16];
    dtostrf(globalLat, 1, 6, latStr);
    dtostrf(globalLng, 1, 6, lngStr);
    String updatedPayload = "Coords:" + String(latStr) + "," + String(lngStr) +
                            " | Bat:" + String(batteryLevel, 1) + "%";
    if (bmeFound) {
      updatedPayload += " | Temp:" + String(temp, 1) + "C" +
                        " | Hum:" + String(humidity, 1) + "%";
      updatedPayload += " | Elapsed: 0m";
      updatedPayload += " | Free:" + String(freeMB, 0) + "MB";
    }
    pCharacteristic->setValue(updatedPayload.c_str());
    pCharacteristic->notify(); // Pushes update directly to connected clients if subscriptions are enabled
    Serial.println("BLE Characteristic updated!");
  }
  if (pAdvertising != nullptr) {
    pAdvertising->stop();
    pAdvertising->start();
    Serial.println("BLE advertising refreshed.");
    logMessage("BLE advertising refreshed.");
  }
  char deviceName[16];
  strlcpy(deviceName, readStringFromEEPROM(eepromAddress).c_str(), sizeof(deviceName));
  snprintf(filename, sizeof(filename), "/%s_%04d-%02d-%02d-%02d%02d%02d_%.6f_%.6f.wav", 
          deviceName, now.year(), now.month(), now.day(), 
          now.hour(), now.minute(), now.second(), globalLat, globalLng);

  Serial.print("Recording has started: ");
  logMessage("Recording has started:  ");
  Serial.println(filename);
  logMessage(String(filename) + " ");


  file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("File is not Open! Trying to re-mount the SD card...");
    digitalWrite(LED_REC, LOW);

    // Try up to 3 times: re-mount the card, then open the file again
    for (int attempt = 1; attempt <= 3 && !file; attempt++) {
      digitalWrite(LED_DISC, HIGH);
      if (mountSD()) {
        file = SD.open(filename, FILE_WRITE);
      }
      digitalWrite(LED_DISC, LOW);
      if (!file) delay(500);
    }

    if (!file) {
      // Recovery failed: blink briefly so the fault is visible, then restart
      Serial.println("SD recovery failed. Restarting.");
      strlcpy(lastError, "file open failed in startRecording", sizeof(lastError));
      for (int i = 0; i < 6; i++) {
        digitalWrite(LED_DISC, HIGH);
        delay(250);
        digitalWrite(LED_DISC, LOW);
        delay(250);
      }
      esp_restart();
    }
    // Card is back. Log it now that the SD card works again.
    logMessage("File open failed; SD re-mounted OK and file opened.");
  }

  writeWavHeader(file, SAMPLE_RATE, 16, 1, 0);
  isRecording = true;
  lastRecBlinkTime = millis();
  ledStateRec = true;
  digitalWrite(LED_REC, ledStateRec);
}

void stopRecording() {
  if (!isRecording) return;
  isRecording = false;
  unsigned long fileSize = file.size() - WAVE_HEADER_SIZE;
  file.seek(0);
  writeWavHeader(file, SAMPLE_RATE, 16, 1, fileSize);
  file.close();
  Serial.println("Recording is Complete!");
  logMessage("Recording is Complete! ");
  digitalWrite(LED_REC, LOW); // Turn LED off when done
}

void appendAudioToSD() {
    int32_t i2sBuffer[256]; 
    size_t bytesRead = i2s.readBytes((char *)i2sBuffer, sizeof(i2sBuffer));
    int samplesCount = bytesRead / 4;
    if (samplesCount == 0) return;   // read timed out

    int16_t samples16[256];
    for (int i = 0; i < samplesCount; i++) {
      samples16[i] = (int16_t)(i2sBuffer[i] >> 16);
    }

    // Write data to SD Card
    size_t bytesWritten = file.write((uint8_t *)samples16, samplesCount * 2);
    
    if (bytesWritten == (size_t)(samplesCount * 2)) {
      writeFailures = 0;
      return;
    }

    // ---- write failed ----
    Serial.println("CRITICAL ERROR: Write failed!");
    writeFailures++;

    // 1) Is the card genuinely full? (totalBytes() == 0 means it isn't responding)
    uint64_t freeBytes = logStorageStatus();
    if (SD.totalBytes() > 0 && freeBytes < 512) {
      logMessage("Error Cause: Insufficient storage space on SD card. ");
      stopRecording();                 // finalize the WAV header and close cleanly
      while (true) {                   // nothing more to do until the card is swapped
        digitalWrite(LED_DISC, HIGH);
        delay(1250);
        digitalWrite(LED_DISC, LOW);
        delay(250);
      }
    }

    // 2) Otherwise assume a card or bus fault
    strlcpy(lastError, "write failed", sizeof(lastError));
    if (writeFailures >= 3) {          // keeps failing: stop retrying and restart
      esp_restart();
    }

    // 3) Re-mount the card and start a new file
    isRecording = false;
    file.close();                      // may fail on a dead card, which is harmless
    if (!mountSD()) {
      esp_restart();
    }
    startRecording();                  // has its own retry-then-restart logic
    recordingStartTime = millis();
}

// Function to get a String out of EEPROM safely
String readStringFromEEPROM(int address) {
  char charBuffer[MAX_STRING_LENGTH];
  // EEPROM.get reads the exact number of bytes needed to fill the char array
  EEPROM.get(address, charBuffer);
  if ((uint8_t)charBuffer[0] == 0xFF || charBuffer[0] == '\0') {
    return String(DEFAULT_DEVICE_NAME);
  }
  // Convert the character array back into a readable Arduino String object
  return String(charBuffer);
}

void writeStringToEEPROM(int address, String data) {
  char charBuffer[MAX_STRING_LENGTH];
  // Ensure the string fits inside our fixed buffer, leaving room for the null terminator '\0'
  data.toCharArray(charBuffer, MAX_STRING_LENGTH);
  // EEPROM.put automatically loops through the char array bytes and updates them
  EEPROM.put(address, charBuffer);
  // CRITICAL FOR ESP32: Push the RAM buffer changes into actual Flash memory
  EEPROM.commit();
}

void writeWavHeader(File &file, int sampleRate, int bitsPerSample, int channels, int dataSize) {
  byte header[WAVE_HEADER_SIZE];
  int fileSize = dataSize + WAVE_HEADER_SIZE - 8;
  int byteRate = sampleRate * channels * (bitsPerSample / 8);

  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  header[4] = (byte)(fileSize & 0xFF);
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0;  
  header[20] = 1; header[21] = 0;  
  header[22] = channels; header[23] = 0;
  header[24] = (byte)(sampleRate & 0xFF);
  header[25] = (byte)((sampleRate >> 8) & 0xFF);
  header[26] = (byte)((sampleRate >> 16) & 0xFF);
  header[27] = (byte)((sampleRate >> 24) & 0xFF);
  header[28] = (byte)(byteRate & 0xFF);
  header[29] = (byte)((byteRate >> 8) & 0xFF);
  header[30] = (byte)((byteRate >> 16) & 0xFF);
  header[31] = (byte)((byteRate >> 24) & 0xFF);
  header[32] = (byte)(channels * (bitsPerSample / 8));
  header[33] = 0;  
  header[34] = bitsPerSample; header[35] = 0;  
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  header[40] = (byte)(dataSize & 0xFF);
  header[41] = (byte)((dataSize >> 8) & 0xFF);
  header[42] = (byte)((dataSize >> 16) & 0xFF);
  header[43] = (byte)((dataSize >> 24) & 0xFF); 

  file.write(header, WAVE_HEADER_SIZE);
}