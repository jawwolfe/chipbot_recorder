#include <dummy.h>
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>
#include <FS.h>
#include <Wire.h>
#include <RTClib.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <EEPROM.h>
#include <TinyGPS++.h>

// --- I2S MIC GLOBAL DEFAULTS and VARIABLES ---
const int I2S_BCK_PIN = 4;
const int I2S_WS_PIN = 5;
const int I2S_SD_PIN = 6;
const unsigned int SAMPLE_RATE = 48000;  // DVD quality
constexpr i2s_port_t I2S_NUM = I2S_NUM_0;
const int WAVE_HEADER_SIZE = 44;
const i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,  
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_STAND_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 8,
  .dma_buf_len = 1024,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0
};
const i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_BCK_PIN,
  .ws_io_num = I2S_WS_PIN,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = I2S_SD_PIN
};

// --- Recording constraints ---
const int recordingTimeLimit = 30000; // 30 seconds limit
const float soundThresholdMultiplier = 1.3; // Starts recording if 1.5x louder than quiet
const int silenceTimeout = 5000; // Stop if silent for 5 seconds
bool isRecording = false;
unsigned long recordingStartTime = 0;
unsigned long lastSoundTime = 0;
unsigned long baselineNoise = 0;

// --- RTC MODULE GLOBAL DEFAULTS and VARIABLES ---
const int RTC_SDA_PIN = 8;
const int RTC_SCL_PIN = 9;

// --- DAILY WAKEUP WINDOWS (in 24-hour format) --
const int START_1_HR = 1;   // Window 1
const int START_1_MIN = 1;
const int STOP_1_HR = 11;    // Window 1 End
const int STOP_1_MIN = 59;
const int START_2_HR = 12;  // Window 2 Start
const int START_2_MIN = 0;
const int STOP_2_HR = 24;   // Window 2 End
const int STOP_2_MIN = 59;

// --- SD CARD MODULE GLOBAL DEFAULTS and VARIABLES ---
const int SD_CS_PIN = 10;
const int SD_MOSI_PIN = 11;
const int SD_MISO_PIN = 13;
const int SD_CLK_PIN = 12; //also know as SCK pin 
// -- for blinking yellow light for disk error when recording --
unsigned long startDiscError = 0;
const unsigned long intDiscError = 10000;

// --- GPS GLOBAL DEFAULTS and Variables ---
const int RXD1 = 16;  // Connect to the TX pin of the GPS!!!! reversed!!
const int TXD1 = 15;  // Connect to the RX pin of the GPS!!!! reversed!!
const double DEFAULT_LAT = 0.0;
const double DEFAULT_LNG = 0.0;
double globalLat = DEFAULT_LAT;
double globalLng = DEFAULT_LNG;
bool hasValidGpsFix = false;
const unsigned long GPS_SETUP_TIMEOUT_MS = 1200000;   // 15 seconds max wait in setup
const int MOSFET_GATE_PIN  = 1;
//TIMEZONE
RTC_DATA_ATTR int savedTimezoneOffsetHours = 0; 
RTC_DATA_ATTR bool timezoneKnown = false;

// --- LED PIN DEFINITIONS ---
const int LED_PIN = 14; // blue led for recording or listening
const int LED_PIN_2 = 17; // yellow led for SD disc status full or not ready
const int LED_PIN_3 = 7; // red led for battery voltage checks
// -- used in startup disk check --
int ledStateRecording = LOW; 
// -- used in recording long blink when sampling--
const long int_samp_blnk = 1000;
unsigned long prevMillisBlnk = 0;   

// --- BLUETOOTH BLE ---
const char* const SERVICE_UUID        = "fdcff45e-438b-4a62-acf6-dbd852aae4b1";
const char* const CHARACTERISTIC_UUID = "cf28c230-d88e-4e6e-8a2e-0efc4d8ec072";

// --- BAT POWER MONITORING ---
float batteryLevel;
const int BAT_PIN = 18;
const float LOW_BATTERY_THRESHOLD = 82.0f; // Flash below 80%
const unsigned long FLASH_INTERVAL = 500;  // Flash every 500ms
unsigned long lastFlashTime = 0;
bool ledStateBattery = false;

// --- EEPROM CONSTANTS for storing device name---
const int MAX_STRING_LENGTH = 16; 
const int eepromAddress = 0; // memory address

// --- FILE LOGGING ---
unsigned long lastRotationTime = 0;
const unsigned long ONE_HOUR_MS = 3600000; // 1 hour in milliseconds
String currentFileName; // Determined dynamically on boot/rotation

// -- INSTANTIATE GLOBAL FILE NAME AND RTC --
File file;
File file1;
char filename[92];
RTC_DS3231 rtc;
TinyGPSPlus gps;

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
  
void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);
  digitalWrite(LED_PIN_3, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED_PIN_2, LOW);
  pinMode(MOSFET_GATE_PIN, OUTPUT);
  digitalWrite(MOSFET_GATE_PIN, LOW);

  // -- Initilize RTC and SD
  if (!rtc.begin()) {
    Serial.println("Error: Could not find DS3231 module. Check your wiring!");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, letting's set the time!");
    // This sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  SPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card not Initialize!");
    while (1) {
      digitalWrite(LED_PIN_2, HIGH);
      delay(500);
      digitalWrite(LED_PIN_2, LOW);
      delay(500);
    }
  }

  // Get current time from the DS3231 module
  DateTime now = rtc.now();
  // Log file work establish filename format with minutes from RTC for initial boot
  char bootLogName[40];
  snprintf(bootLogName, sizeof(bootLogName), "/debug_%04d%02d%02d_%02d%02d.txt", 
           now.year(), now.month(), now.day(), now.hour(), now.minute());
  currentFileName = String(bootLogName);
  lastRotationTime = millis();

  // -- Initialize I2C  mic
  bool i2c_ok = Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  if (!i2c_ok) {
    Serial.println("Error: Failed to initialize I2C bus.");
    logMessage("Error: Failed to initialize I2C bus. ");
    while (1);
  }
  // Initialize Serial1
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  unsigned long setupStart = millis();
  batteryLevel = map(analogRead(BAT_PIN), 0.0f, 4095.0f, 0, 100);

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
  //String myString = "aw_chipbot_01";  
  //writeStringToEEPROM(eepromAddress, myString);
  //Serial.println("String saved successfully.");
  //String retrievedString = readStringFromEEPROM(eepromAddress);
  //Serial.print("Retrieved String: ");
  //Serial.println(retrievedString);

  if (inWindow1 || inWindow2) {
    // We are supposed to be awake! Proceed to void loop()
    digitalWrite(MOSFET_GATE_PIN, LOW);
    Serial.println("Waiting 20 min max for GPS coords.");
    logMessage("Waiting 20 min max for GPS coords. ");
    // GET initial GPS coordinates
    while (!hasValidGpsFix && (millis() - setupStart < GPS_SETUP_TIMEOUT_MS)) {
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
            logMessage (latStr);
            Serial.print(", ");
            logMessage(", ");
            Serial.println(longStr);
            logMessage(longStr);

            struct tm timeinfo;
            if (getLocalTime(&timeinfo) && timezoneKnown) {
                printLocalTime();
            } else {
                Serial.println("Cold boot or invalid time. Waiting for GPS fix...");
                logMessage("Cold boot or invalid time. Waiting for GPS fix... ");
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
    delay(1); // Keep the ESP32-S3 watchdog happy
    }
    // If the loop finished but we never got a valid fix, use defaults
    if (!hasValidGpsFix) {
      globalLat = DEFAULT_LAT;
      globalLng = DEFAULT_LNG;
      Serial.println("Could not get a satellite lock in time.");
      logMessage("Could not get a satellite lock in time. ");
    }
    digitalWrite(MOSFET_GATE_PIN, HIGH);

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
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
    String initCharacteristic = "Coordinates: " + coordinatesString;
    pCharacteristic->setValue(initCharacteristic);
        pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    // MIN & MAX advertising intervals. 
    // 0x0640 * 0.625 ms = 1000 ms (1 second)
    // Higher interval = lower power, but takes longer for your Android phone to discover.
    pAdvertising->setMinInterval(0x0640); 
    pAdvertising->setMaxInterval(0x0640);
      BLEDevice::startAdvertising();
    Serial.println("Characteristic defined! Now advertising...");
    // This allows the ESP32-S3 to automatically enter light sleep 
    // between BLE advertising intervals.
    esp_sleep_enable_timer_wakeup(1000000); // Optional: dynamic fallback
    #if CONFIG_PM_ENABLE
      esp_pm_config_esp32s3_t pm_config = {
          .max_freq_mhz = 240,
          .min_freq_mhz = 40,      // XTAL frequency
          .light_sleep_enable = true
      };
      esp_pm_configure(&pm_config);
    #endif

    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM);
    calibrateNoiseFloor();
    Serial.println("System is Ready. Waiting for sound trigger");
    logMessage("System is Ready. Waiting for sound trigger");

    } else {
      // We are outside the windows. Calculate sleep duration and go to sleep immediately.
      digitalWrite(MOSFET_GATE_PIN, HIGH);
      int minutesToSleep = 0;

      if (currentMinutes < start1) {
        // It's early morning, sleep until Window 1
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

      Serial.printf("Outside active windows. Sleeping for %ld seconds...", secondsToSleep);
      logMessage("Outside active windows. Sleeping for %ld seconds..." + String(secondsToSleep));
      Serial.flush();

      // Configure deep sleep timer (expects microseconds)
      esp_sleep_enable_timer_wakeup((uint64_t)secondsToSleep * 1000000ULL);
      esp_deep_sleep_start();
  }
}

void loop() {  

  // Check if battery is low
  if (batteryLevel < LOW_BATTERY_THRESHOLD) {
    // Non-blocking flash logic
    unsigned long currentMillis = millis();
    if (currentMillis - lastFlashTime >= FLASH_INTERVAL) {
      lastFlashTime = currentMillis;
      ledStateBattery = !ledStateBattery; // Toggle the LED state
      digitalWrite(LED_PIN_3, ledStateBattery);
    }
  } else {
    // Force the LED off if battery is safe
    digitalWrite(LED_PIN_3, LOW);
    ledStateBattery = false;
  }

  // 1. If NOT recording, we sample the microphone to look for the trigger sound
  if (!isRecording) {
    //put the slow blink of blue LED_PIN using millis here
    unsigned long curMillisBlnk = millis();
    // Check if the time difference is greater than or equal to the interval
    if (curMillisBlnk - prevMillisBlnk >= int_samp_blnk) {
      // Save the last time you blinked the LED
      prevMillisBlnk = curMillisBlnk;
      // If the LED is off, turn it on, and vice-versa
      if (ledStateRecording == LOW) {
        ledStateRecording = HIGH; 
      } else {
        ledStateRecording = LOW;
      }
      // Update the physical LED pin with the new state
      digitalWrite(LED_PIN, ledStateRecording);
    }

    float currentVolume = readMicrophoneVolume();
    
    if (currentVolume > (baselineNoise * soundThresholdMultiplier)) {
      startRecording();
      recordingStartTime = millis();
      lastSoundTime = millis();
    }
  } 
  // 2. If WE ARE recording, we process audio inside appendAudioToSD
  else {
    unsigned long elapsed = millis() - recordingStartTime;
    unsigned long silenceDuration = millis() - lastSoundTime; // Use unsigned long for millis

    // Check time limit OR silence limit
    if (elapsed >= recordingTimeLimit || silenceDuration >= silenceTimeout) {
      stopRecording();
    } else {
      // appendAudioToSD now returns true if it detected sound, false if quiet
      bool soundDetected = appendAudioToSD();
      
      if (soundDetected) {
        lastSoundTime = millis(); // Reset silence timer
      }
    }
  }

  // Dynamically check if our active window has just expired
  DateTime now = rtc.now();
  int currentMinutes = (now.hour() * 60) + now.minute();
  int start1 = (START_1_HR * 60) + START_1_MIN;
  int stop1  = (STOP_1_HR * 60) + STOP_1_MIN;
  int start2 = (START_2_HR * 60) + START_2_MIN;
  int stop2  = (STOP_2_HR * 60) + STOP_2_MIN;

  bool inWindow1 = (currentMinutes >= start1 && currentMinutes < stop1);
  bool inWindow2 = (currentMinutes >= start2 && currentMinutes < stop2);

  // If we drop out of BOTH windows, force a restart so setup() can handle the deep sleep math
  if (!inWindow1 && !inWindow2) {
    Serial.println("Active window expired! Going to sleep.");
    logMessage("Active window expired! Going to sleep. ");
    digitalWrite(MOSFET_GATE_PIN, HIGH);
    Serial.flush();
    esp_restart(); 
  }
}

void calibrateNoiseFloor() {
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += readMicrophoneVolume();
    delay(10);
  }
  baselineNoise = sum / 100.0;
}

// Helper to calculate RMS from raw PCM data (Used only when idling)
float readMicrophoneVolume() {
  int32_t raw_samples[512];
  size_t bytes_read;
  i2s_read(I2S_NUM_0, (void**)raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY);
  
  float sum_squares = 0;
  int samples_count = bytes_read / sizeof(int32_t);
  if (samples_count == 0) return 0;

  for (int i = 0; i < samples_count; i++) {
    int16_t sample16 = (int16_t)(raw_samples[i] >> 14); 
    float sample = (float)sample16;
    sum_squares += sample * sample;
  }
  return sqrt(sum_squares / samples_count);
}

void startRecording() { 
  // Fetch the current date and time from the DS3231 
  DateTime now = rtc.now();
  char deviceName[16];
  strlcpy(deviceName, readStringFromEEPROM(eepromAddress).c_str(), sizeof(deviceName));
  snprintf(filename, sizeof(filename), "/%s_%04d-%02d-%02d_%02d_%02d_%02d_%.6f_%.6f.wav", 
          deviceName, now.year(), now.month(), now.day(), 
          now.hour(), now.minute(), now.second(), globalLat, globalLng);

  Serial.print("Recording has started: ");
  logMessage("Recording has started:  ");
  Serial.println(filename);
  logMessage(String(filename) + " ");
  file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("File is not Open!");
    logMessage("File is not Open! ");
    //Turn off blue 3 led
    digitalWrite(LED_PIN, LOW);
    //Fast blink the 17 yellow for 10 seconds then return to lisitening
    startDiscError = millis();
    while (millis() - startDiscError <= intDiscError) {
        digitalWrite(LED_PIN_2, HIGH);
        delay(500);
        digitalWrite(LED_PIN_2, LOW);
        delay(500);
    }
    return;
  }
   writeWavHeader(file, SAMPLE_RATE, 16, 1, 0);
  isRecording = true;
  digitalWrite(LED_PIN, HIGH); // Turn LED on when starting
}

void stopRecording() {
  isRecording = false;
  unsigned long fileSize = file.size() - WAVE_HEADER_SIZE;
  file.seek(0);
  writeWavHeader(file, SAMPLE_RATE, 16, 1, fileSize);
  file.close();
  Serial.println("Recording is Complete!");
  logMessage("Recording is Complete! ");
  digitalWrite(LED_PIN, LOW); // Turn LED off when done
}

// Fixed function: Writes to SD AND checks volume at the same time
bool appendAudioToSD() {
    size_t bytesRead;
    int32_t i2sBuffer[256]; 
 
    i2s_read(I2S_NUM_0, (void *)i2sBuffer, sizeof(i2sBuffer), &bytesRead, portMAX_DELAY);

    int16_t samples16[256];
    int samplesCount = bytesRead / 4;
    if (samplesCount == 0) return false;

    float sum_squares = 0;

    for (int i = 0; i < samplesCount; i++) {
      // Downsample 32-bit to 16-bit
      samples16[i] = (int16_t)(i2sBuffer[i] >> 14);
      
      // Calculate volume on the fly
      float sample = (float)samples16[i];
      sum_squares += sample * sample;
    }

    // Write data to SD Card
    size_t bytesWritten = file.write((uint8_t *)samples16, samplesCount * 2);

    // If bytes written don't match buffer size, the card is likely full
    if (bytesWritten < 512) {
      Serial.println("CRITICAL ERROR: Write failed!");
      logMessage("CRITICAL ERROR: Write failed! ");
      uint64_t totalBytes = SD.totalBytes();
      uint64_t usedBytes = SD.usedBytes();
      uint64_t freeBytes = totalBytes - usedBytes;

      Serial.printf("Storage Status: %llu / %llu bytes used.\n", usedBytes, totalBytes);
      //logMessage("Storage Status: %llu / %llu bytes used.\n", usedBytes, totalBytes);
      Serial.printf("Remaining Space: %llu bytes.", freeBytes);
      //logMessage("Remaining Space: %llu bytes.", freeBytes);
      if (freeBytes < 512) {
          Serial.println("Error Cause: Insufficient storage space on SD card.");
          logMessage("Error Cause: Insufficient storage space on SD card. ");
          // Halt execution or trigger a system alert/LED here
          digitalWrite(LED_PIN, LOW); 
          while (true) { 
            digitalWrite(LED_PIN_2, HIGH);
            delay(1250); 
            digitalWrite(LED_PIN_2, LOW);
          } 
      } else {
          Serial.println("Error Cause: Hardware disconnect or file corruption.");
          logMessage("Error Cause: Hardware disconnect or file corruption. ");
          while (true) { 
            digitalWrite(LED_PIN_2, HIGH);
            delay(1250); 
            digitalWrite(LED_PIN_2, LOW);
          } 
      }
    }
    // Calculate current RMS volume of this chunk
    float currentVolume = sqrt(sum_squares / samplesCount);
    // Return true if volume is above the threshold
    return (currentVolume > (baselineNoise * soundThresholdMultiplier));
}

// Function to get a String out of EEPROM safely
String readStringFromEEPROM(int address) {
  char charBuffer[MAX_STRING_LENGTH];
  
  // EEPROM.get reads the exact number of bytes needed to fill the char array
  EEPROM.get(address, charBuffer);
  
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

  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  header[4] = (byte)(fileSize & 0xFF);
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 16;
  header[17] = 0;
  header[18] = 0;
  header[19] = 0;  // PCM Chunk Size
  header[20] = 1;
  header[21] = 0;  // PCM Format
  header[22] = channels;
  header[23] = 0;
  header[24] = (byte)(sampleRate & 0xFF);
  header[25] = (byte)((sampleRate >> 8) & 0xFF);
  header[26] = (byte)((sampleRate >> 16) & 0xFF);
  header[27] = (byte)((sampleRate >> 24) & 0xFF);
  header[28] = (byte)(byteRate & 0xFF);
  header[29] = (byte)((byteRate >> 8) & 0xFF);
  header[30] = (byte)((byteRate >> 16) & 0xFF);
  header[31] = (byte)((byteRate >> 24) & 0xFF);
  header[32] = (byte)(channels * (bitsPerSample / 8));
  header[33] = 0;  // Block Align
  header[34] = bitsPerSample;
  header[35] = 0;  // Bits Per Sample
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (byte)(dataSize & 0xFF);
  header[41] = (byte)((dataSize >> 8) & 0xFF);
  header[42] = (byte)((dataSize >> 16) & 0xFF);
  header[43] = (byte)((dataSize >> 24) & 0xFF); 

  file.write(header, WAVE_HEADER_SIZE);
}

void checkLogRotation() {
  if (millis() - lastRotationTime >= ONE_HOUR_MS) {
    lastRotationTime = millis();

    DateTime now = rtc.now();
    char rotationBuffer[40];
    snprintf(rotationBuffer, sizeof(rotationBuffer), "/debug_%04d%02d%02d_%02d%02d.txt", 
             now.year(), now.month(), now.day(), now.hour(), now.minute());
    
    Serial.print("--- One hour elapsed. Rotating logs to: ");
    logMessage("--- One hour elapsed. Rotating logs to:  ");
    Serial.print(currentFileName);
    logMessage(currentFileName);
  }
}
