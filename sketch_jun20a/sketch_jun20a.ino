#include <TinyGPS++.h>

// --- GPS GLOBAL DEFAULTS and Variables ---
const int RXD1 = 16;  // Connect to the TX pin of the GPS!!!! reversed!!
const int TXD1 = 15;  // Connect to the RX pin of the GPS!!!! reversed!!
const double DEFAULT_LAT = 0.0;
const double DEFAULT_LNG = 0.0;
double globalLat = DEFAULT_LAT;
double globalLng = DEFAULT_LNG;
bool hasValidGpsFix = false;
unsigned long wiringLastDataTime = 0;
unsigned long lastGPSHourlyUpdate = 0;
const unsigned long GPS_ONE_HOUR_MS = 3600000;      // 60 mins * 60 secs * 1000 ms
const unsigned long GPS_SETUP_TIMEOUT_MS = 15000;   // 15 seconds max wait in setup

TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  
  // Initialize Serial1
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  Serial.println("ESP32-S3 GPS Test Initialized.");

  unsigned long setupStart = millis();

  while (!hasValidGpsFix && (millis() - setupStart < GPS_SETUP_TIMEOUT_MS)) {
    while (Serial1.available() > 0) {
      char c = Serial1.read();
      if (gps.encode(c)) {
        if (gps.location.isValid()) {
          globalLat = gps.location.lat();
          globalLng = gps.location.lng();
          hasValidGpsFix = true;
          
          Serial.println("\n--- Setup GPS Fix Acquired! ---");
          Serial.print("Initial Coordinates: ");
          Serial.print(globalLat, 6);
          Serial.print(", ");
          Serial.println(globalLng, 6);
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
    Serial.println("\n--- Setup GPS Timeout ---");
    Serial.println("Could not get a satellite lock in time.");
  }

  // Synchronize our timers right as setup finishes
  wiringLastDataTime = millis();
  lastGPSHourlyUpdate = millis();
}

void loop() {

  bool dataReceived = false;

  // Keep feeding TinyGPS++ constantly in the background
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    dataReceived = true;
    gps.encode(c);
  }

  if (dataReceived) {
    wiringLastDataTime = millis(); // Reset wiring timeout tracker
  }

  // --- HOURLY UPDATE LOGIC (Non-blocking) ---
  if (millis() - lastGPSHourlyUpdate >= GPS_ONE_HOUR_MS) {
    updateHourlyCoordinates();
    lastGPSHourlyUpdate = millis(); // Reset the 1-hour timer
  }

  // Wiring check: If 5 seconds pass without ANY raw data over the serial line
  if (millis() - wiringLastDataTime > 5000) {
    Serial.println("Error: No raw GPS data detected. Check your wiring or pins!");
    wiringLastDataTime = millis(); 
  }

}

void updateHourlyCoordinates() {
  Serial.println("\n--- Triggering Hourly GPS Update ---");
  
  if (gps.location.isValid()) {
    globalLat = gps.location.lat();
    globalLng = gps.location.lng();
    hasValidGpsFix = true;

    Serial.print("Global Coordinates Updated: ");
    Serial.print(globalLat, 6);
    Serial.print(", ");
    Serial.println(globalLng, 6);
  } else {
    hasValidGpsFix = false;
    Serial.println("Hourly update failed: No valid GPS fix at this moment.");
  }
}