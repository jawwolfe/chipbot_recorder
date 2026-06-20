#include <TinyGPS++.h>

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Use safer, unassigned GPIOs for the ESP32-S3
#define RXD1 18  // Connect to the TX pin of the GPS!!!! reversed!!
#define TXD1 17  // Connect to the RX pin of the GPS!!!! reversed!!

// --- GLOBAL DEFAULTS ---
// Set these to whatever fallback coordinates you want 
const double DEFAULT_LAT = 0.0;
const double DEFAULT_LNG = 0.0;

// Global variables for your active coordinates
double globalLat = DEFAULT_LAT;
double globalLng = DEFAULT_LNG;
bool hasValidGpsFix = false;

// Non-blocking timer variables
unsigned long lastDataTime = 0;
unsigned long lastHourlyUpdate = 0;
const unsigned long ONE_HOUR_MS = 3600000;      // 60 mins * 60 secs * 1000 ms
const unsigned long SETUP_TIMEOUT_MS = 15000;   // 15 seconds max wait in setup

void setup() {
  Serial.begin(9600);
  
  // Initialize Serial1 with safe ESP32-S3 pins
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);

  Serial.println("ESP32-S3 GPS Test Initialized.");
  Serial.print("Attempting to get initial GPS fix (Timeout: ");
  Serial.print(SETUP_TIMEOUT_MS / 1000);
  Serial.println(" seconds)...");

  unsigned long setupStart = millis();

  // --- SETUP UPDATE WITH TIMEOUT ---
  while (!hasValidGpsFix && (millis() - setupStart < SETUP_TIMEOUT_MS)) {
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
          Serial.println("--------------------------------\n");
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
    Serial.print("Using default coordinates: ");
    Serial.print(globalLat, 6);
    Serial.print(", ");
    Serial.println(globalLng, 6);
    Serial.println("---------------------------\n");
  }

  // Synchronize our timers right as setup finishes
  lastDataTime = millis();
  lastHourlyUpdate = millis();
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
    lastDataTime = millis(); // Reset wiring timeout tracker
  }

  // --- HOURLY UPDATE LOGIC (Non-blocking) ---
  if (millis() - lastHourlyUpdate >= ONE_HOUR_MS) {
    updateHourlyCoordinates();
    lastHourlyUpdate = millis(); // Reset the 1-hour timer
  }

  // Wiring check: If 5 seconds pass without ANY raw data over the serial line
  if (millis() - lastDataTime > 5000) {
    Serial.println("Error: No raw GPS data detected. Check your wiring or pins!");
    lastDataTime = millis(); 
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
  Serial.println("------------------------------------\n");
}