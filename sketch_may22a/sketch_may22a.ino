#include "FS.h"
#include "SD.h"
#include "SPI.h"

// Set these to match your wiring
#define SD_CS   10
#define SD_MOSI 11
#define SD_MISO 13
#define SD_SCK  12

void setup() {
  Serial.begin(9600);
  
  // Initialize SPI with S3-specific pin
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  Serial.print("Initializing SD card... ");
  if (!SD.begin(SD_CS)) {
    Serial.println("FAILED!");
    return;
  }
  Serial.println("SUCCESS!");

  // Basic Write Test
  File file = SD.open("/test.txt", FILE_WRITE);
  if (file) {
    file.println("ESP32-S3 SD Test Successful!");
    file.close();
    Serial.println("File written.");
  } else {
    Serial.println("Error opening file for writing.");
  }

  // Basic Read Test
  file = SD.open("/test.txt");
  if (file) {
    Serial.print("Read from file: ");
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
  }
}

void loop() {
  // Nothing here for a simple test
}
