#include "Arduino.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "driver/i2s.h"

// Pin Definitions for ESP32-S3
#define I2S_WS          4
#define I2S_SD          5
#define I2S_SCK         6
#define PIN_SD_CS       7

// Audio Configuration
#define I2S_PORT        I2S_NUM_0
#define SAMPLE_RATE     16000
#define BUFFER_SIZE     1024

// Sound Trigger Thresholds
#define NOISE_THRESHOLD 5000000     // Adjust based on your mic sensitivity
#define QUIET_DURATION  3000        // Hold recording for 3 seconds of silence

// Global Variables
File audioFile;
bool isRecording = false;
unsigned long lastNoiseTime = 0;
uint32_t totalAudioBytes = 0;

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // INMP441 uses 32-bit data
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void writeWavHeader(File file) {
  byte header[44];
  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  uint32_t fileSize = totalAudioBytes + 36;
  header[4] = fileSize & 0xff; header[5] = (fileSize >> 8) & 0xff; header[6] = (fileSize >> 16) & 0xff; header[7] = (fileSize >> 24) & 0xff;
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0; // Subchunk1Size
  header[20] = 1; header[21] = 0; // AudioFormat (PCM)
  header[22] = 1; header[23] = 0; // NumChannels (Mono)
  header[24] = SAMPLE_RATE & 0xff; header[25] = (SAMPLE_RATE >> 8) & 0xff; header[26] = (SAMPLE_RATE >> 16) & 0xff; header[27] = (SAMPLE_RATE >> 24) & 0xff;
  uint32_t byteRate = SAMPLE_RATE * 1 * 2; // 16-bit payload calculation
  header[28] = byteRate & 0xff; header[29] = (byteRate >> 8) & 0xff; header[30] = (byteRate >> 16) & 0xff; header[31] = (byteRate >> 24) & 0xff;
  header[32] = 2; header[33] = 0; // BlockAlign
  header[34] = 16; header[35] = 0; // BitsPerSample (Downsampled to 16-bit for standard compatibility)
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  header[40] = totalAudioBytes & 0xff; header[41] = (totalAudioBytes >> 8) & 0xff; header[42] = (totalAudioBytes >> 16) & 0xff; header[43] = (totalAudioBytes >> 24) & 0xff;
  
  file.seek(0);
  file.write(header, 44);
}

void startRecording() {
  String filename = "/rec_" + String(millis()) + ".wav";
  audioFile = SD.open(filename, FILE_WRITE);
  if (!audioFile) {
    Serial.println("Failed to open file for recording");
    return;
  }
  
  // Leave space for the header
  byte blankHeader[44] = {0};
  audioFile.write(blankHeader, 44);
  
  totalAudioBytes = 0;
  isRecording = true;
  Serial.println("Noise detected! Recording started: " + filename);
}

void stopRecording() {
  if (!isRecording) return;
  writeWavHeader(audioFile);
  audioFile.close();
  isRecording = false;
  Serial.println("Environment quiet. Recording stopped.");
}

void setup() {
  Serial.begin(115200);
  setupI2S();

  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("SD Card Mount Failed!");
    while (1);
  }
  Serial.println("System Ready. Monitoring noise...");
}

void loop() {
  int32_t i2s_buffer[BUFFER_SIZE];
  size_t bytes_read;
  
  // Read data from I2S microphone
  i2s_read(I2S_PORT, &i2s_buffer, sizeof(i2s_buffer), &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / 4;

  if (samples_read > 0) {
    long long peak_to_peak = 0;
    int32_t max_sample = -2147483648;
    int32_t min_sample = 2147483647;

    // Process chunk for sound level and downsample to 16-bit
    int16_t out_buffer[BUFFER_SIZE];
    for (int i = 0; i < samples_read; i++) {
      int32_t sample = i2s_buffer[i] >> 14; // Clear raw low bits
      
      if (sample > max_sample) max_sample = sample;
      if (sample < min_sample) min_sample = sample;

      // Scale to 16-bit PCM for WAV storage
      out_buffer[i] = (int16_t)(sample >> 16); 
    }

    peak_to_peak = (long long)max_sample - min_sample;

    // Noise Check
    if (peak_to_peak > NOISE_THRESHOLD) {
      lastNoiseTime = millis();
      if (!isRecording) {
        startRecording();
      }
    }

    // Write data if active
    if (isRecording) {
      size_t bytes_to_write = samples_read * 2;
      audioFile.write((uint8_t*)out_buffer, bytes_to_write);
      totalAudioBytes += bytes_to_write;

      // Timeout check for silence
      if (millis() - lastNoiseTime > QUIET_DURATION) {
        stopRecording();
      }
    }
  }
}