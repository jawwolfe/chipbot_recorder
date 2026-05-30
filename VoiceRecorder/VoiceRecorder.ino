#include <RTClib.h>

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>
#include <FS.h>
#include <Wire.h>
#include <RTClib.h>

#define I2S_BCK_PIN 4
#define I2S_WS_PIN 5
#define I2S_SD_PIN 6

#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 13
#define SD_CLK_PIN 12 

#define LED_PIN 3
#define LED_PIN_2 17

#define I2C_SDA 8
#define I2C_SCL 9

#define SAMPLE_RATE 48000  
#define I2S_NUM I2S_NUM_0
#define WAVE_HEADER_SIZE 44

// Recording constraints
const int recordingTimeLimit = 30000; // 30 seconds
const float soundThresholdMultiplier = 1.4; 
const int silenceTimeout = 5000; // 5 seconds

// RAM Audio Buffer Optimization (Packs data before writing to SD)
#define AUDIO_BUF_SIZE 8192  // 8KB chunk sizing is highly optimal for SD blocks
int16_t ramAudioBuffer[AUDIO_BUF_SIZE];
int ramBufferIndex = 0;

File file;
bool isRecording = false;
unsigned long recordingStartTime = 0;
unsigned long lastSoundTime = 0;
float baselineNoiseSquared = 0; // Using squared value to avoid costly sqrt() operations
char filename[64];
RTC_DS3231 rtc;
 
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

// Forward Declarations
void writeWavHeader(File &file, int sampleRate, int bitsPerSample, int channels, int dataSize);
void calibrateNoiseFloor();
float readMicrophoneVolume();
void startRecording();
void stopRecording();
bool appendAudioToSD();
void flushAudioBuffer();

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED_PIN_2, LOW);

  if (!Wire.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("Error: Failed to initialize I2C bus.");
    while (1);
  }

  if (!rtc.begin()) {
    Serial.println("Error: Could not find DS3231 module.");
    while (1);
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  SPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card not Initialize!");
    while (1) {
      digitalWrite(LED_PIN, HIGH); delay(500);
      digitalWrite(LED_PIN, LOW); delay(500);
    }
  }
  
  Serial.println("SD Card Ready.");
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM);
  
  calibrateNoiseFloor();
  Serial.println("System Ready. Listening for sound trigger...");
}

void loop() {  
  if (!isRecording) {
    float currentVolume = readMicrophoneVolume();
    float currentVolumeSquared = currentVolume * currentVolume;
    
    if (currentVolumeSquared > (baselineNoiseSquared * soundThresholdMultiplier)) {
      Serial.println("Threshold exceeded! Start Recording...");
      startRecording();
      recordingStartTime = millis();
      lastSoundTime = millis();
    }
  } 
  else {
    unsigned long elapsed = millis() - recordingStartTime;
    unsigned long silenceDuration = millis() - lastSoundTime;

    if (elapsed >= recordingTimeLimit || silenceDuration >= silenceTimeout) {
      stopRecording();
    } else {
      if (appendAudioToSD()) {
        lastSoundTime = millis(); 
      }
    }
  }
}

void calibrateNoiseFloor() {
  float sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += readMicrophoneVolume();
    delay(10);
  }
  float avgBaseline = sum / 50.0;
  baselineNoiseSquared = avgBaseline * avgBaseline; // Store squared value to avoid math overhead later
  Serial.print("Calibrated Baseline Noise RMS (Squared): ");
  Serial.println(baselineNoiseSquared);
}

float readMicrophoneVolume() {
  int32_t raw_samples[256];
  size_t bytes_read;
  i2s_read(I2S_NUM, (void**)raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY);
  
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
  DateTime now = rtc.now();
  // FIXED: Added now.hour() to line up properly with formatting template
  snprintf(filename, sizeof(filename), "/rec_%04d-%02d-%02d_%02d_%02d_%02d.wav", 
          now.year(), now.month(), now.day(), 
          now.hour(), now.minute(), now.second());

  Serial.printf("Creating File: %s\n", filename);

  file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("CRITICAL ERROR: Failed to open file for writing!");
    return;
  }
 
  writeWavHeader(file, SAMPLE_RATE, 16, 1, 0);

  ramBufferIndex = 0; // Reset working RAM allocation buffer
  isRecording = true;
  digitalWrite(LED_PIN, HIGH); 
}

void stopRecording() {
  Serial.println("Stopping Recording, cleaning up buffers...");
  
  // Write any residual data left over in RAM before closing
  flushAudioBuffer();

  unsigned long fileSize = file.size() - WAVE_HEADER_SIZE;
  file.seek(0);
  writeWavHeader(file, SAMPLE_RATE, 16, 1, fileSize);
  file.close();
  
  isRecording = false;
  Serial.println("Recording Complete and Saved!");
  digitalWrite(LED_PIN, LOW); 
}

bool appendAudioToSD() {
    size_t bytesRead;
    int32_t i2sBuffer[256]; 
 
    i2s_read(I2S_NUM, (void *)i2sBuffer, sizeof(i2sBuffer), &bytesRead, portMAX_DELAY);

    int samplesCount = bytesRead / 4;
    if (samplesCount == 0) return false;

    float sum_squares = 0;

    for (int i = 0; i < samplesCount; i++) {
      int16_t sample16 = (int16_t)(i2sBuffer[i] >> 14);
      
      // Accumulate sample into the larger RAM block buffer
      ramAudioBuffer[ramBufferIndex++] = sample16;
      
      // Calculate mean square without square-root operations
      float sampleVal = (float)sample16;
      sum_squares += sampleVal * sampleVal;

      // When RAM buffer fills up entirely, trigger a multi-block SD Flash write
      if (ramBufferIndex >= AUDIO_BUF_SIZE) {
         flushAudioBuffer();
      }
    }
   
    float meanSquare = sum_squares / samplesCount;
    float triggerThreshold = baselineNoiseSquared * soundThresholdMultiplier;

    return (meanSquare > triggerThreshold);
}

// Write accumulated RAM chunks directly to SD
void flushAudioBuffer() {
  if (ramBufferIndex == 0) return;

  size_t bytesToWrite = ramBufferIndex * 2;
  size_t bytesWritten = file.write((uint8_t *)ramAudioBuffer, bytesToWrite);

  if (bytesWritten < bytesToWrite) {
    Serial.println("CRITICAL SD WRITE FAULT!");
    uint64_t freeBytes = SD.totalBytes() - SD.usedBytes();
    if (freeBytes < bytesToWrite) {
        Serial.println("Cause: Card Space Exhausted!");
        digitalWrite(LED_PIN, LOW); 
        digitalWrite(LED_PIN_2, HIGH); // Light error indicator LED
        while (true) { delay(1000); } 
    }
  }
  ramBufferIndex = 0; // Reset index counter
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
  header[20] = 1;  header[21] = 0;  
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