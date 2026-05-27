#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>
#include <FS.h>

//#define GAIN_FACTOR 2.5

#define I2S_BCK_PIN 4
#define I2S_WS_PIN 5
#define I2S_SD_PIN 6

#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 13
#define SD_CLK_PIN 12 //also know as SCK pin 

//#define BUTTON_PIN 8
#define LED_PIN 3

#define SAMPLE_RATE 48000  // DVD quality
#define I2S_NUM I2S_NUM_0
#define WAVE_HEADER_SIZE 44
#define MY_CLOCK 4000000

// Recording constraints
const int recordingTimeLimit = 30000; // 30 seconds limit
const float soundThresholdMultiplier = 1.4; // Starts recording if 1.5x louder than quiet
const int silenceTimeout = 5000; // Stop if silent for 5 seconds

File file;
bool isRecording = false;
unsigned long lastBlinkTime = 0;
unsigned long recordingStartTime = 0;
unsigned long lastSoundTime = 0;
unsigned long baselineNoise = 0;
int fileIndex = 1;
char filename[32];
 
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

void setup() {
  Serial.begin(9600);
  //pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  SPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card not Initialize!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }
  Serial.println("SD Card Ready.");
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM);
  calibrateNoiseFloor();
  Serial.println("System is Ready. Click the utton for Recording.");
}

void loop() {  
  // 1. If NOT recording, we sample the microphone to look for the trigger sound
  if (!isRecording) {
    float currentVolume = readMicrophoneVolume();
    
    if (currentVolume > (baselineNoise * soundThresholdMultiplier)) {
      Serial.println("Start Recording!");
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
    // FIX: Downsample to 16-bit exactly like you do during recording
    int16_t sample16 = (int16_t)(raw_samples[i] >> 14); 
    float sample = (float)sample16;
    sum_squares += sample * sample;
  }
  return sqrt(sum_squares / samples_count);
}

void startRecording() { 
  while (SD.exists("/rec" + String(fileIndex) + ".wav")) {
    fileIndex++;
  }
  sprintf(filename, "/rec%d.wav", fileIndex);

  Serial.print("Recording has started: ");
  Serial.println(filename);

  file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("File is not Open!");
    return;
  }
 
  writeWavHeader(file, SAMPLE_RATE, 16, 1, 0);

  isRecording = true;
  digitalWrite(LED_PIN, HIGH); // Turn LED on when starting
}

void stopRecording() {
  Serial.println("Recording is Stopped...");
  isRecording = false;
  unsigned long fileSize = file.size() - WAVE_HEADER_SIZE;
  file.seek(0);
  writeWavHeader(file, SAMPLE_RATE, 16, 1, fileSize);
  file.close();
  Serial.println("Recording is Complete!");
  digitalWrite(LED_PIN, LOW); // Turn LED off when done
}

// Fixed function: Writes to SD AND checks volume at the same time
bool appendAudioToSD() {
    size_t bytesRead;
    int32_t i2sBuffer[256]; 
 
    // FIXED: Changed I2S_NUM to I2S_NUM_0
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
    file.write((uint8_t *)samples16, samplesCount * 2);

    // Calculate current RMS volume of this chunk
    float currentVolume = sqrt(sum_squares / samplesCount);

    // Return true if volume is above the threshold
    return (currentVolume > (baselineNoise * soundThresholdMultiplier));
}