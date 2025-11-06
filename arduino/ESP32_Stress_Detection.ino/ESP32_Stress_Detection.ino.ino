/*
 * ═══════════════════════════════════════════════════════════════════════════
 * ESP32 WESAD Stress Detection System - COMPLETE SERIAL MONITOR VERSION
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * Features:
 * ✓ Complete standalone operation via Serial Monitor
 * ✓ Memory optimized (reduced buffer size)
 * ✓ RGB LED visual feedback
 * ✓ BLE communication for mobile app
 * ✓ Real-time feature display
 * ✓ Interactive menu system
 * ✓ Detailed diagnostics
 * 
 * Commands (type in Serial Monitor):
 * - START: Begin stress detection
 * - STOP: Stop detection
 * - STATUS: Show system status
 * - TEST: Test RGB LED
 * - HELP: Show command menu
 * 
 * RGB Color Codes:
 * 🟢 GREEN   = Baseline (Relaxed)
 * 🔴 RED     = Stress Detected
 * 🔵 BLUE    = Amusement (Happy)
 * 🟡 YELLOW  = Collecting Data
 * 🔵 CYAN    = Processing
 * ⚪ WHITE   = BLE Connected
 * ⚫ OFF     = Idle/Disconnected
 * 🔴 BLINK   = Error (Electrodes)
 * 
 * Hardware Connections:
 * ECG Sensor:  OUTPUT→GPIO36, LO+→GPIO25, LO-→GPIO26, GND→GND, 3.3V→3.3V
 * EDA Sensor:  SIG→GPIO39, GND→GND, VCC→3.3V
 * RGB LED:     R→GPIO23, G→GPIO22, B→GPIO21, GND→GND (HW-478 module)
 * 
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "stress_rf_model.h"

// ═══════════════════════════════════════════════════════════════════════════
// MEMORY-OPTIMIZED CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════
#define SAMPLING_RATE 700           // Hz (WESAD standard)
#define WINDOW_SIZE 10              // seconds (reduced from 30 for memory)
#define BUFFER_SIZE (SAMPLING_RATE * WINDOW_SIZE)  // 7000 samples
#define PROGRESS_INTERVAL 2         // Update every 2 seconds

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════
// Sensors
#define ECG_PIN 36                  // ADC1_CH0
#define EDA_PIN 39                  // ADC1_CH3
#define LO_PLUS 25                  // Lead-off detection +
#define LO_MINUS 26                 // Lead-off detection -

// RGB LED (HW-478 Module)
#define RGB_RED_PIN 23
#define RGB_GREEN_PIN 22
#define RGB_BLUE_PIN 21
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// ═══════════════════════════════════════════════════════════════════════════
// BLE CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define PREDICTION_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define FEATURES_UUID       "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"
#define STATUS_UUID         "d8f8c93e-6e1e-4a7c-b9f3-5e7a8c9d0e1f"
#define CONTROL_UUID        "a3b2c1d4-e5f6-4a5b-9c8d-7e6f5a4b3c2d"

BLEServer* pServer = NULL;
BLECharacteristic* pPredictionChar = NULL;
BLECharacteristic* pFeaturesChar = NULL;
BLECharacteristic* pStatusChar = NULL;
BLECharacteristic* pControlChar = NULL;

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════════════════
// Data buffers (dynamically allocated)
float* ecg_buffer = NULL;
float* eda_buffer = NULL;
int buffer_index = 0;

// Features
float features[12];
const char* feature_names[] = {
  "ECG Mean", "ECG Std", "ECG Min", "ECG Max",
  "HRV MeanNN", "HRV SDNN", "HRV RMSSD", "HRV pNN50",
  "EDA Tonic", "EDA Phasic", "SCR Peaks", "SCR Amplitude"
};

// State variables
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool is_collecting = false;
bool continuous_mode = false;
bool electrode_error = false;
bool system_ready = false;
int current_prediction = -1;
unsigned long last_sample_time = 0;
unsigned long sample_interval = 1428;  // microseconds (1000000/700)
uint32_t total_predictions = 0;
uint32_t stress_count = 0;

// Prediction statistics
const char* class_names[] = {"Baseline", "Stress", "Amusement"};
int prediction_history[100];  // Last 100 predictions
int history_index = 0;

// ═══════════════════════════════════════════════════════════════════════════
// RGB LED FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void setRGB(int r, int g, int b) {
  ledcWrite(RGB_RED_PIN, r);
  ledcWrite(RGB_GREEN_PIN, g);
  ledcWrite(RGB_BLUE_PIN, b);
}

void rgb_off()     { setRGB(0, 0, 0); }
void rgb_red()     { setRGB(255, 0, 0); }
void rgb_green()   { setRGB(0, 255, 0); }
void rgb_blue()    { setRGB(0, 0, 255); }
void rgb_yellow()  { setRGB(255, 255, 0); }
void rgb_cyan()    { setRGB(0, 255, 255); }
void rgb_white()   { setRGB(255, 255, 255); }
void rgb_magenta() { setRGB(255, 0, 255); }

void rgb_blink(int r, int g, int b, int times, int ms) {
  for (int i = 0; i < times; i++) {
    setRGB(r, g, b);
    delay(ms);
    rgb_off();
    delay(ms);
  }
}

void rgb_pulse(int r, int g, int b, int duration) {
  for (int i = 0; i < 256; i += 5) {
    setRGB((r * i) / 255, (g * i) / 255, (b * i) / 255);
    delay(duration / 51);
  }
  for (int i = 255; i >= 0; i -= 5) {
    setRGB((r * i) / 255, (g * i) / 255, (b * i) / 255);
    delay(duration / 51);
  }
}

void setPredictionColor(int prediction) {
  switch (prediction) {
    case 0: rgb_green(); break;   // Baseline
    case 1: rgb_red(); break;     // Stress
    case 2: rgb_blue(); break;    // Amusement
    default: rgb_off();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// BLE CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════

class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║   📱 BLE CLIENT CONNECTED            ║");
    Serial.println("╚═══════════════════════════════════════╝");
    rgb_pulse(255, 255, 255, 500);
    rgb_white();
  }
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║   📱 BLE CLIENT DISCONNECTED         ║");
    Serial.println("╚═══════════════════════════════════════╝");
    rgb_off();
    delay(500);
    BLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising restarted");
  }
};

class ControlCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    String cmd = String(pChar->getValue().c_str());
    cmd.trim();
    cmd.toUpperCase();
    Serial.printf("\n[BLE] 📲 Command received: %s\n", cmd.c_str());
    handleCommand(cmd);
  }
};

// ═══════════════════════════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void printLine(char c = '═', int length = 75) {
  for (int i = 0; i < length; i++) Serial.print(c);
  Serial.println();
}

void printBox(const char* text, char border = '║') {
  int len = strlen(text);
  int padding = (73 - len) / 2;
  Serial.print(border);
  for (int i = 0; i < padding; i++) Serial.print(' ');
  Serial.print(text);
  for (int i = 0; i < 73 - padding - len; i++) Serial.print(' ');
  Serial.println(border);
}

void showMenu() {
  Serial.println("\n╔═══════════════════════════════════════════════════════════════════════╗");
  printBox("📋 COMMAND MENU");
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  Serial.println("║  START     - Begin stress detection (10 seconds collection)          ║");
  Serial.println("║  STOP      - Stop continuous detection                               ║");
  Serial.println("║  CONTINUOUS- Start continuous detection (auto-restart)               ║");
  Serial.println("║  STATUS    - Show system status and statistics                       ║");
  Serial.println("║  TEST      - Test RGB LED (all colors)                               ║");
  Serial.println("║  HISTORY   - Show prediction history                                 ║");
  Serial.println("║  RESET     - Reset statistics                                        ║");
  Serial.println("║  HELP      - Show this menu                                          ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
}

void showStatus() {
  Serial.println("\n╔═══════════════════════════════════════════════════════════════════════╗");
  printBox("📊 SYSTEM STATUS");
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  Serial.printf("║ System Ready:        %-50s║\n", system_ready ? "✓ YES" : "✗ NO");
  Serial.printf("║ BLE Connected:       %-50s║\n", deviceConnected ? "✓ YES" : "✗ NO");
  Serial.printf("║ Collecting Data:     %-50s║\n", is_collecting ? "✓ YES" : "✗ NO");
  Serial.printf("║ Continuous Mode:     %-50s║\n", continuous_mode ? "✓ YES" : "✗ NO");
  Serial.printf("║ Buffer Progress:     %d/%d samples (%.1f%%)%22s║\n", 
                buffer_index, BUFFER_SIZE, (buffer_index * 100.0) / BUFFER_SIZE, "");
  Serial.printf("║ Free Heap:           %d bytes%40s║\n", ESP.getFreeHeap(), "");
  Serial.printf("║ Total Predictions:   %d%48s║\n", total_predictions, "");
  Serial.printf("║ Stress Detections:   %d%48s║\n", stress_count, "");
  if (total_predictions > 0) {
    Serial.printf("║ Stress Rate:         %.1f%%%49s║\n", 
                  (stress_count * 100.0) / total_predictions, "");
  }
  Serial.printf("║ Last Prediction:     %-50s║\n", 
                current_prediction >= 0 ? class_names[current_prediction] : "None");
  Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
}

void showHistory() {
  if (total_predictions == 0) {
    Serial.println("\n[INFO] No predictions yet. Run START to begin detection.\n");
    return;
  }
  
  Serial.println("\n╔═══════════════════════════════════════════════════════════════════════╗");
  printBox("📈 PREDICTION HISTORY (Last 20)");
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  
  int start = history_index - 20;
  if (start < 0) start = 0;
  int count = 0;
  
  for (int i = start; i < history_index && count < 20; i++) {
    const char* label = class_names[prediction_history[i]];
    const char* emoji = prediction_history[i] == 0 ? "🟢" : 
                       prediction_history[i] == 1 ? "🔴" : "🔵";
    Serial.printf("║  #%02d: %s %-60s║\n", i + 1, emoji, label);
    count++;
  }
  
  Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
}

void testRGB() {
  Serial.println("\n╔═══════════════════════════════════════════════════════════════════════╗");
  printBox("🎨 RGB LED TEST");
  Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
  
  Serial.println("[TEST] 🔴 Red..."); rgb_red(); delay(1000);
  Serial.println("[TEST] 🟢 Green..."); rgb_green(); delay(1000);
  Serial.println("[TEST] 🔵 Blue..."); rgb_blue(); delay(1000);
  Serial.println("[TEST] 🟡 Yellow..."); rgb_yellow(); delay(1000);
  Serial.println("[TEST] 🔵 Cyan..."); rgb_cyan(); delay(1000);
  Serial.println("[TEST] 🟣 Magenta..."); rgb_magenta(); delay(1000);
  Serial.println("[TEST] ⚪ White..."); rgb_white(); delay(1000);
  Serial.println("[TEST] ⚫ Off..."); rgb_off(); delay(500);
  Serial.println("[TEST] ✓ RGB LED test complete!\n");
}

void handleCommand(String cmd) {
  if (cmd == "START") {
    continuous_mode = false;
    is_collecting = true;
    buffer_index = 0;
    Serial.println("\n[CMD] ▶️  Starting single detection cycle...");
    rgb_yellow();
  }
  else if (cmd == "CONTINUOUS") {
    continuous_mode = true;
    is_collecting = true;
    buffer_index = 0;
    Serial.println("\n[CMD] 🔁 Starting continuous detection...");
    rgb_yellow();
  }
  else if (cmd == "STOP") {
    continuous_mode = false;
    is_collecting = false;
    Serial.println("\n[CMD] ⏹️  Detection stopped");
    rgb_off();
  }
  else if (cmd == "STATUS") {
    showStatus();
  }
  else if (cmd == "TEST") {
    testRGB();
  }
  else if (cmd == "HISTORY") {
    showHistory();
  }
  else if (cmd == "RESET") {
    total_predictions = 0;
    stress_count = 0;
    history_index = 0;
    current_prediction = -1;
    Serial.println("\n[CMD] 🔄 Statistics reset\n");
  }
  else if (cmd == "HELP") {
    showMenu();
  }
  else {
    Serial.printf("\n[ERROR] ❌ Unknown command: %s\n", cmd.c_str());
    Serial.println("[INFO] Type HELP for command list\n");
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// MEMORY ALLOCATION
// ═══════════════════════════════════════════════════════════════════════════

bool allocateBuffers() {
  Serial.println("\n╔═══════════════════════════════════════════════════════════════════════╗");
  printBox("💾 MEMORY ALLOCATION");
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  Serial.printf("║ Buffer size:         %d samples%39s║\n", BUFFER_SIZE, "");
  Serial.printf("║ Memory per buffer:   %d bytes%41s║\n", BUFFER_SIZE * 4, "");
  Serial.printf("║ Total needed:        %d bytes%40s║\n", BUFFER_SIZE * 8, "");
  Serial.printf("║ Free heap before:    %d bytes%40s║\n", ESP.getFreeHeap(), "");
  Serial.printf("║ PSRAM available:     %-50s║\n", psramFound() ? "✓ YES" : "✗ NO");
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  
  // Allocate ECG buffer
  ecg_buffer = (float*)malloc(BUFFER_SIZE * sizeof(float));
  if (ecg_buffer == NULL) {
    Serial.println("║ ECG Buffer:          ✗ ALLOCATION FAILED!                            ║");
    Serial.println("╚═══════════════════════════════════════════════════════════════════════╝");
    return false;
  }
  Serial.println("║ ECG Buffer:          ✓ Allocated successfully                        ║");
  
  // Allocate EDA buffer
  eda_buffer = (float*)malloc(BUFFER_SIZE * sizeof(float));
  if (eda_buffer == NULL) {
    Serial.println("║ EDA Buffer:          ✗ ALLOCATION FAILED!                            ║");
    free(ecg_buffer);
    Serial.println("╚═══════════════════════════════════════════════════════════════════════╝");
    return false;
  }
  Serial.println("║ EDA Buffer:          ✓ Allocated successfully                        ║");
  
  // Initialize to zero
  memset(ecg_buffer, 0, BUFFER_SIZE * sizeof(float));
  memset(eda_buffer, 0, BUFFER_SIZE * sizeof(float));
  
  Serial.printf("║ Free heap after:     %d bytes%40s║\n", ESP.getFreeHeap(), "");
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  Serial.println("║                    ✓ MEMORY ALLOCATION SUCCESS                        ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
  
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Clear screen
  Serial.println("\n\n\n\n\n");
  
  // Startup banner
  printLine();
  printBox("ESP32 WESAD STRESS DETECTION SYSTEM v3.3");
  printBox("TinyML + BLE + RGB Visual Feedback");
  printLine();
  
  // System info
  Serial.println("\n╔═══════════════════════════════════════════════════════════════════════╗");
  printBox("🖥️  SYSTEM INFORMATION");
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  Serial.printf("║ Chip:                %-50s║\n", ESP.getChipModel());
  Serial.printf("║ CPU Frequency:       %d MHz%43s║\n", ESP.getCpuFreqMHz(), "");
  Serial.printf("║ Flash Size:          %d MB%45s║\n", ESP.getFlashChipSize() / 1048576, "");
  Serial.printf("║ Free Heap:           %d bytes%40s║\n", ESP.getFreeHeap(), "");
  Serial.printf("║ PSRAM:               %-50s║\n", psramFound() ? "Available" : "Not Available");
  Serial.printf("║ Sampling Rate:       %d Hz%45s║\n", SAMPLING_RATE, "");
  Serial.printf("║ Window Size:         %d seconds%40s║\n", WINDOW_SIZE, "");
  Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
  
  // Initialize RGB
  Serial.println("[INIT] Initializing RGB LED...");
  ledcAttach(RGB_RED_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RGB_GREEN_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RGB_BLUE_PIN, PWM_FREQ, PWM_RESOLUTION);
  rgb_off();
  Serial.println("[INIT] ✓ RGB LED ready");
  
  // Startup light show
  Serial.println("[INIT] Running RGB startup sequence...");
  rgb_blink(255, 0, 0, 2, 150);
  delay(100);
  rgb_blink(0, 255, 0, 2, 150);
  delay(100);
  rgb_blink(0, 0, 255, 2, 150);
  Serial.println("[INIT] ✓ RGB test complete");
  
  // Initialize pins
  Serial.println("[INIT] Configuring sensor pins...");
  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MINUS, INPUT);
  pinMode(ECG_PIN, INPUT);
  pinMode(EDA_PIN, INPUT);
  Serial.println("[INIT] ✓ Sensor pins configured");
  
  // Allocate memory
  if (!allocateBuffers()) {
    Serial.println("\n╔═══════════════════════════════════════════════════════════════════════╗");
    printBox("❌ CRITICAL ERROR: MEMORY ALLOCATION FAILED");
    Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
    Serial.println("║  The system cannot allocate memory for data buffers.                 ║");
    Serial.println("║                                                                       ║");
    Serial.println("║  Solutions:                                                           ║");
    Serial.println("║  1. Enable PSRAM: Tools > PSRAM > 'Enabled'                          ║");
    Serial.println("║  2. Use ESP32 with more RAM                                           ║");
    Serial.println("║  3. Reduce WINDOW_SIZE in code                                        ║");
    Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
    
    while (1) {
      rgb_blink(255, 0, 0, 3, 300);
      Serial.println("[ERROR] System halted - Memory allocation failed");
      delay(3000);
    }
  }
  
  // Initialize BLE
  Serial.println("[INIT] Initializing BLE...");
  BLEDevice::init("ESP32_StressMonitor");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pPredictionChar = pService->createCharacteristic(PREDICTION_UUID, 
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pPredictionChar->addDescriptor(new BLE2902());
  
  pFeaturesChar = pService->createCharacteristic(FEATURES_UUID,
    BLECharacteristic::PROPERTY_READ);
  
  pStatusChar = pService->createCharacteristic(STATUS_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pStatusChar->addDescriptor(new BLE2902());
  
  pControlChar = pService->createCharacteristic(CONTROL_UUID,
    BLECharacteristic::PROPERTY_WRITE);
  pControlChar->setCallbacks(new ControlCallbacks());
  
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("[INIT] ✓ BLE service started (ESP32_StressMonitor)");
  
  system_ready = true;
  rgb_pulse(0, 255, 0, 1000);
  
  // Ready message
  Serial.println("\n╔═══════════════════════════════════════════════════════════════════════╗");
  printBox("✅ SYSTEM READY");
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  Serial.println("║  Type HELP to see available commands                                  ║");
  Serial.println("║  Type START to begin stress detection                                 ║");
  Serial.println("║  Type TEST to test RGB LED                                            ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
  
  showMenu();
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    if (cmd.length() > 0) {
      handleCommand(cmd);
    }
  }
  
  // Handle BLE connection changes
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = false;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = true;
  }
  
  // Handle electrode errors
  if (electrode_error) {
    rgb_blink(255, 0, 0, 1, 200);
    electrode_error = false;
  }
  
  // Data collection
  if (is_collecting) {
    if (collectSensorData()) {
      // Buffer full - process prediction
      processAndPredict();
      
      // Handle modes
      if (continuous_mode) {
        buffer_index = 0;
        delay(2000);  // Show result for 2 seconds
        rgb_yellow();
        Serial.println("\n[INFO] 🔁 Restarting collection (continuous mode)...\n");
      } else {
        is_collecting = false;
        rgb_off();
        Serial.println("\n[INFO] ⏹️  Single cycle complete. Type START to run again.\n");
      }
    }
  }
  
  delay(1);
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR DATA COLLECTION
// ═══════════════════════════════════════════════════════════════════════════

bool collectSensorData() {
  unsigned long current_time = micros();
  
  if (current_time - last_sample_time >= sample_interval) {
    last_sample_time = current_time;
    
    // Check electrodes
    if (digitalRead(LO_PLUS) == 1 || digitalRead(LO_MINUS) == 1) {
      if (buffer_index % (SAMPLING_RATE * 3) == 0) {
        Serial.println("\n[SENSOR] ⚠️  WARNING: Electrodes disconnected!");
        Serial.println("[SENSOR] Please check ECG electrode connections\n");
        electrode_error = true;
      }
      return false;
    }
    
    // Read sensors
    ecg_buffer[buffer_index] = analogRead(ECG_PIN) * (3.3 / 4095.0);
    eda_buffer[buffer_index] = analogRead(EDA_PIN) * (3.3 / 4095.0);
    buffer_index++;
    
    // Progress updates
    if (buffer_index % (SAMPLING_RATE * PROGRESS_INTERVAL) == 0) {
      int seconds = buffer_index / SAMPLING_RATE;
      float progress = (seconds * 100.0) / WINDOW_SIZE;
      Serial.printf("[COLLECT] ⏱️  %d/%d seconds (%.0f%%) | Samples: %d\n", 
                    seconds, WINDOW_SIZE, progress, buffer_index);
      
      // Send BLE status update
      if (deviceConnected) {
        String status = "Collecting," + String((int)progress) + "," + String(millis());
        pStatusChar->setValue(status.c_str());
        pStatusChar->notify();
      }
    }
    
    // Check if buffer full
    if (buffer_index >= BUFFER_SIZE) {
      Serial.println("\n[COLLECT] ✓ Data collection complete!");
      return true;
    }
  }
  
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// FEATURE EXTRACTION & PREDICTION
// ═══════════════════════════════════════════════════════════════════════════

float calculate_mean(float* data, int len) {
  float sum = 0;
  for (int i = 0; i < len; i++) sum += data[i];
  return sum / len;
}

float calculate_std(float* data, int len, float mean) {
  float sum_sq = 0;
  for (int i = 0; i < len; i++) {
    float diff = data[i] - mean;
    sum_sq += diff * diff;
  }
  return sqrt(sum_sq / len);
}

float find_min(float* data, int len) {
  float min_val = data[0];
  for (int i = 1; i < len; i++) {
    if (data[i] < min_val) min_val = data[i];
  }
  return min_val;
}

float find_max(float* data, int len) {
  float max_val = data[0];
  for (int i = 1; i < len; i++) {
    if (data[i] > max_val) max_val = data[i];
  }
  return max_val;
}

int detect_r_peaks(float* ecg, int len, float* rr_intervals, int max_peaks) {
  float threshold = 1.5;
  int n_peaks = 0;
  int last_peak = 0;
  
  for (int i = 50; i < len - 50 && n_peaks < max_peaks; i++) {
    if (ecg[i] > threshold && 
        ecg[i] > ecg[i-1] && ecg[i] > ecg[i+1] &&
        (i - last_peak) > 200) {
      if (n_peaks > 0) {
        rr_intervals[n_peaks - 1] = (i - last_peak) / 700.0 * 1000.0;
      }
      last_peak = i;
      n_peaks++;
    }
  }
  return n_peaks > 0 ? n_peaks - 1 : 0;
}

float calculate_rmssd(float* rr, int len) {
  if (len <= 1) return 0;
  float sum_sq = 0;
  for (int i = 1; i < len; i++) {
    float diff = rr[i] - rr[i-1];
    sum_sq += diff * diff;
  }
  return sqrt(sum_sq / (len - 1));
}

float calculate_pnn50(float* rr, int len) {
  if (len <= 1) return 0;
  int count = 0;
  for (int i = 1; i < len; i++) {
    if (abs(rr[i] - rr[i-1]) > 50) count++;
  }
  return (count / (float)(len - 1)) * 100.0;
}

int detect_scr_peaks(float* eda, int len) {
  float threshold = 0.05;
  int peaks = 0;
  for (int i = 100; i < len - 100; i++) {
    float slope = eda[i] - eda[i-50];
    if (slope > threshold) {
      peaks++;
      i += 500;
    }
  }
  return peaks;
}

void extractFeatures() {
  Serial.println("\n╔═══════════════════════════════════════════════════════════════════════╗");
  printBox("🔬 FEATURE EXTRACTION");
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  
  // ECG Features
  Serial.println("║ Extracting ECG features...                                           ║");
  features[0] = calculate_mean(ecg_buffer, BUFFER_SIZE);
  features[1] = calculate_std(ecg_buffer, BUFFER_SIZE, features[0]);
  features[2] = find_min(ecg_buffer, BUFFER_SIZE);
  features[3] = find_max(ecg_buffer, BUFFER_SIZE);
  Serial.println("║ ✓ ECG statistical features extracted                                 ║");
  
  // HRV Features
  Serial.println("║ Detecting R-peaks for HRV...                                         ║");
  float* rr_intervals = (float*)malloc(500 * sizeof(float));
  if (rr_intervals != NULL) {
    int n_peaks = detect_r_peaks(ecg_buffer, BUFFER_SIZE, rr_intervals, 500);
    Serial.printf("║ ✓ Detected %d R-peaks%52s║\n", n_peaks, "");
    
    if (n_peaks > 2) {
      features[4] = calculate_mean(rr_intervals, n_peaks);
      features[5] = calculate_std(rr_intervals, n_peaks, features[4]);
      features[6] = calculate_rmssd(rr_intervals, n_peaks);
      features[7] = calculate_pnn50(rr_intervals, n_peaks);
      Serial.println("║ ✓ HRV features calculated                                            ║");
    } else {
      features[4] = features[5] = features[6] = features[7] = 0;
      Serial.println("║ ⚠️  Insufficient peaks for HRV (using defaults)                      ║");
    }
    free(rr_intervals);
  } else {
    features[4] = features[5] = features[6] = features[7] = 0;
    Serial.println("║ ⚠️  Memory allocation failed for HRV (using defaults)                ║");
  }
  
  // EDA Features
  Serial.println("║ Extracting EDA features...                                           ║");
  features[8] = calculate_mean(eda_buffer, BUFFER_SIZE);
  features[9] = calculate_std(eda_buffer, BUFFER_SIZE, features[8]);
  features[10] = detect_scr_peaks(eda_buffer, BUFFER_SIZE);
  features[11] = features[8];
  Serial.println("║ ✓ EDA features extracted                                             ║");
  
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  Serial.println("║                    ✓ FEATURE EXTRACTION COMPLETE                      ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
}

void displayFeatures() {
  Serial.println("╔═══════════════════════════════════════════════════════════════════════╗");
  printBox("📊 EXTRACTED FEATURES");
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  
  // ECG Features
  Serial.println("║ ECG Features:                                                         ║");
  Serial.printf("║   • Mean:              %8.4f V%37s║\n", features[0], "");
  Serial.printf("║   • Std Dev:           %8.4f V%37s║\n", features[1], "");
  Serial.printf("║   • Min:               %8.4f V%37s║\n", features[2], "");
  Serial.printf("║   • Max:               %8.4f V%37s║\n", features[3], "");
  
  // HRV Features
  Serial.println("║                                                                       ║");
  Serial.println("║ Heart Rate Variability (HRV):                                        ║");
  Serial.printf("║   • Mean NN:           %8.2f ms%36s║\n", features[4], "");
  Serial.printf("║   • SDNN:              %8.2f ms%36s║\n", features[5], "");
  Serial.printf("║   • RMSSD:             %8.2f ms%36s║\n", features[6], "");
  Serial.printf("║   • pNN50:             %8.2f %%%37s║\n", features[7], "");
  
  // EDA Features
  Serial.println("║                                                                       ║");
  Serial.println("║ Electrodermal Activity (EDA):                                        ║");
  Serial.printf("║   • Tonic Level:       %8.4f V%37s║\n", features[8], "");
  Serial.printf("║   • Phasic Std:        %8.4f V%37s║\n", features[9], "");
  Serial.printf("║   • SCR Peaks:         %8.0f peaks%34s║\n", features[10], "");
  Serial.printf("║   • SCR Amplitude:     %8.4f V%37s║\n", features[11], "");
  
  Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
}

void processAndPredict() {
  rgb_cyan();
  
  // Extract features
  extractFeatures();
  
  // Display features
  displayFeatures();
  
  // Run ML model
  Serial.println("╔═══════════════════════════════════════════════════════════════════════╗");
  printBox("🤖 MACHINE LEARNING PREDICTION");
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  Serial.println("║ Running Random Forest model (100 trees, 97.4% accuracy)...           ║");
  
  int32_t prediction = predict_stress(features);
  current_prediction = prediction;
  total_predictions++;
  if (prediction == 1) stress_count++;
  
  // Store in history
  if (history_index < 100) {
    prediction_history[history_index++] = prediction;
  } else {
    // Shift history
    for (int i = 0; i < 99; i++) {
      prediction_history[i] = prediction_history[i + 1];
    }
    prediction_history[99] = prediction;
  }
  
  Serial.println("║ ✓ Prediction complete                                                ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
  
  // Display result
  const char* result_emoji = prediction == 0 ? "🟢" : prediction == 1 ? "🔴" : "🔵";
  const char* result_label = class_names[prediction];
  
  Serial.println("╔═══════════════════════════════════════════════════════════════════════╗");
  Serial.printf("║                 %s  PREDICTION: %-13s                     ║\n", 
                result_emoji, result_label);
  Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
  Serial.printf("║ Class:                  %d (%s)%31s║\n", 
                prediction, result_label, "");
  Serial.printf("║ Total Predictions:      %d%46s║\n", total_predictions, "");
  Serial.printf("║ Stress Detections:      %d%46s║\n", stress_count, "");
  
  if (prediction == 0) {
    Serial.println("║                                                                       ║");
    Serial.println("║ 💚 Status: Normal/Relaxed State                                      ║");
    Serial.println("║ 📝 Note: Person is in a baseline emotional state                     ║");
  } else if (prediction == 1) {
    Serial.println("║                                                                       ║");
    Serial.println("║ ⚠️  Status: STRESS DETECTED                                           ║");
    Serial.println("║ 📝 Recommendations:                                                   ║");
    Serial.println("║    • Take deep breaths                                                ║");
    Serial.println("║    • Consider a short break                                           ║");
    Serial.println("║    • Practice relaxation techniques                                   ║");
  } else if (prediction == 2) {
    Serial.println("║                                                                       ║");
    Serial.println("║ 😊 Status: Positive/Amusement State                                  ║");
    Serial.println("║ 📝 Note: Person is experiencing positive emotions                    ║");
  }
  
  Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
  
  // Set RGB color
  setPredictionColor(prediction);
  
  // Send BLE notification
  if (deviceConnected) {
    String payload = String(prediction) + ",100.0,";
    for (int i = 0; i < 12; i++) {
      payload += String(features[i], 4);
      if (i < 11) payload += ",";
    }
    payload += "," + String(millis());
    
    pPredictionChar->setValue(payload.c_str());
    pPredictionChar->notify();
    
    Serial.println("[BLE] 📤 Prediction sent to mobile app\n");
  }
}