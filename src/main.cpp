/**
 * @file main.cpp
 * @brief ESP32 BLE to UART Bridge for Autopilot System
 * 
 * This application acts as a bridge between an iOS device (via BLE) and a 
 * Teensy 4.0 microcontroller (via UART). It enables remote monitoring and 
 * control of an autopilot system by forwarding telemetry data and PID parameters.
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ============================================================================
// Constants and Configuration
// ============================================================================

// BLE Service and Characteristic UUIDs
constexpr const char* SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
constexpr const char* AUTOPILOT_CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
constexpr const char* PID_CHARACTERISTIC_UUID = "98ab29d2-2b95-497d-9df7-f064e5ac05a5";

// Hardware Configuration
constexpr uint8_t UART_RX_PIN = 20;
constexpr uint8_t UART_TX_PIN = 21;
constexpr uint32_t SERIAL_BAUD_RATE = 9600;
constexpr uint32_t DEBUG_BAUD_RATE = 115200;

// Communication Protocol
constexpr uint8_t PACKET_START_MARKER = 0x02;
constexpr uint8_t PACKET_END_MARKER = 0x03;

// BLE Configuration
constexpr const char* DEVICE_NAME = "Autopilot";
constexpr uint16_t BLE_MTU_SIZE = 80;

// Timing Configuration
constexpr uint32_t LOOP_DELAY_MS = 10;  // Reduced from 500ms for better responsiveness
constexpr uint32_t MAX_PACKET_TIMEOUT_MS = 1000;

// FreeRTOS Configuration
constexpr uint32_t UART_TASK_STACK_SIZE = 4096;
constexpr uint32_t BLE_TASK_STACK_SIZE = 4096;
constexpr uint32_t UART_TASK_PRIORITY = 2;
constexpr uint32_t BLE_TASK_PRIORITY = 1;
constexpr uint32_t QUEUE_SIZE = 10;
constexpr uint32_t QUEUE_WAIT_MS = 10;
constexpr size_t MAX_BLE_DATA_SIZE = 512;  // Maximum size for BLE data

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief Structure for BLE data queue items
 */
typedef struct {
  uint8_t data[MAX_BLE_DATA_SIZE];
  size_t length;
} BleDataQueueItem;

/**
 * @brief Telemetry data structure for autopilot communication
 * 
 * This packed structure must match exactly between ESP32 and Teensy.
 * Any changes require updating both devices.
 */
typedef struct __attribute__((packed)) {
  // PID Controller Parameters
  float kp;
  float ki;
  float kd;
  float heading_gain;
  
  // Navigation Data
  float bearingToWaypoint;
  float destinationLatitude;
  float destinationLongitude;
  float previousDestinationLatitude;
  float previousDestinationLongitude;
  float heading;
  
  // Cross Track Error
  float xte;
  float filteredHeading;
  float filteredXTE;
  float previousXte;
  float previousTime;
  float previousBearing;
  float integralXTE;
  float derivativeXTE;
  float timeDelta;
  
  // Control Outputs
  float rudderAngle;
  float rudderPosition;
  float targetMotorPosition;
  float headingCorrection;
  
  // Status
  bool homingComplete;
} AutopilotTelemetry;

// ============================================================================
// Global Variables
// ============================================================================

// BLE Objects
static BLECharacteristic* g_autopilotCharacteristic = nullptr;
static BLECharacteristic* g_pidCharacteristic = nullptr;
static BLEAdvertising* g_advertising = nullptr;
static BLEServer* g_bleServer = nullptr;
static ServerCallbacks* g_serverCallbacks = nullptr;
static CharacteristicCallbacks* g_characteristicCallbacks = nullptr;

// Connection State
static volatile bool g_deviceConnected = false;
static volatile bool g_previouslyConnected = false;

// FreeRTOS Objects
static QueueHandle_t g_teensyToBleQueue = nullptr;
static QueueHandle_t g_bleToTeensyQueue = nullptr;
static SemaphoreHandle_t g_bleDataMutex = nullptr;

// Serial Communication Buffer
static uint8_t g_serialBuffer[sizeof(AutopilotTelemetry)];
static size_t g_bufferIndex = 0;
static bool g_packetInProgress = false;
static uint32_t g_packetStartTime = 0;

// Compile-time assertion to ensure buffer size matches telemetry structure
static_assert(sizeof(g_serialBuffer) == sizeof(AutopilotTelemetry), 
              "Serial buffer size must match AutopilotTelemetry structure size");

// ============================================================================
// BLE Callbacks
// ============================================================================

/**
 * @brief BLE Server connection callbacks
 */
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("[BLE] Device connected");
    g_deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) override {
    Serial.println("[BLE] Device disconnected");
    g_deviceConnected = false;
  }
};

/**
 * @brief BLE Characteristic write callbacks
 */
class CharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    std::string data = pCharacteristic->getValue();
    if (data.length() > 0 && data.length() <= MAX_BLE_DATA_SIZE && g_bleToTeensyQueue) {
      // Prepare queue item
      BleDataQueueItem queueItem;
      memcpy(queueItem.data, data.c_str(), data.length());
      queueItem.length = data.length();
      
      // Send data to queue for UART task to process
      if (xQueueSend(g_bleToTeensyQueue, &queueItem, 0) == pdTRUE) {
        Serial.println("[BLE] Data received from client and queued");
      } else {
        Serial.println("[BLE] Warning: Queue full, data dropped");
      }
    } else if (data.length() > MAX_BLE_DATA_SIZE) {
      Serial.printf("[BLE] Error: Data too large (%d bytes, max %d)\n", 
                    data.length(), MAX_BLE_DATA_SIZE);
    }
  }
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Log telemetry data in a formatted way
 * @param data Pointer to telemetry data structure
 */
void logTelemetryData(const AutopilotTelemetry* data) {
  if (!data) return;
  
  Serial.println("\n========== Telemetry Data ==========");
  
  Serial.println("-- PID Parameters --");
  Serial.printf("  kp: %.4f, ki: %.4f, kd: %.4f\n", data->kp, data->ki, data->kd);
  
  Serial.println("\n-- Navigation --");
  Serial.printf("  Destination: %.6f, %.6f\n", data->destinationLatitude, data->destinationLongitude);
  Serial.printf("  Bearing to Waypoint: %.2f°\n", data->bearingToWaypoint);
  Serial.printf("  Heading: %.2f° (Filtered: %.2f°)\n", data->heading, data->filteredHeading);
  Serial.printf("  Heading Gain: %.4f\n", data->heading_gain);
  
  Serial.println("\n-- Cross Track Error --");
  Serial.printf("  XTE: %.2f (Filtered: %.2f)\n", data->xte, data->filteredXTE);
  Serial.printf("  Previous XTE: %.2f\n", data->previousXte);
  Serial.printf("  Integral XTE: %.4f, Derivative XTE: %.4f\n", data->integralXTE, data->derivativeXTE);
  
  Serial.println("\n-- Control Output --");
  Serial.printf("  Rudder Angle: %.2f°, Position: %.2f\n", data->rudderAngle, data->rudderPosition);
  Serial.printf("  Target Motor Position: %.2f\n", data->targetMotorPosition);
  Serial.printf("  Heading Correction: %.2f°\n", data->headingCorrection);
  
  Serial.println("\n-- Status --");
  Serial.printf("  Homing Complete: %s\n", data->homingComplete ? "Yes" : "No");
  Serial.printf("  Time Delta: %.3fs\n", data->timeDelta);
  
  Serial.println("====================================\n");
}

/**
 * @brief Clean up dynamically allocated BLE objects
 */
void cleanupBLE() {
  if (g_serverCallbacks) {
    delete g_serverCallbacks;
    g_serverCallbacks = nullptr;
  }
  if (g_characteristicCallbacks) {
    delete g_characteristicCallbacks;
    g_characteristicCallbacks = nullptr;
  }
}

/**
 * @brief Initialize serial communication
 */
void initializeSerial() {
  Serial.begin(DEBUG_BAUD_RATE);
  while (!Serial && millis() < 3000) {
    delay(10);  // Wait for serial port to connect (max 3 seconds)
  }
  
  Serial1.begin(SERIAL_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  
  Serial.println("\n[SYSTEM] ESP32 BLE-UART Bridge Starting...");
  Serial.printf("[SYSTEM] Debug Port: %d baud\n", DEBUG_BAUD_RATE);
  Serial.printf("[SYSTEM] Teensy Port: %d baud (RX:%d, TX:%d)\n", 
                SERIAL_BAUD_RATE, UART_RX_PIN, UART_TX_PIN);
}

/**
 * @brief Initialize BLE server and services
 * @return true if initialization successful, false otherwise
 */
bool initializeBLE() {
  Serial.println("[BLE] Initializing BLE...");
  
  // Initialize BLE Device
  BLEDevice::init(DEVICE_NAME);
  
  // Set MTU for larger data packets
  if (!BLEDevice::setMTU(BLE_MTU_SIZE)) {
    Serial.println("[BLE] Warning: Failed to set MTU size");
  }
  
  // Create BLE Server
  g_bleServer = BLEDevice::createServer();
  if (!g_bleServer) {
    Serial.println("[BLE] Error: Failed to create BLE server");
    return false;
  }
  g_serverCallbacks = new ServerCallbacks();
  g_bleServer->setCallbacks(g_serverCallbacks);
  
  // Create BLE Service
  BLEService* pService = g_bleServer->createService(SERVICE_UUID);
  if (!pService) {
    Serial.println("[BLE] Error: Failed to create BLE service");
    return false;
  }
  
  // Create Autopilot Telemetry Characteristic
  g_autopilotCharacteristic = pService->createCharacteristic(
    AUTOPILOT_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  if (!g_autopilotCharacteristic) {
    Serial.println("[BLE] Error: Failed to create autopilot characteristic");
    return false;
  }
  
  // Create PID Control Characteristic
  g_pidCharacteristic = pService->createCharacteristic(
    PID_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  if (!g_pidCharacteristic) {
    Serial.println("[BLE] Error: Failed to create PID characteristic");
    return false;
  }
  
  // Set callbacks
  g_characteristicCallbacks = new CharacteristicCallbacks();
  g_pidCharacteristic->setCallbacks(g_characteristicCallbacks);
  
  // Initialize characteristic values
  uint8_t emptyData[1] = {0};
  g_autopilotCharacteristic->setValue(emptyData, 1);
  g_pidCharacteristic->setValue(emptyData, 1);
  
  // Start the service
  pService->start();
  
  // Start advertising
  g_advertising = BLEDevice::getAdvertising();
  g_advertising->addServiceUUID(SERVICE_UUID);
  g_advertising->setScanResponse(true);
  g_advertising->setMinPreferred(0x12);  // Functions that help with iPhone connections issue
  g_advertising->start();
  
  Serial.println("[BLE] BLE initialized successfully");
  Serial.printf("[BLE] Device name: %s\n", DEVICE_NAME);
  Serial.println("[BLE] Advertising started");
  
  return true;
}

/**
 * @brief Forward BLE data to Teensy via UART (called from UART task)
 */
void forwardBleDataToTeensy(const std::string& data) {
  if (data.empty()) {
    return;
  }
  
  // Safety check for data size
  if (data.length() > MAX_BLE_DATA_SIZE) {
    Serial.printf("[UART] Error: Data too large to forward (%d bytes, max %d)\n", 
                  data.length(), MAX_BLE_DATA_SIZE);
    return;
  }
  
  Serial.printf("[UART] Forwarding %d bytes to Teensy\n", data.length());
  
  Serial1.write(PACKET_START_MARKER);
  for (size_t i = 0; i < data.length(); i++) {
    Serial1.write(data[i]);
  }
  Serial1.write(PACKET_END_MARKER);
}

/**
 * @brief Process incoming serial data and handle complete packets
 * @return true if a complete packet was processed
 */
bool processSerialData() {
  static uint32_t lastDataTime = millis();
  bool packetProcessed = false;
  
  while (Serial1.available()) {
    uint8_t incomingByte = Serial1.read();
    lastDataTime = millis();
    
    if (incomingByte == PACKET_START_MARKER) {
      // Start of new packet
      g_bufferIndex = 0;
      g_packetInProgress = true;
      g_packetStartTime = millis();
      continue;
    }
    
    if (!g_packetInProgress) {
      continue;  // Ignore data outside of packet markers
    }
    
    if (incomingByte == PACKET_END_MARKER) {
      // End of packet - validate and process
      if (g_bufferIndex == sizeof(AutopilotTelemetry)) {
        // Valid packet received
        const AutopilotTelemetry* telemetry = reinterpret_cast<const AutopilotTelemetry*>(g_serialBuffer);
        
        // Forward to BLE task via queue
        if (g_teensyToBleQueue) {
          AutopilotTelemetry telemetryData;
          memcpy(&telemetryData, g_serialBuffer, sizeof(AutopilotTelemetry));
          xQueueSend(g_teensyToBleQueue, &telemetryData, 0);
        }
        
        // Log telemetry data
        logTelemetryData(telemetry);
        packetProcessed = true;
      } else {
        Serial.printf("[UART] Warning: Invalid packet size (%d bytes, expected %d)\n", 
                      g_bufferIndex, sizeof(AutopilotTelemetry));
      }
      
      g_packetInProgress = false;
      g_bufferIndex = 0;
      continue;
    }
    
    // Add byte to buffer
    if (g_bufferIndex < sizeof(g_serialBuffer)) {
      g_serialBuffer[g_bufferIndex++] = incomingByte;
    } else {
      // Buffer overflow - reset
      Serial.println("[UART] Error: Buffer overflow, resetting");
      g_packetInProgress = false;
      g_bufferIndex = 0;
    }
  }
  
  // Check for packet timeout
  if (g_packetInProgress && (millis() - g_packetStartTime > MAX_PACKET_TIMEOUT_MS)) {
    Serial.println("[UART] Warning: Packet timeout, resetting buffer");
    g_packetInProgress = false;
    g_bufferIndex = 0;
  }
  
  return packetProcessed;
}

/**
 * @brief Handle BLE connection state changes
 */
void handleConnectionStateChange() {
  // Device just connected
  if (g_deviceConnected && !g_previouslyConnected) {
    g_previouslyConnected = g_deviceConnected;
    Serial.println("[BLE] Connection established");
  }
  
  // Device just disconnected
  if (!g_deviceConnected && g_previouslyConnected) {
    delay(100);  // Give BLE stack time to clean up
    g_advertising->start();  // Restart advertising
    g_previouslyConnected = g_deviceConnected;
    Serial.println("[BLE] Restarted advertising");
  }
}

// ============================================================================
// FreeRTOS Task Functions
// ============================================================================

/**
 * @brief UART task - handles serial communication with Teensy
 */
void uartTask(void* parameter) {
  Serial.println("[TASK] UART task started");
  
  while (true) {
    // Process incoming serial data from Teensy
    processSerialData();
    
    // Check for data to send to Teensy
    BleDataQueueItem queueItem;
    if (xQueueReceive(g_bleToTeensyQueue, &queueItem, 0) == pdTRUE) {
      // Convert queue item to string for forwarding
      std::string dataToSend(reinterpret_cast<const char*>(queueItem.data), queueItem.length);
      forwardBleDataToTeensy(dataToSend);
    }
    
    // Small delay to prevent CPU hogging
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/**
 * @brief BLE task - handles BLE notifications
 */
void bleTask(void* parameter) {
  Serial.println("[TASK] BLE task started");
  
  while (true) {
    // Handle connection state changes
    handleConnectionStateChange();
    
    // Check for telemetry data to send via BLE
    AutopilotTelemetry telemetryData;
    if (xQueueReceive(g_teensyToBleQueue, &telemetryData, pdMS_TO_TICKS(QUEUE_WAIT_MS)) == pdTRUE) {
      if (g_deviceConnected && g_autopilotCharacteristic) {
        if (xSemaphoreTake(g_bleDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          g_autopilotCharacteristic->setValue((uint8_t*)&telemetryData, sizeof(AutopilotTelemetry));
          g_autopilotCharacteristic->notify();
          xSemaphoreGive(g_bleDataMutex);
        }
      }
    }
    
    // Small delay to prevent CPU hogging
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================================
// Main Functions
// ============================================================================

void setup() {
  // Initialize serial communication
  initializeSerial();
  
  // Initialize BLE
  if (!initializeBLE()) {
    Serial.println("[SYSTEM] FATAL: BLE initialization failed!");
    Serial.println("[SYSTEM] Entering error state...");
    cleanupBLE();  // Clean up any partially allocated resources
    while (true) {
      delay(1000);
      Serial.println("[SYSTEM] BLE initialization failed - restart required");
    }
  }
  
  // Create FreeRTOS queues
  g_teensyToBleQueue = xQueueCreate(QUEUE_SIZE, sizeof(AutopilotTelemetry));
  g_bleToTeensyQueue = xQueueCreate(QUEUE_SIZE, sizeof(BleDataQueueItem));
  
  if (!g_teensyToBleQueue || !g_bleToTeensyQueue) {
    Serial.println("[SYSTEM] FATAL: Failed to create queues!");
    while (true) {
      delay(1000);
      Serial.println("[SYSTEM] Queue creation failed - restart required");
    }
  }
  
  // Create mutex for BLE data protection
  g_bleDataMutex = xSemaphoreCreateMutex();
  if (!g_bleDataMutex) {
    Serial.println("[SYSTEM] FATAL: Failed to create mutex!");
    while (true) {
      delay(1000);
      Serial.println("[SYSTEM] Mutex creation failed - restart required");
    }
  }
  
  // Create FreeRTOS tasks
  BaseType_t uartTaskCreated = xTaskCreate(
    uartTask,                 // Task function
    "UART_Task",             // Task name
    UART_TASK_STACK_SIZE,    // Stack size
    nullptr,                 // Parameters
    UART_TASK_PRIORITY,      // Priority
    nullptr                  // Task handle
  );
  
  BaseType_t bleTaskCreated = xTaskCreate(
    bleTask,                 // Task function
    "BLE_Task",             // Task name
    BLE_TASK_STACK_SIZE,    // Stack size
    nullptr,                // Parameters
    BLE_TASK_PRIORITY,      // Priority
    nullptr                 // Task handle
  );
  
  if (uartTaskCreated != pdPASS || bleTaskCreated != pdPASS) {
    Serial.println("[SYSTEM] FATAL: Failed to create tasks!");
    while (true) {
      delay(1000);
      Serial.println("[SYSTEM] Task creation failed - restart required");
    }
  }
  
  Serial.println("[SYSTEM] Setup complete - FreeRTOS tasks started");
}

void loop() {
  // Main loop is now empty - all work is done in FreeRTOS tasks
  // This is required by Arduino framework but we don't use it
  vTaskDelay(pdMS_TO_TICKS(1000));  // Just sleep to prevent watchdog issues
}