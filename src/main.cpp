#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define AUTOPILOT_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define PID_CHARACTERISTIC_UUID "98ab29d2-2b95-497d-9df7-f064e5ac05a5"

const int UART_RX_PIN = 20;
const int UART_TX_PIN = 21;

typedef struct __attribute__((packed)) {
  float kp;
  float ki;
  float kd;
  float bearingPositionToDestinationWaypoint;
  float destinationLatitude;
  float destinationLongitude;
  float previousDestinationLatitude;
  float previousDestinationLongitude;
  float heading;
  float xte;
  float previousXte;
  float previousTime;
  float previousBearing;
  float integralXTE;
  float derivativeXTE;
  float timeDelta;
  float rudderAngle;
  float rudderPosition;
  float targetMotorPosition;
  bool homingComplete;
} Autopilot;

BLECharacteristic *autopilotCharacteristic;
BLECharacteristic *pidCharacteristic;
BLEAdvertising* pAdvertising = NULL;
byte incomingBuffer[sizeof(Autopilot)];
std::string bleData;
bool deviceConnected = false;
bool isWritten = false;
byte startMarker = 0x02;
byte endMarker = 0x03;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("Device connected");
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    Serial.println("Device disconnected");
    deviceConnected = false;
    pAdvertising->start();
    Serial.println("Advertising restarted");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    bleData = pCharacteristic->getValue();
    isWritten = true;
  }
};

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  BLEDevice::init("Autopilot");
  BLEDevice::setMTU(80);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  autopilotCharacteristic = pService->createCharacteristic(
    AUTOPILOT_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pidCharacteristic = pService->createCharacteristic(
    PID_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  autopilotCharacteristic->setCallbacks(new MyCallbacks());
  pidCharacteristic->setCallbacks(new MyCallbacks());
  pService->addCharacteristic(autopilotCharacteristic);
  pService->addCharacteristic(pidCharacteristic);
  autopilotCharacteristic->setValue("");
  pidCharacteristic->setValue("");
  pService->start();
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
}

void loop() {
  static byte buffer[sizeof(Autopilot)];
  static int bufferIndex = 0;
  bool dataStarted = false;

  while (Serial1.available()) {
    byte incomingByte = Serial1.read();
    if (incomingByte == startMarker) {  // Start marker detected
      bufferIndex = 0;
      dataStarted = true;
    } else if (dataStarted && bufferIndex < sizeof(Autopilot)) {
      buffer[bufferIndex++] = incomingByte;
    } else if (incomingByte == endMarker && bufferIndex == sizeof(Autopilot)) {  // End marker detected
      // Deserialize the buffer into the Autopilot struct
      Autopilot* receivedData = (Autopilot*)buffer;

      if (deviceConnected) {
        if (isWritten) {
          Serial1.write(startMarker);
          for (int i = 0; i < bleData.length(); i++) {
            Serial1.write(bleData[i]);
          }
          Serial1.write(endMarker);
          isWritten = false;
        } else {
          autopilotCharacteristic->setValue(buffer, sizeof(Autopilot));
          autopilotCharacteristic->notify();
        }
      }

      Serial.print("kp: ");
      Serial.println(receivedData->kp);
      Serial.print("ki: ");
      Serial.println(receivedData->ki);
      Serial.print("kd: ");
      Serial.println(receivedData->kd);
      Serial.print("destinationLatitude: ");
      Serial.println(receivedData->destinationLatitude);
      Serial.print("destinationLongitude: ");
      Serial.println(receivedData->destinationLongitude);
      Serial.print("bearingPositionToDestinationWaypoint: ");
      Serial.println(receivedData->bearingPositionToDestinationWaypoint);
      Serial.print("heading: ");
      Serial.println(receivedData->heading);
      Serial.print("xte: ");
      Serial.println(receivedData->xte);
      Serial.print("previousXte: ");
      Serial.println(receivedData->previousXte);
      Serial.print("previousTime: ");
      Serial.println(receivedData->previousTime);
      Serial.print("previousBearing: ");
      Serial.println(receivedData->previousBearing);
      Serial.print("integralXTE: ");
      Serial.println(receivedData->integralXTE);
      Serial.print("derivativeXTE: ");
      Serial.println(receivedData->derivativeXTE);
      Serial.print("timeDelta: ");
      Serial.println(receivedData->timeDelta);
      Serial.print("rudderAngle: ");
      Serial.println(receivedData->rudderAngle);
      Serial.print("rudderPosition: ");
      Serial.println(receivedData->rudderPosition);
      Serial.print("targetMotorPosition: ");
      Serial.println(receivedData->targetMotorPosition);
      Serial.println("==================================");

      bufferIndex = 0;
      dataStarted = false;
    }
  }

  delay(500);
}