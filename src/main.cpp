#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

const int UART_RX_PIN = 20;
const int UART_TX_PIN = 21;

BLECharacteristic *pCharacteristic;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  // BLEDevice::init("BLE Module");
  // BLEServer *pServer = BLEDevice::createServer();
  // BLEService *pService = pServer->createService(SERVICE_UUID);
  // pCharacteristic = pService->createCharacteristic(
  //                                         CHARACTERISTIC_UUID,
  //                                         BLECharacteristic::PROPERTY_READ |
  //                                         BLECharacteristic::PROPERTY_WRITE |
  //                                         BLECharacteristic::PROPERTY_NOTIFY
  //                                       );
  // pService->addCharacteristic(pCharacteristic);
  // pCharacteristic->setValue("Hello World!");
  // pService->start();
  // BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  // pAdvertising->start();
}

void loop() {
  if (Serial1.available()) {
    String uartData = Serial1.readString();
    Serial.println("Received from MCU: " + uartData);
    pCharacteristic->setValue(uartData.c_str());
    pCharacteristic->notify();
  }

  // if (pCharacteristic->getValue().length() > 0) {
  //   String bleData = pCharacteristic->getValue().c_str();
  //   Serial.println("Received from BLE: " + bleData);
  //   Serial1.print(bleData);
  //   pCharacteristic->setValue("");
  // }

  delay(100);
}