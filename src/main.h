#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#include <esp_wifi.h>
#include <Ticker.h>
#include <Log/my-log.h>
#elif defined(NRF52)
#include <nrf.h>
#include "nrf_timer.h"
#include "nrf52Timer.h"
#include <Log/my-log_nrf52.h>
#endif

extern uint32_t deviceID;

/** LoRa package types */
#define LORA_INVALID 0
#define LORA_DIRECT 1
#define LORA_FORWARD 2
#define LORA_BROADCAST 3
#define LORA_NODEMAP 4

// BLE
#include "BLE/ble_uart.h"

// LoRa & Mesh
#include <SX126x-Arduino.h>
#include <SPI.h>
#include <Mesh/mesh.h>
bool initLoRa(void);

// Display
void initDisplay(void);
void dispWriteHeader(void);
void dispWrite(String text, int x, int y);
void dispUpdate(void);