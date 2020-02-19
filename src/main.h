#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#include <esp_wifi.h>
#include <Ticker.h>
#include <my-log.h>
#elif defined(NRF52)
#include <nrf.h>
#include "nrf_timer.h"
#include "nrf52Timer.h"
#include <my-log_nrf52.h>
#endif

extern uint32_t deviceID;

// BLE
#include "ble_uart.h"
// LoRa
#include <SX126x-Arduino.h>
#include <SPI.h>
// Mesh
#include <mesh.h>
// Display
void initDisplay(void);
void dispWriteHeader(void);
void dispWrite(String text, int x, int y);
void dispUpdate(void);