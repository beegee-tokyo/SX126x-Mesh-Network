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
// // BLE functions & variables
// void initBLE(void);
// void startAdv(void);
// void connect_callback(uint16_t conn_handle);
// void disconnect_callback(uint16_t conn_handle, uint8_t reason);
// extern bool bleUARTisConnected;
#endif
#include "ble_uart.h"
#include <SX126x-Arduino.h>
#include <SPI.h>

#include <mesh.h>

extern uint32_t deviceID;
extern uint8_t numElements;

void initDisplay(void);
void dispWriteHeader(void);
void dispWrite(String text, int x, int y);
void dispUpdate(void);