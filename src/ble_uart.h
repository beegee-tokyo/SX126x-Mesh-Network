/**
 * Definitions for both ESP32 and nRF52 BLE UART
 */

#include <Arduino.h>

void initBLE(void);
int bleUartAvailable(void);
size_t bleUartRead(char *data, size_t buffSize);
size_t bleUartWrite(char *data, size_t buffSize);

extern bool bleUARTisConnected;