#ifdef NRF52_SERIES
#include "main.h"
#include <bluefruit.h>

// OTA DFU service
BLEDfu bledfu;
// UART service
BLEUart bleuart;

/** Flag if a device is connected */
bool bleUARTisConnected = false;

/** Flag if client notification is enabled */
bool bleUARTnotifyEnabled = false;

/** Callback for client notification enable/disable */
void uartNotifyCallback(uint16_t connHandle, boolean status);
/** Callback for client connection */
void connect_callback(uint16_t conn_handle);
/** Callback for client disconnect */
void disconnect_callback(uint16_t conn_handle, uint8_t reason);

/** Start advertising */
void startAdv(void);

/**
 * Initialize nRF52 BLE UART
 */
void initBLE(void)
{
	Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

	Bluefruit.begin();
	// Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
	Bluefruit.setTxPower(4);

	char serial_str[256] = {0};
	sprintf(serial_str, "DR-%08lX", deviceID);
	myLog_d("BLE name %s", serial_str);

	Bluefruit.setName(serial_str);

	Bluefruit.autoConnLed(true);

	Bluefruit.Periph.setConnectCallback(connect_callback);
	Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

	// To be consistent OTA DFU should be added first if it exists
	bledfu.begin();

	// Configure and Start BLE Uart Service
	bleuart.begin();

	bleuart.setNotifyCallback(uartNotifyCallback);

	// Set up and start advertising
	startAdv();
}

/**
 * Platform independant function to check BLE UART for
 * incoming data
 * @return number of bytes in the RX buffer or 0 if empty
 */
int bleUartAvailable(void)
{
	return bleuart.available();
}

/**
 * Platform independant function to read out BLE RX buffer
 * @param data pointer to receiving buffer
 * @param buffSize size of receiving buffer
 * @return number of bytes read
 */
size_t bleUartRead(char *data, size_t buffSize)
{
	return (size_t)bleuart.read(data, buffSize);
}

/**
 * Platform independant function to send data over BLE UART
 * @param data pointer to data
 * @param buffSize size of data
 * @return number of bytes written
 */
size_t bleUartWrite(char *data, size_t buffSize)
{
	size_t bytesWritten = bleuart.write(data, buffSize);
	delay(100);
	return bytesWritten;
}

/**
 * Start nRF52 BLE UART advertising
 */
void startAdv(void)
{
	// Advertising packet
	Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
	Bluefruit.Advertising.addTxPower();
	Bluefruit.Advertising.addName();

	/* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds 
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
	Bluefruit.Advertising.restartOnDisconnect(true);
	Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
	Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
	Bluefruit.Advertising.start(0);				// 0 = Don't stop advertising after n seconds
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
	(void)conn_handle;
	bleUARTisConnected = true;
	myLog_d("BLE connected");
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
	(void)conn_handle;
	(void)reason;
	bleUARTisConnected = false;
	myLog_d("BLE disconnected");
}

/** 
 * Callback invoked when client enables notifications.
 * Data should not be sent before notifications are enabled
 * @param conn_handle connection where this event happens
 * @param status notification status
 */
void uartNotifyCallback(uint16_t connHandle, boolean status)
{
	bleUARTnotifyEnabled = status;
}
#endif