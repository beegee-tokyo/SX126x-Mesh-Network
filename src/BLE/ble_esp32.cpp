#ifdef ESP32

#include "main.h"
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>
#include <esp_bt_device.h>

/** Service UUID for Uart */
#define UART_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
/** Characteristic UUID for receiver */
#define RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
/** Characteristic UUID for transmitter */
#define TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

/** Characteristic for BLE-UART TX */
BLECharacteristic *pCharacteristicUartTX;
/** Characteristic for BLE-UART RX */
BLECharacteristic *pCharacteristicUartRX;
/** Descriptor for the BLE-UART TX characteristic */
BLEDescriptor *txDescriptor;
/** BLE Advertiser */
BLEAdvertising *pAdvertising;
/** BLE Service for WiFi*/
BLEService *wifiService;
/** BLE Service for Uart*/
BLEService *uartService;
/** BLE Server */
BLEServer *pServer;

/** Flag if a device is connected */
bool bleUARTisConnected = false;

/** Flag if client notification is enabled */
bool bleUARTnotifyEnabled = false;

/** RX and TX buffer size */
#define BUFF_SIZE 253

/** Buffer for outgoing data */
uint8_t txData[BUFF_SIZE] = {0};
/** Buffer for incoming data */
uint8_t rxData[BUFF_SIZE] = {0};
/** Number of bytes in the of incoming data buffer */
size_t rxLen = 0;

/** Mutex used to enter critical code part (RX data buffer access) */
SemaphoreHandle_t _mutex;

/**
 * Callbacks for client connection and disconnection
 */
class MyServerCallbacks : public BLEServerCallbacks
{
	void onConnect(BLEServer *pServer)
	{
		Serial.println("BLE client connected");
		pServer->updatePeerMTU(pServer->getConnId(), 260);
		bleUARTisConnected = true;
	};

	void onDisconnect(BLEServer *pServer)
	{
		Serial.println("BLE client disconnected");
		bleUARTisConnected = false;
		pAdvertising->start();
	}
};

/**
 * Callbacks for BLE client read/write requests
 * on BLE UART characteristic
 */
class UartTxCbHandler : public BLECharacteristicCallbacks
{
	void onWrite(BLECharacteristic *pCharacteristic)
	{
		std::string rxValue = pCharacteristic->getValue();
		uint8_t *rxDataPtr = pCharacteristic->getData();
		size_t buffSize = rxValue.size();

		xSemaphoreTake(_mutex, portMAX_DELAY);
		if (buffSize > 0)
		{
			if ((rxLen + buffSize) <= BUFF_SIZE)
			{
				// Enough size in the buffer
				memcpy((char *)&rxData[rxLen], (const void *)rxDataPtr, buffSize);
				rxLen += buffSize;
			}
			else
			{
				// More data than buffer available
				/// \todo decide how to handle it correctly
				memcpy((char *)&rxData[rxLen], (const void *)rxDataPtr, BUFF_SIZE - rxLen);
				rxLen = BUFF_SIZE;
				myLog_e("RX Buffer overflow!");
			}
		}
		xSemaphoreGive(_mutex);
	};
};

/**
 * Callbacks for BLE client descriptor changes
 * on BLE UART characteristic
 */
class DescriptorCallbacks : public BLEDescriptorCallbacks
{
	void onWrite(BLEDescriptor *pDescriptor)
	{
		uint8_t *descrValue;
		descrValue = pDescriptor->getValue();
		if (descrValue[0] & (1 << 0))
		{
			bleUARTnotifyEnabled = true;
		}
		else
		{
			bleUARTnotifyEnabled = false;
		}
	};
};

/**
 * Initialize nRF52 BLE UART
 */
void initBLE(void)
{
	char apName[] = "DR-xxxxxxxxxxxx";
	// Using ESP32 MAC (48 bytes only, so upper 2 bytes will be 0)
	sprintf(apName, "DR-%08lX", deviceID);
	Serial.printf("Device name: %s\n", apName);

	// Create mutex for access to RX data buffer
	_mutex = xSemaphoreCreateMutex();
	xSemaphoreGive(_mutex);

	// Initialize BLE and set output power
	BLEDevice::init(apName);
	BLEDevice::setMTU(260);
	BLEDevice::setPower(ESP_PWR_LVL_P7);

	BLEAddress thisAddress = BLEDevice::getAddress();

	Serial.printf("BLE address: %s\n", thisAddress.toString().c_str());

	// Create BLE Server
	pServer = BLEDevice::createServer();

	// Set server callbacks
	pServer->setCallbacks(new MyServerCallbacks());

	// Create the UART BLE Service
	uartService = pServer->createService(UART_UUID);

	// Create a BLE Characteristic
	pCharacteristicUartTX = uartService->createCharacteristic(
		TX_UUID,
		BLECharacteristic::PROPERTY_NOTIFY |
			BLECharacteristic::PROPERTY_READ);

	pCharacteristicUartTX->addDescriptor(new BLE2902());

	// Register callback for notification enabled
	pCharacteristicUartTX->setNotifyProperty(true);

	pCharacteristicUartRX = uartService->createCharacteristic(
		RX_UUID,
		BLECharacteristic::PROPERTY_WRITE);

	pCharacteristicUartRX->setCallbacks(new UartTxCbHandler());

	txDescriptor = pCharacteristicUartTX->getDescriptorByUUID("2902");
	if (txDescriptor != NULL)
	{
		Serial.println("Got descriptor for TX as 2902");
		txDescriptor->setCallbacks(new DescriptorCallbacks());
	}

	// Start the service
	uartService->start();

	// Start advertising
	pAdvertising = pServer->getAdvertising();
	pAdvertising->addServiceUUID(UART_UUID);
	pAdvertising->start();
}

/**
 * Platform independant function to check BLE UART for
 * incoming data
 * @return number of bytes in the RX buffer or 0 if empty
 */
int bleUartAvailable(void)
{
	return rxLen;
}

/**
 * Platform independant function to read out BLE RX buffer
 * @param data pointer to receiving buffer
 * @param buffSize size of receiving buffer
 * @return number of bytes read
 */
size_t bleUartRead(char *data, size_t buffSize)
{
	xSemaphoreTake(_mutex, portMAX_DELAY);
	size_t numOfReadBytes = rxLen;
	if (rxLen <= buffSize)
	{
		memcpy(data, rxData, rxLen);
		rxLen = 0;
		xSemaphoreGive(_mutex);
		return numOfReadBytes;
	}
	else
	{
		memcpy(data, rxData, rxLen);
		// Move unread data to the beginning of the buffer
		memcpy(rxData, &rxData[rxLen], rxLen - buffSize);
		rxLen = rxLen - buffSize;
		xSemaphoreGive(_mutex);
		return buffSize;
	}
}

/**
 * Platform independant function to send data over BLE UART
 * @param data pointer to data
 * @param buffSize size of data
 * @return number of bytes written
 */
size_t bleUartWrite(char *data, size_t buffSize)
{
	if (buffSize <= BUFF_SIZE)
	{
		memcpy(txData, data, buffSize);
		pCharacteristicUartTX->setValue(txData, buffSize);
		pCharacteristicUartTX->notify();
		delay(100);
		return buffSize;
	}
	else
	{
		memcpy(txData, data, BUFF_SIZE);
		pCharacteristicUartTX->setValue(txData, BUFF_SIZE);
		pCharacteristicUartTX->notify();
		delay(100);
		return (size_t)BUFF_SIZE;
	}
}

#endif
