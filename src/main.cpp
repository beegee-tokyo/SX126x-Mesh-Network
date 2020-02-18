#include "main.h"

/** HW configuration structure for the LoRa library */
hw_config hwConfig;

#ifdef ESP32
#ifdef IS_NOT_FEATHER
// ESP32 Wrover - SX126x pin configuration
/** LORA RESET */
int PIN_LORA_RESET = 4;
/** LORA DIO_1 */
int PIN_LORA_DIO_1 = 21;
/** LORA SPI BUSY */
int PIN_LORA_BUSY = 22;
/** LORA SPI CS */
int PIN_LORA_NSS = 5;
/** LORA SPI CLK */
int PIN_LORA_SCLK = 18;
/** LORA SPI MISO */
int PIN_LORA_MISO = 19;
/** LORA SPI MOSI */
int PIN_LORA_MOSI = 23;
/** LORA ANTENNA TX ENABLE */
int RADIO_TXEN = 26;
/** LORA ANTENNA RX ENABLE */
int RADIO_RXEN = 27;

#define LED_BUILTIN 15
#else
// ESP32 Feather - SX126x pin configuration
/** LORA RESET */
int PIN_LORA_RESET = 32;
/** LORA DIO_1 */
int PIN_LORA_DIO_1 = 14;
/** LORA SPI BUSY */
int PIN_LORA_BUSY = 27;
/** LORA SPI CS */
int PIN_LORA_NSS = 33;
/** LORA SPI CLK */
int PIN_LORA_SCLK = SCK;
/** LORA SPI MISO */
int PIN_LORA_MISO = MISO;
/** LORA SPI MOSI */
int PIN_LORA_MOSI = MOSI;
/** LORA ANTENNA TX ENABLE */
int RADIO_TXEN = -1;
/** LORA ANTENNA RX ENABLE */
int RADIO_RXEN = -1;
#endif
#ifdef RED_ESP
#undef LED_BUILTIN
#define LED_BUILTIN 16
#endif
#endif
#ifdef NRF52
#ifdef ADAFRUIT
/** LORA RESET */
int PIN_LORA_RESET = 31;
/** LORA DIO_1 */
int PIN_LORA_DIO_1 = 11;
/** LORA SPI BUSY */
int PIN_LORA_BUSY = 30;
/** LORA SPI CS */
int PIN_LORA_NSS = 27;
/** LORA SPI CLK */
int PIN_LORA_SCLK = SCK;
/** LORA SPI MISO */
int PIN_LORA_MISO = MISO;
/** LORA SPI MOSI */
int PIN_LORA_MOSI = MOSI;
/** LORA ANTENNA TX ENABLE */
int RADIO_TXEN = -1;
/** LORA ANTENNA RX ENABLE */
int RADIO_RXEN = -1;
// Singleton for SPI connection to the LoRa chip
SPIClass SPI_LORA(NRF_SPIM1, MISO, SCK, MOSI);
#else
// Singleton for SPI connection to the LoRa chip
SPIClass SPI_LORA(NRF_SPIM2, MISO, SCK, MOSI);
#undef LED_BUILTIN
#define LED_BUILTIN 22
#endif
#endif

void OnLoraData(uint8_t *rxPayload, uint16_t rxSize, int16_t rxRssi, int8_t rxSnr);
static MeshEvents_t MeshEvents;

#ifdef ESP32
Ticker ledOffTick;
#else
#define nrf_timer_num (1)
#define cc_channel_num (0)
TimerClass timer(nrf_timer_num, cc_channel_num);
#endif

dataMsg outData;
nodesList routeToNode;
uint32_t nodeId[48];
uint32_t firstHop[48];
uint8_t numHops[48];
uint8_t numElements;

char sendData[512] = {0};

void ledOff(void)
{
#if defined(HAS_DISPLAY) || defined(RED_ESP)
	digitalWrite(LED_BUILTIN, HIGH);
#else
	digitalWrite(LED_BUILTIN, LOW);
#endif
}

void setup()
{
#ifdef NRF52
	pinMode(17, OUTPUT);
	digitalWrite(17, HIGH);
#endif

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Start Serial
	Serial.begin(115200);

#ifdef ADAFRUIT
	// The serial interface of the nRF52's seems to need some time before it is ready
	delay(500);
#endif

	// Create node ID
	uint8_t deviceMac[8];

	BoardGetUniqueId(deviceMac);

	deviceID += (uint32_t)deviceMac[2] << 24;
	deviceID += (uint32_t)deviceMac[3] << 16;
	deviceID += (uint32_t)deviceMac[4] << 8;
	deviceID += (uint32_t)deviceMac[5];

	myLog_n("Mesh NodeId = %08lX", deviceID);

#ifdef HAS_DISPLAY
	initDisplay();
#endif

	// Initialize BLE
	initBLE();

	// Initialize the LoRa
#if defined(ESP32) || defined(ADAFRUIT)
	// Define the HW configuration between MCU and SX126x
	hwConfig.CHIP_TYPE = SX1262_CHIP;		  // eByte E22 module with an SX1262
	hwConfig.PIN_LORA_RESET = PIN_LORA_RESET; // LORA RESET
	hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;	 // LORA SPI CS
	hwConfig.PIN_LORA_SCLK = PIN_LORA_SCLK;   // LORA SPI CLK
	hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;   // LORA SPI MISO
	hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO_1; // LORA DIO_1
	hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;   // LORA SPI BUSY
	hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;   // LORA SPI MOSI
	hwConfig.RADIO_TXEN = RADIO_TXEN;		  // LORA ANTENNA TX ENABLE
	hwConfig.RADIO_RXEN = RADIO_RXEN;		  // LORA ANTENNA RX ENABLE
	hwConfig.USE_DIO2_ANT_SWITCH = true;	  // Example uses an eByte E22 module which uses RXEN and TXEN pins as antenna control
	hwConfig.USE_DIO3_TCXO = true;			  // Example uses an eByte E22 module which uses DIO3 to control oscillator voltage
	hwConfig.USE_DIO3_ANT_SWITCH = false;	 // Only Insight ISP4520 module uses DIO3 as antenna control

	if (lora_hardware_init(hwConfig) != 0)
	{
		myLog_e("Error in hardware init");
	}
#else // ISP4520
	if (lora_isp4520_init(SX1262) != 0)
	{
		myLog_e("Error in hardware init");
	}
#endif
	uint16_t readSyncWord = 0;
	SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);
	myLog_w("Got syncword %X", readSyncWord);

	MeshEvents.DataAvailable = OnLoraData;

	// Initialize the LoRa Mesh
#ifdef ESP32
	initMesh(&MeshEvents, 48);
#else
	initMesh(&MeshEvents, 30);
#endif
}

void loop()
{
	delay(30000);
	Serial.println("---------------------------------------------");
	if (xSemaphoreTake(accessNodeList, (TickType_t)1000) == pdTRUE)
	{
		numElements = numOfNodes();
#ifdef HAS_DISPLAY
		dispWriteHeader();
		char line[128];
		// sprintf(line, "%08X", deviceID);
		sprintf(line, "%02X%02X", (uint8_t)(deviceID >> 24), (uint8_t)(deviceID >> 16));
		dispWrite(line, 0, 11);
#endif
		for (int idx = 0; idx < numElements; idx++)
		{
			getNode(idx, nodeId[idx], firstHop[idx], numHops[idx]);
		}
		// Select random node to send a package
		getRoute(nodeId[random(0, numElements)], &routeToNode);
		// Release access to nodes list
		xSemaphoreGive(accessNodeList);
		// Prepare data
		outData.mark1 = 'L';
		outData.mark2 = 'o';
		outData.mark3 = 'R';
		if (routeToNode.firstHop != 0)
		{
			outData.dest = routeToNode.firstHop;
			outData.from = routeToNode.nodeId;
			outData.type = 2;
			Serial.printf("Queuing msg to hop to %08X over %08X\n", outData.from, outData.dest);
			if (bleUARTisConnected)
			{
				int sendLen = snprintf(sendData, 512, "Queuing msg to hop to %08X over %08X\n", outData.from, outData.dest);
				bleUartWrite(sendData, sendLen);
			}
		}
		else
		{
			outData.dest = routeToNode.nodeId;
			outData.from = deviceID;
			outData.type = 1;
			Serial.printf("Queuing msg direct to %08X\n", outData.dest);
			if (bleUARTisConnected)
			{
				int sendLen = snprintf(sendData, 512, "Queuing msg direct to %08X\n", outData.dest);
				bleUartWrite(sendData, sendLen);
			}
		}
		int dataLen = MAP_HEADER_SIZE + sprintf((char *)outData.data, ">>%08X<<", deviceID);
		// Add package to send queue
		if (!addSendRequest(&outData, dataLen))
		{
			Serial.println("Sending package failed");
			if (bleUARTisConnected)
			{
				int sendLen = snprintf(sendData, 512, "Sending package failed\n");
				bleUartWrite(sendData, sendLen);
			}
		}

		// Display the nodes
		Serial.printf("%d nodes in the map\n", numElements + 1);
		Serial.printf("Node #01 id: %08X\n", deviceID);
		if (bleUARTisConnected)
		{
			int sendLen = snprintf(sendData, 512, "%d nodes in the map\n", numElements + 1);
			bleUartWrite(sendData, sendLen);
			sendLen = snprintf(sendData, 512, "Node #01 id: %08X\n", deviceID);
			bleUartWrite(sendData, sendLen);
		}
		for (int idx = 0; idx < numElements; idx++)
		{
#ifdef HAS_DISPLAY
			if (firstHop[idx] == 0)
			{
				// sprintf(line, "%08X", nodeId[idx]);
				sprintf(line, "%02X%02X", (uint8_t)(nodeId[idx] >> 24), (uint8_t)(nodeId[idx] >> 16));
			}
			else
			{
				// sprintf(line, "%08X*", nodeId[idx]);
				sprintf(line, "%02X%02X*", (uint8_t)(nodeId[idx] >> 24), (uint8_t)(nodeId[idx] >> 16));
			}
			if (idx < 4)
			{
				dispWrite(line, 0, ((idx + 2) * 10) + 1);
			}
			else if (idx < 9)
			{
				dispWrite(line, 42, ((idx - 3) * 10) + 1);
			}
			else
			{
				dispWrite(line, 84, ((idx - 8) * 10) + 1);
			}
			
#endif
			if (firstHop[idx] == 0)
			{
				Serial.printf("Node #%02d id: %08X direct\n", idx + 2, nodeId[idx]);
				if (bleUARTisConnected)
				{
					int sendLen = snprintf(sendData, 512, "Node #%02d id: %08LX direct\n", idx + 2, nodeId[idx]);
					bleUartWrite(sendData, sendLen);
				}
			}
			else
			{
				Serial.printf("Node #%02d id: %08X first hop %08X #hops %d\n", idx + 2, nodeId[idx], firstHop[idx], numHops[idx]);
				if (bleUARTisConnected)
				{
					int sendLen = snprintf(sendData, 512, "Node #%02d id: %08X first hop %08X #hops %d\n", idx + 2, nodeId[idx], firstHop[idx], numHops[idx]);
					bleUartWrite(sendData, sendLen);
				}
			}
		}
#ifdef HAS_DISPLAY
		dispUpdate();
#endif
	}
	else
	{
		Serial.println("Could not access the nodes list");
	}

	Serial.println("---------------------------------------------");
}

/**
 * Callback after a LoRa package was received
 * @param payload
 * 			Pointer to the received data
 * @param size
 * 			Length of the received package
 * @param rssi
 * 			Signal strength while the package was received
 * @param snr
 * 			Signal to noise ratio while the package was received
 */
void OnLoraData(uint8_t *rxPayload, uint16_t rxSize, int16_t rxRssi, int8_t rxSnr)
{
	Serial.println("-------------------------------------");
	Serial.println("Got");
	for (int idx = 0; idx < rxSize; idx++)
	{
		Serial.printf("%02X ", rxPayload[idx]);
	}
	Serial.printf("\n\n%s\n", rxPayload);
	Serial.println("-------------------------------------");
	if (bleUARTisConnected)
	{
		// int sendLen = snprintf(sendData, 512, "Received data package:%s\n", rxPayload);
		// bleUartWrite(sendData, sendLen);
		bleUartWrite((char *)rxPayload, rxSize);
		sendData[0] = '\n';
		bleUartWrite(sendData, 1);
	}

#if defined(HAS_DISPLAY) || defined(RED_ESP)
	digitalWrite(LED_BUILTIN, LOW);
#else
	digitalWrite(LED_BUILTIN, HIGH);
#endif
#ifdef ESP32
	ledOffTick.detach();
	ledOffTick.once(1, ledOff);
#else
	timer.attachInterrupt(&ledOff, 1000 * 1000); // microseconds
#endif
}