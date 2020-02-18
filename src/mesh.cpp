#include "main.h"

#define BROKEN_NET

/** Task to handle mesh */
TaskHandle_t meshTaskHandle = NULL;

/** Queue to handle cloud send requests */
volatile xQueueHandle meshMsgQueue;

/** Counter for CAD retry */
uint8_t channelFreeRetryNum = 0;

/** The Mesh node ID, created from ID of the nRF52 */
uint32_t deviceID;

/** Map message buffer */
mapMsg syncMsg;

/** Max number of messages in the queue */
#define SEND_QUEUE_SIZE 10
/** Send buffer for SEND_QUEUE_SIZE messages */
dataMsg sendMsg[SEND_QUEUE_SIZE];
/** Message size buffer for SEND_QUEUE_SIZE messages */
uint8_t sendMsgSize[SEND_QUEUE_SIZE];
/** Queue to handle send requests */
volatile xQueueHandle sendQueue;
#ifdef ESP32
/** Mux used to enter critical code part (clear up queue content) */
portMUX_TYPE accessMsgQueue = portMUX_INITIALIZER_UNLOCKED;
#endif
/** Mux used to enter critical code part (access to node list) */
SemaphoreHandle_t accessNodeList;

/** Package to be sent */
uint8_t txPckg[256];
/** Size of data package */
uint16_t txLen = 0;

uint8_t rxBuffer[256];

/** Sync time for routing at start */
#define INIT_SYNCTIME 30000
/** Sync time for routing after mesh has settled */
#define DEFAULT_SYNCTIME 60000
/** Time to switch from INIT_SYNCTIME to DEFAULT_SYNCTIME */
#define SWITCH_SYNCTIME 300000
/** Sync time */
time_t syncTime = INIT_SYNCTIME;

/**
 * Sleep and Listen time definitions 
 * 2 -> number of preambles required to detect package 
 * 1024 -> length of a symbol im ms
 * 1000 -> wake time is in us
 * 15.625 -> SX126x counts in increments of 15.625 us
 * 
 * 10 -> max length we can sleep in symbols 
 * 1024 -> length of a symbol im ms
 * 1000 -> sleep time is in us
 * 15.625 -> SX126x counts in increments of 15.625 us
 */
#define RX_SLEEP_TIMES 2 * 1024 * 1000 * 15.625, 10 * 1024 * 1000 * 15.625

typedef enum
{
	MESH_IDLE = 0, //!< The radio is idle
	MESH_RX,	   //!< The radio is in reception state
	MESH_TX,	   //!< The radio is in transmission state
	MESH_NOTIF	 //!< The radio is doing mesh notification
} meshRadioState_t;

meshRadioState_t loraState = MESH_IDLE;

/** LoRa callback events */
static RadioEvents_t RadioEvents;

/*!
 * Mesh callback variable
 */
static MeshEvents_t *_MeshEvents;

int _numOfNodes = 0;

/** Timeout for RX after Preamble detection */
time_t preambTimeout;

boolean nodesChanged = false;

/**
 * Initialize the MESH functions
 */
// void initMesh(MeshEvents_t *events, int numOfNodes, RadioModems_t modem, uint32_t fre, int8_t power,
// 			  uint32_t fdev,
// 			  uint32_t bandwidth,
// 			  uint32_t datarate,
// 			  uint8_t coderate,
// 			  uint16_t preambleLen,
// 			  bool fixLen,
// 			  bool crcOn,
// 			  bool freqHopOn,
// 			  uint8_t hopPeriod,
// 			  bool iqInverted,
// 			  uint32_t timeout,
// 			  uint32_t bandwidthAfc,
// 			  uint16_t symbTimeout,
// 			  uint8_t payloadLen,
// 			  bool rxContinuous)
void initMesh(MeshEvents_t *events, int numOfNodes)
{
	_MeshEvents = events;

	// Initialize the callbacks
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;
	RadioEvents.CadDone = OnCadDone;
	// RadioEvents.PreAmpDetect = OnPreAmbDetect;
	Radio.Init(&RadioEvents);

	_numOfNodes = numOfNodes;

	nodesMap = (nodesList *)malloc(_numOfNodes * sizeof(nodesList));

	if (nodesMap == NULL)
	{
		myLog_e("Could not allocate memory for nodes map");
	}
	else
	{
		myLog_d("Memory for nodes map is allocated");
	}
	memset(nodesMap, 0, _numOfNodes * sizeof(nodesList));

	// Create queue
	sendQueue = xQueueCreate(SEND_QUEUE_SIZE, sizeof(uint8_t));
	if (sendQueue == NULL)
	{
		myLog_e("Could not create send queue!");
	}
	else
	{
		myLog_d("Send queue created!");
	}
	// Create blocking semaphore for nodes list access
	accessNodeList = xSemaphoreCreateBinary();
	xSemaphoreGive(accessNodeList);

	// Put LoRa into standby
	Radio.Standby();

	// Set Frequency
	Radio.SetChannel(RF_FREQUENCY);

	// Set transmit configuration
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
					  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
					  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
					  true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

	// Set receive configuration
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
					  LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
					  LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
					  0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

	// Create message queue for LoRa
	meshMsgQueue = xQueueCreate(10, sizeof(uint8_t));
	if (meshMsgQueue == NULL)
	{
		myLog_e("Could not create LoRa message queue!");
	}
	else
	{
		myLog_d("LoRa message queue created!");
	}

	if (!xTaskCreate(meshTask, "MeshSync", 3096, NULL, 1, &meshTaskHandle))
	{
		myLog_e("Starting Mesh Sync Task failed");
	}
	else
	{
		myLog_d("Starting Mesh Sync Task success");
	}
}

/**
 * Task to handle the mesh
 */
void meshTask(void *pvParameters)
{
	// Queue variable to be sent to the task
	uint8_t queueIndex;

	time_t notifyTimer = millis() + syncTime;
	// time_t cleanTimer = millis();
	time_t checkSwitchSyncTime = millis();

	loraState = MESH_IDLE;
	// Start waiting for data package
	Radio.Standby();

	SX126xSetDioIrqParams(IRQ_RADIO_ALL,
						  IRQ_RADIO_ALL,
						  IRQ_RADIO_NONE, IRQ_RADIO_NONE);
	Radio.Rx(0);

	time_t txTimeout = millis();

	while (1)
	{
		Radio.IrqProcess();

		if (nodesChanged)
		{
			nodesChanged = false;
			if ((_MeshEvents != NULL) && (_MeshEvents->NodesListChanged != NULL))
			{
				_MeshEvents->NodesListChanged();
			}
		}
		// Time to sync the Mesh ???
		if ((millis() - notifyTimer) >= syncTime)
		{
			if (xSemaphoreTake(accessNodeList, (TickType_t)1000) == pdTRUE)
			{
				myLog_v("Checking mesh map");
				if (!cleanMap())
				{
					syncTime = INIT_SYNCTIME;
					checkSwitchSyncTime = millis();
					if ((_MeshEvents != NULL) && (_MeshEvents->NodesListChanged != NULL))
					{
						_MeshEvents->NodesListChanged();
					}
				}
				myLog_d("Sending mesh map");
				syncMsg.from = deviceID;
				syncMsg.type = 5;
				memset(syncMsg.nodes, 0, 48 * 5);

				// Get sub nodes
				uint8_t subsLen = nodeMap(syncMsg.nodes);

				if (subsLen != 0)
				{
					for (int idx = 0; idx < subsLen; idx++)
					{
						uint32_t checkNode = syncMsg.nodes[idx][0];
						checkNode |= syncMsg.nodes[idx][1] << 8;
						checkNode |= syncMsg.nodes[idx][2] << 16;
						checkNode |= syncMsg.nodes[idx][3] << 24;
					}
				}
				syncMsg.nodes[subsLen][0] = 0xAA;
				syncMsg.nodes[subsLen][1] = 0x55;
				syncMsg.nodes[subsLen][2] = 0x00;
				syncMsg.nodes[subsLen][3] = 0xFF;
				syncMsg.nodes[subsLen][4] = 0xAA;
				subsLen++;

				subsLen = MAP_HEADER_SIZE + (subsLen * 5);

				if (!addSendRequest((dataMsg *)&syncMsg, subsLen))
				{
					myLog_e("Cannot send map because send queue is full");
				}
				notifyTimer = millis();

				xSemaphoreGive(accessNodeList);
			}
			else
			{
				myLog_e("Cannot access map for clean up and syncing");
			}
		}

		// Time to relax the syncing ???
		if (((millis() - checkSwitchSyncTime) >= SWITCH_SYNCTIME) && (syncTime != DEFAULT_SYNCTIME))
		{
			myLog_v("Switching sync time to DEFAULT_SYNCTIME");
			syncTime = DEFAULT_SYNCTIME;
			checkSwitchSyncTime = millis();
		}

		// Check if loraState is stuck in MESH_TX
		if ((loraState == MESH_TX) && ((millis() - txTimeout) > 2000))
		{
			Radio.Standby();
			Radio.Rx(0);
			loraState = MESH_IDLE;
			myLog_e("loraState stuck in TX for 2 seconds");
		}

		// Check if we have something in the queue
		if (xQueuePeek(sendQueue, &queueIndex, (TickType_t)10) == pdTRUE)
		{
			if (loraState != MESH_TX)
			{
#ifdef ESP32
				portENTER_CRITICAL(&accessMsgQueue);
#else
				taskENTER_CRITICAL();
#endif
				txLen = sendMsgSize[queueIndex];
				memset(txPckg, 0, 256);
				memcpy(txPckg, &sendMsg[queueIndex].mark1, txLen);
				if (xQueueReceive(sendQueue, &queueIndex, portMAX_DELAY) == pdTRUE)
				{
					sendMsg[queueIndex].type = 0;
#ifdef ESP32
					portEXIT_CRITICAL(&accessMsgQueue);
#else
					taskEXIT_CRITICAL();
#endif

					myLog_d("Sending msg #%d with len %d", queueIndex, txLen);

					loraState = MESH_TX;

					Radio.Standby();
					SX126xSetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
					SX126xSetDioIrqParams(IRQ_RADIO_ALL,
										  IRQ_RADIO_ALL,
										  IRQ_RADIO_NONE, IRQ_RADIO_NONE);
					Radio.StartCad();
					txTimeout = millis();
				}
				else
				{
#ifdef ESP32
					portEXIT_CRITICAL(&accessMsgQueue);
#else
					taskEXIT_CRITICAL();
#endif
				}
			}
		}

		// Enable a task switch
		delay(100);
	}
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
void OnRxDone(uint8_t *rxPayload, uint16_t rxSize, int16_t rxRssi, int8_t rxSnr)
{
	// Secure buffer before restart listening
	if (rxSize < 256)
	{
		memcpy(rxBuffer, rxPayload, rxSize + 1);
	}
	else
	{
		memcpy(rxBuffer, rxPayload, rxSize);
	}

	uint16_t tempSize = rxSize;

	delay(1);

	loraState = MESH_IDLE;

	myLog_v("OnRxDone");
	myLog_d("LoRa Packet received size:%d, rssi:%d, snr:%d", rxSize, rxRssi, rxSnr);
#if MYLOG_LOG_LEVEL == MYLOG_LOG_LEVEL_VERBOSE
	for (int idx = 0; idx < rxSize; idx++)
	{
		Serial.printf(" %02X", rxBuffer[idx]);
	}
	Serial.println("");
#endif

	// Restart listening
	Radio.Standby();
	Radio.Rx(0);
	// Radio.SetRxDutyCycle(RX_SLEEP_TIMES);

	/// \todo Check the received data
	if ((rxBuffer[0] == 'L') && (rxBuffer[1] == 'o') && (rxBuffer[2] == 'R'))
	{
		// Valid Mesh data received
		mapMsg *thisMsg = (mapMsg *)rxBuffer;
		dataMsg *thisDataMsg = (dataMsg *)rxBuffer;

		if (thisMsg->type == 5)
		{
			/// \todo for debug make one node unreachable
#ifdef BROKEN_NET
			switch (deviceID)
			{
			case 0x87EB981E:
				if ((thisMsg->from == 0x30C2050B) || (thisMsg->from == 0x28B0495F))
				{
				}
				else
				{
					myLog_d("0x87EB981E connects only to 0x30C2050B & 0x28B0495F");
					return;
				}
				break;
			case 0x30C2050B:
				if ((thisMsg->from == 0x84E26CBF) || (thisMsg->from == 0x87EB981E))
				{
				}
				else
				{
					myLog_d("0x30C2050B connects only to 0x87EB981E & 0x84E26CBF");
					return;
				}
				break;
			case 0x28B0495F:
				if ((thisMsg->from == 0x30C2050B) || (thisMsg->from == 0x87EB981E))
				{
				}
				else
				{
					myLog_d("0x28B0495F connects only to 0x87EB981E & 0x30C2050B");
					return;
				}
				break;
			case 0x84E26CBF:
				if ((thisMsg->from == 0x0C666CBF) || (thisMsg->from == 0x87EB981E) || (thisMsg->from == 0x28B0495F))
				{
					myLog_d("No connection from 0x84E26CBF to 0x87EB981E & 0x0C666CBF & 0x28B0495F");
					return;
				}
				break;
			case 0x0C666CBF:
				if ((thisMsg->from == 0x84E26CBF) || (thisMsg->from == 0x87EB981E) || (thisMsg->from == 0x30C2050B) || (thisMsg->from == 0x28B0495F))
				{
					myLog_d("No connection from 0x0C666CBF to 0x87EB981E & 0x84E26CBF & 0x30C2050B & 0x28B0495F");
					return;
				}
				break;
			default:
				if ((thisMsg->from == 0x30C2050B) || (thisMsg->from == 0x87EB981E) || (thisMsg->from == 0x28B0495F))
				{
					myLog_d("No connection from 0x87EB981E & 0x30C2050B & 0x28B0495F to any other node");
					return;
				}
				break;
			}
#endif
			myLog_d("Got map message");
			// Mapping received
			uint8_t subsSize = tempSize - MAP_HEADER_SIZE;
			uint8_t numSubs = subsSize / 5;

			// Serial.println("********************************");
			// for (int idx = 0; idx < tempSize; idx++)
			// {
			// 	Serial.printf("%02X ", rxBuffer[idx]);
			// }
			// Serial.println("");
			// Serial.printf("subsSize %d -> # subs %d\n", subsSize, subsSize / 5);
			// Serial.println("********************************");

			// Check if end marker is in the message
			if ((thisMsg->nodes[numSubs - 1][0] != 0xAA) ||
				(thisMsg->nodes[numSubs - 1][1] != 0x55) ||
				(thisMsg->nodes[numSubs - 1][2] != 0x00) ||
				(thisMsg->nodes[numSubs - 1][3] != 0xFF) ||
				(thisMsg->nodes[numSubs - 1][4] != 0xAA))
			{
				myLog_e("Invalid map, end marker is missing from %08X", thisMsg->from);
				xSemaphoreGive(accessNodeList);
				return;
			}
			if (xSemaphoreTake(accessNodeList, (TickType_t)1000) == pdTRUE)
			{
				nodesChanged = addNode(thisMsg->from, 0, 0);

				// if (!checkValidId(thisMsg->from))
				// {
				// 	Serial.printf("** INVALID DIRECT ID: %08X **\n", thisMsg->from);
				// 	for (int idx = 0; idx < tempSize; idx++)
				// 	{
				// 		Serial.printf("%02X ", rxBuffer[idx]);
				// 	}
				// 	Serial.println("");
				// }
				// Remove nodes that use sending node as hop
				clearSubs(thisMsg->from);

				myLog_v("From %08X", thisMsg->from);
				myLog_v("Dest %08X", thisMsg->dest);

				if (subsSize != 0)
				{
					// Mapping contains subs

					myLog_v("Msg size %d", tempSize);
					myLog_v("#subs %d", numSubs);

					// Serial.println("++++++++++++++++++++++++++++");
					// Serial.printf("From %08X Dest %08X #Subs %d\n", thisMsg->from, thisMsg->dest, numSubs);
					// for (int idx = 0; idx < numSubs; idx++)
					// {
					// 	uint32_t subId = (uint32_t)thisMsg->nodes[idx][0];
					// 	subId += (uint32_t)thisMsg->nodes[idx][1] << 8;
					// 	subId += (uint32_t)thisMsg->nodes[idx][2] << 16;
					// 	subId += (uint32_t)thisMsg->nodes[idx][3] << 24;
					// 	uint8_t hops = thisMsg->nodes[idx][4];
					// 	Serial.printf("ID: %08X numHops: %d\n", subId, hops);
					// }
					// Serial.println("++++++++++++++++++++++++++++");

					for (int idx = 0; idx < numSubs - 1; idx++)
					{
						uint32_t subId = (uint32_t)thisMsg->nodes[idx][0];
						subId += (uint32_t)thisMsg->nodes[idx][1] << 8;
						subId += (uint32_t)thisMsg->nodes[idx][2] << 16;
						subId += (uint32_t)thisMsg->nodes[idx][3] << 24;
						uint8_t hops = thisMsg->nodes[idx][4];
						if (subId != deviceID)
						{
							// if (!checkValidId(subId))
							// {
							// 	Serial.printf("***** INVALID SUB ID: %08X *****\n", subId);
							// 	for (int idx = 0; idx < tempSize; idx++)
							// 	{
							// 		Serial.printf("%02X ", rxBuffer[idx]);
							// 	}
							// 	Serial.println("");
							// }
							nodesChanged |= addNode(subId, thisMsg->from, hops + 1);
							myLog_v("Subs %08X", subId);
						}
					}
				}
				xSemaphoreGive(accessNodeList);
			}
			else
			{
				myLog_e("Could not access map to add node");
			}
		}
		else if (thisDataMsg->type == 1)
		{
			if (thisDataMsg->dest == deviceID)
			{
				// Message is for us, call user callback to handle the data
				myLog_w("Got data message %s", (char *)thisDataMsg->data);
				if ((_MeshEvents != NULL) && (_MeshEvents->DataAvailable != NULL))
				{
					// Serial.println("Forwarding message");
					_MeshEvents->DataAvailable(thisDataMsg->data, tempSize - 12, rxRssi, rxSnr);
				}
			}
			else
			{
				// Message is not for us
			}
		}
		else if (thisDataMsg->type == 2)
		{
			if (thisDataMsg->dest == deviceID)
			{
				// Message is for sub node, forward the message
				nodesList route;
				if (xSemaphoreTake(accessNodeList, (TickType_t)1000) == pdTRUE)
				{
					if (getRoute(thisDataMsg->from, &route))
					{
						// We found a route, send package to next hop
						if (route.firstHop == 0)
						{
							myLog_i("Route for %lX is direct", route.nodeId);
							// Destination is a direct
							thisDataMsg->dest = thisDataMsg->from;
							thisDataMsg->from = deviceID;
							thisDataMsg->type = 1;
						}
						else
						{
							myLog_i("Route for %lX is to %lX", route.nodeId, route.firstHop);
							// Destination is a sub
							thisDataMsg->dest = route.firstHop;
							thisDataMsg->type = 2;
						}

						// Put message into send queue
						if (!addSendRequest(thisDataMsg, tempSize))
						{
							myLog_e("Cannot forward message because send queue is full");
						}
					}
					else
					{
						myLog_e("No route found for %lX", thisMsg->from);
					}
					xSemaphoreGive(accessNodeList);
				}
				else
				{
					myLog_e("Could not access map to forward package");
				}
			}
			else
			{
				// Message is not for us
			}
		}
	}
	else
	{
		myLog_e("Invalid package");
		for (int idx = 0; idx < tempSize; idx++)
		{
			Serial.printf("%02X ", rxBuffer[idx]);
		}
		Serial.println("");
	}
}

/**
 * Callback after a package was successfully sent
 */
void OnTxDone(void)
{
	myLog_w("OnTxDone");
	loraState = MESH_IDLE;

	// Restart listening
	Radio.Standby();
	Radio.Rx(0);
	// Radio.SetRxDutyCycle(RX_SLEEP_TIMES);
}

/**
 * Callback if sending a package timed out
 * Not sure what triggers that, it never occured
 */
void OnTxTimeout(void)
{
	myLog_e("OnTxTimeout");
	loraState = MESH_IDLE;

	// Restart listening
	Radio.Standby();
	Radio.Rx(0);
	// Radio.SetRxDutyCycle(RX_SLEEP_TIMES);
}

/**
 * Callback if sending a package timed out
 * Not sure what triggers that, it never occured
 */
void OnTxTimerTimeout(void)
{
	myLog_e("OnTxTimerTimeout");
	loraState = MESH_IDLE;

	// Internal timer timeout, maybe some problem with SX126x ???
	Radio.Standby();

	OnTxTimeout();
}

/**
 * Callback if waiting for a package timed out
 */
void OnRxTimeout(void)
{
	myLog_e("OnRxTimeout");

	if (loraState != MESH_TX)
	{
		loraState = MESH_IDLE;

		// Restart listening
		Radio.Standby();
		Radio.Rx(0);
		// Radio.SetRxDutyCycle(RX_SLEEP_TIMES);}
	}
}

/**
 * Callback if waiting for a package timed out
 */
void OnRxTimerTimeout(void)
{
	myLog_e("OnRxTimerTimeout");
	if (loraState != MESH_TX)
	{
		loraState = MESH_IDLE;

		// Internal timer timeout, maybe some problem with SX126x ???
		Radio.Standby();

		OnRxTimeout();
	}
}

/**
 * Callback if waiting for a package failed 
 * Not sure what triggers that, it never occured
 */
void OnRxError(void)
{
	myLog_e("OnRxError");
	if (loraState != MESH_TX)
	{
		loraState = MESH_IDLE;

		// Restart listening
		Radio.Standby();
		Radio.Rx(0);
		// Radio.SetRxDutyCycle(RX_SLEEP_TIMES);
	}
}

/**
 * Callback if a preamble is detected during RX 
 * That means a package should be coming in
 */
void OnPreAmbDetect(void)
{
	myLog_d("OnPreAmbDetect");
	// Put LoRa modem state into RX as a message is coming in
	loraState = MESH_RX;
	preambTimeout = millis();
}

/**
 * Callback if the Channel Activity Detection has finished
 * Starts sending the ACK package if the channel is available
 * Retries 5 times if the channel was busy.
 * Returns to listen mode if the channel was occupied 5 times
 * 
 * @param cadResult
 * 		True if channel activity was detected
 * 		False if the channel is available
 */
void OnCadDone(bool cadResult)
{
	// Not used
	if (cadResult)
	{
		myLog_d("CAD returned channel busy");
		channelFreeRetryNum++;
		if (channelFreeRetryNum >= CAD_RETRY)
		{
			myLog_e("CAD returned channel busy %d times, giving up", CAD_RETRY);
			loraState = MESH_IDLE;
			// Restart listening
			Radio.Standby();
			Radio.Rx(0);
			// Radio.SetRxDutyCycle(RX_SLEEP_TIMES);
		}
		else
		{
			Radio.Standby();
			Radio.StartCad();
		}
	}
	else
	{
		myLog_d("CAD returned channel free");
		myLog_d("Sending %d bytes", txLen);

		// Send the data package
		Radio.Standby();
		Radio.Send((uint8_t *)&txPckg, txLen);
	}
}

/**
 * Add a data package to the queue
 * @param package
 * 			dataPckg * to the sensor data
 * @return result
 * 			TRUE if task could be added to queue
 * 			FALSE if queue is full or not initialized 
 */
bool addSendRequest(dataMsg *package, uint8_t msgSize)
{
	if (sendQueue != NULL)
	{
#ifdef ESP32
		portENTER_CRITICAL(&accessMsgQueue);
#else
		taskENTER_CRITICAL();
#endif
		// Find unused entry in queue list
		int next = SEND_QUEUE_SIZE;
		for (int idx = 0; idx < SEND_QUEUE_SIZE; idx++)
		{
			if (sendMsg[idx].type == 0)
			{
				next = idx;
				break;
			}
		}

		if (next != SEND_QUEUE_SIZE)
		{
			// Found an empty entry!
			memcpy(&sendMsg[next], package, msgSize);
			sendMsgSize[next] = msgSize;

			myLog_d("Queued msg #%d with len %d", next, msgSize);

			// Try to add to cloudTaskQueue
			if (xQueueSend(sendQueue, &next, (TickType_t)1000) != pdTRUE)
			{
				myLog_e("Send queue is busy");
#ifdef ESP32
				portEXIT_CRITICAL(&accessMsgQueue);
#else
				taskEXIT_CRITICAL();
#endif
				return false;
			}
			else
			{
				myLog_v("Send request queued:");
#ifdef ESP32
				portEXIT_CRITICAL(&accessMsgQueue);
#else
				taskEXIT_CRITICAL();
#endif
				return true;
			}
		}
		else
		{
			myLog_e("Send queue is full");
			// Queue is already full!
#ifdef ESP32
			portEXIT_CRITICAL(&accessMsgQueue);
#else
			taskEXIT_CRITICAL();
#endif
			return false;
		}
	}
	else
	{
		myLog_e("Send queue not yet initialized");
		return false;
	}
}
