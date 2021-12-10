#include "Arduino.h"

struct mapMsg
{
	uint8_t mark1 = 'L';
	uint8_t mark2 = 'o';
	uint8_t mark3 = 'R';
	uint8_t type = 5;
	uint32_t dest = 0;
	uint32_t from = 0;
	uint8_t nodes[48][5];
};

struct dataMsg
{
	uint8_t mark1 = 'L';
	uint8_t mark2 = 'o';
	uint8_t mark3 = 'R';
	uint8_t type = 0;
	uint32_t dest = 0;
	uint32_t from = 0;
	uint32_t orig = 0;
	uint8_t data[243];
};

/**
 * Mesh callback functions
 */
typedef struct
{
	/**
     * Data available callback prototype.
     *
     * @param payload 
	 * 			Received buffer pointer
     * @param size    
	 * 			Received buffer size
     * @param rssi    
	 * 			RSSI value computed while receiving the frame [dBm]
     * @param snr     
	 * 			SNR value computed while receiving the frame [dB]
     *                     FSK : N/A ( set to 0 )
     *                     LoRa: SNR value in dB
     */
	void (*DataAvailable)(uint32_t fromID, uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

	/**
     * Nodes list change callback prototype.
     */
	void (*NodesListChanged)(void);

} MeshEvents_t;

// LoRa Mesh functions & variables
void initMesh(MeshEvents_t *events, int numOfNodes);
void meshTask(void *pvParameters);
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);
void OnPreAmbDetect(void);
void OnCadDone(bool cadResult);
bool addSendRequest(dataMsg *package, uint8_t msgSize);
extern TaskHandle_t meshTaskHandle;
extern volatile xQueueHandle meshMsgQueue;

/** Size of map message buffer without subnode */
#define MAP_HEADER_SIZE 12
/** Size of data message buffer without subnode */
#define DATA_HEADER_SIZE 16

/** Number of retries if CAD shows busy */
#define CAD_RETRY 20

// LoRa definitions
#define RF_FREQUENCY 916000000  // Hz
#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 1		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 5000
#define TX_TIMEOUT_VALUE 5000

struct nodesList
{
	uint32_t nodeId;
	uint32_t firstHop;
	time_t timeStamp;
	uint8_t numHops;
};

bool getRoute(uint32_t id, nodesList *route);
boolean addNode(uint32_t id, uint32_t hop, uint8_t numHops);
void removeNode(uint32_t id);
void clearSubs(uint32_t id);
bool cleanMap(void);
uint8_t nodeMap(uint32_t subs[], uint8_t hops[]);
uint8_t nodeMap(uint8_t nodes[][5]);
uint8_t numOfNodes();
bool getNode(uint8_t nodeNum, uint32_t &nodeId, uint32_t &firstHop, uint8_t &numHops);
uint32_t getNextBroadcastID(void);
bool isOldBroadcast(uint32_t broadcastID);

extern SemaphoreHandle_t accessNodeList;
extern nodesList *nodesMap;
extern int _numOfNodes;
