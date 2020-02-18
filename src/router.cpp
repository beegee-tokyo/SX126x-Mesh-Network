#include "main.h"

nodesList *nodesMap;

time_t inActiveTimeout = 120000;

void deleteRoute(uint8_t index)
{
	// Delete a route by copying following routes on top of it
	memcpy(&nodesMap[index], &nodesMap[index + 1],
		   sizeof(nodesList) * (_numOfNodes - index - 1));
	nodesMap[_numOfNodes - 1].nodeId = 0;
}

bool getRoute(uint32_t id, nodesList *route)
{
	for (int idx = 0; idx < _numOfNodes; idx++)
	{
		if (nodesMap[idx].nodeId == id)
		{
			route->firstHop = nodesMap[idx].firstHop;
			route->nodeId = nodesMap[idx].nodeId;
			// Node found in map
			return true;
		}
	}
	// Node not in map
	return false;
}

void addNode(uint32_t id, uint32_t hop, uint8_t hopNum)
{
	nodesList _newNode;
	_newNode.nodeId = id;
	_newNode.firstHop = hop;
	_newNode.timeStamp = millis();
	_newNode.numHops = hopNum;

	for (int idx = 0; idx < _numOfNodes; idx++)
	{
		if (nodesMap[idx].nodeId == id)
		{
			if (nodesMap[idx].firstHop == 0)
			{
				if (hop == 0)
				{ // Node entry exist already as direct, update timestamp
					nodesMap[idx].timeStamp = millis();
				}
				myLog_w("Node %08X already exists as direct", _newNode.nodeId);
				return;
			}
			else
			{
				if (hop == 0)
				{
					// Found the node, but not as direct neighbor
					myLog_w("Node %08X removed because it was a sub", _newNode.nodeId);
					deleteRoute(idx);
					idx--;
					break;
				}
				else
				{
					// Node entry exists, check number of hops
					if (nodesMap[idx].numHops < hopNum)
					{
						// Node entry exist with smaller # of hops
						myLog_w("Node %08X exist with a lower number of hops", _newNode.nodeId);
						return;
					}
					else
					{
						// Found the node, but with higher # of hops
						myLog_w("Node %08X exist with a higher number of hops", _newNode.nodeId);
						deleteRoute(idx);
						idx--;
						break;
					}
				}
			}
		}
	}

	// Find unused node entry
	int newEntry = 0;
	for (newEntry = 0; newEntry < _numOfNodes; newEntry++)
	{
		if (nodesMap[newEntry].nodeId == 0)
		{
			// Found a new entry
			break;
		}
	}

	if (newEntry == _numOfNodes)
	{
		// Map is full, remove oldest entry
		deleteRoute(0);
	}

	// New node entry
	memcpy(&nodesMap[newEntry], &_newNode, sizeof(nodesList));
myLog_w("Added node %lX with hop %lX and num hops %d", id, hop, hopNum);
}

void clearSubs(uint32_t id)
{
	for (int idx = 0; idx < _numOfNodes; idx++)
	{
		if (nodesMap[idx].firstHop == id)
		{
			myLog_d("Removed node %lX with hop %lX", nodesMap[idx].nodeId, nodesMap[idx].firstHop);
			deleteRoute(idx);
			idx--;
		}
	}
}

bool cleanMap(void)
{
	// Check active nodes list
	bool mapUpToDate = true;
	for (int idx = 0; idx < _numOfNodes; idx++)
	{
		if (nodesMap[idx].nodeId == 0)
		{
			// Last entry found
			break;
		}
		time_t now = millis();
		myLog_w("ID: %08lX Timestamp: %lld, millis(): %lld", nodesMap[idx].nodeId, nodesMap[idx].timeStamp, now);
		if ((millis() > (nodesMap[idx].timeStamp + inActiveTimeout)))
		{
			// Node was not refreshed for inActiveTimeout milli seconds
			myLog_w("Node %lX with hop %lX timed out", nodesMap[idx].nodeId, nodesMap[idx].firstHop);
			if (nodesMap[idx].firstHop == 0)
			{
				clearSubs(nodesMap[idx].nodeId);
			}
			deleteRoute(idx);
			idx--;
			mapUpToDate = false;
		}
	}
	return mapUpToDate;
}

uint8_t nodeMap(uint32_t subs[], uint8_t hops[])
{
	uint8_t subsNameIndex = 0;

	for (int idx = 0; idx < _numOfNodes; idx++)
	{
		if (nodesMap[idx].nodeId == 0)
		{
			// Last node found
			break;
		}
		hops[subsNameIndex] = nodesMap[idx].numHops;

		subs[subsNameIndex] = nodesMap[idx].nodeId;
		subsNameIndex++;
	}

	return subsNameIndex;
}

uint8_t nodeMap(uint8_t nodes[][5])
{
	uint8_t subsNameIndex = 0;

	for (int idx = 0; idx < _numOfNodes; idx++)
	{
		if (nodesMap[idx].nodeId == 0)
		{
			// Last node found
			break;
		}
		nodes[subsNameIndex][0] = nodesMap[idx].nodeId & 0x000000FF;
		nodes[subsNameIndex][1] = (nodesMap[idx].nodeId >> 8) & 0x000000FF;
		nodes[subsNameIndex][2] = (nodesMap[idx].nodeId >> 16) & 0x000000FF;
		nodes[subsNameIndex][3] = (nodesMap[idx].nodeId >> 24) & 0x000000FF;
		nodes[subsNameIndex][4] = nodesMap[idx].numHops;

		subsNameIndex++;
	}

	return subsNameIndex;
}

uint8_t numOfNodes()
{
	uint8_t subsNameIndex = 0;

	for (int idx = 0; idx < _numOfNodes; idx++)
	{
		if (nodesMap[idx].nodeId == 0)
		{
			// Last node found
			break;
		}
		subsNameIndex++;
	}

	return subsNameIndex;
}

bool getNode(uint8_t nodeNum, uint32_t &nodeId, uint32_t &firstHop, uint8_t &numHops)
{
	if (nodeNum >= numOfNodes())
	{
		return false;
	}

	nodeId = nodesMap[nodeNum].nodeId;
	firstHop = nodesMap[nodeNum].firstHop;
	numHops = nodesMap[nodeNum].numHops;
	return true;
}
