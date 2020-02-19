#include "main.h"

/** The list with all known nodes */
nodesList *nodesMap;

/** Timeout to remove unresponsive nodes */
time_t inActiveTimeout = 120000;

/**
 * Delete a node route by copying following routes on top of it.
 * @param index
 * 		The node to be deleted
 */
void deleteRoute(uint8_t index)
{
	// Delete a route by copying following routes on top of it
	memcpy(&nodesMap[index], &nodesMap[index + 1],
		   sizeof(nodesList) * (_numOfNodes - index - 1));
	nodesMap[_numOfNodes - 1].nodeId = 0;
}

/**
 * Find a route to a node
 * @param id
 * 		Node ID we need a route to
 * @param route
 * 		nodesList struct that will be filled with the route
 * @return bool
 * 		True if a route was found, false if not
 */
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

/** 
 * Add a node into the list.
 * Checks if the node already exists and
 * removes node if the existing entry has more hops
 */
boolean addNode(uint32_t id, uint32_t hop, uint8_t hopNum)
{
	boolean listChanged = false;
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
				return listChanged;
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
						return listChanged;
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
		listChanged = true;
	}

	// New node entry
	memcpy(&nodesMap[newEntry], &_newNode, sizeof(nodesList));
	listChanged = true;
	myLog_w("Added node %lX with hop %lX and num hops %d", id, hop, hopNum);
	return listChanged;
}

/**
 * Remove all nodes that are a non direct and have a given node as first hop.
 * This is to clean up the nodes list from left overs of an unresponsive node
 * @param id
 * 		The node which is listed as first hop
 */
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

/**
 * Check the list for nodes that did not be refreshed within a given timeout
 * @return bool
 * 			True if no changes were done, false if any node was removed
 */
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

/**
 * Create a list of nodes and hops to be broadcasted as this nodes map
 * @param subs[]
 * 		Pointer to an array to hold the node IDs
 * @param hops[]
 * 		Pointer to an array to hold the hops for the node IDs
 * @return uint8_t
 * 		Number of nodes in the list
 */
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

/**
 * Create a list of nodes and hops to be broadcasted as this nodes map
 * @param nodes[]
 * 		Pointer to an two dimensional array to hold the node IDs and hops
 * @return uint8_t
 * 		Number of nodes in the list
 */
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

/**
 * Get number of nodes in the map
 * @return uint8_t
 * 		Number of nodes
 */
uint8_t numOfNodes(void)
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

/**
 * Get the information of a specific node
 * @param nodeNum
 * 		Index of the node we want to query
 * @param nodeId
 * 		Pointer to an uint32_t to save the node ID to
 * @param firstHop
 *		Pointer to an uint32_t to save the nodes first hop ID to
 * @param numHops
 * 		Pointer to an uint8_t to save the number of hops to
 * @return bool
 * 		True if the data could be found, false if the requested index is out of range
 */
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
