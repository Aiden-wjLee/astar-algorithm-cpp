
#ifndef MAPSEARHNODE_H
#define MAPSEARCHNODE_H
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
#include <fstream>
#include <vector>

using namespace std;

#include "global.h"
#include "stlastar.h" // See header for copyright and usage information

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

class MapSearchNode
{
public:
	int x;	 // the (x,y) positions of the node
	int y;	
	
	MapSearchNode() { x = y = 0; }
	MapSearchNode( int px, int py ) { x=px; y=py; }

	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	float GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &rhs );
	size_t Hash();
	int GetMap( int x, int y );

	void PrintNodeInfo(); 
	void GetNodeInfo(int *x, int *y);

};


#endif