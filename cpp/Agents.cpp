#include "Agents.h"
#include "global.h"


int main( int argc, char *argv[] ){
    Agents agents;
    agents.readMapFromFile("/mnt/Topics/Learning/MAPF/LocalHeuristics/map/random-32-32-10.map");
    agents.taskAllocation();
    agents.findPath();


    return 0;
}