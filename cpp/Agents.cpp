#include "Agents.h"
#include "global.h"


int main( int argc, char *argv[] ){
    int agent_num=5;
    Agents agents(agent_num);
    agents.readMapFromFile("/mnt/Topics/Learning/MAPF/LocalHeuristics/map/random-32-32-10.map");
    agents.taskAllocation();
    agents.findPath();
    agents.checkCollision();
    agents.savePath();


    return 0;
}