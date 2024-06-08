#include "Agents.h"
#include "global.h"

int main(int argc, char* argv[]) {
    int agent_num = 5;
    string map_file_path = "/mnt/Topics/Learning/MAPF/LocalHeuristics/map/random-32-32-10.map";
    Agents agents(agent_num, map_file_path);
    agents.readMapFromFile();
    agents.taskAllocation();
    agents.findPath();
    agents.checkCollision();

    agents.savePathAll();
    agents.saveCollision();
    return 0;
}