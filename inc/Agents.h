#include "MapSearchNode.h"
#include "global.h"
#include <boost/functional/hash.hpp>

int MAP_WIDTH, MAP_HEIGHT;
int* world_map;
class Agents{
    /*
    각 agent들의 정보를 저장하는 클래스
    저장하는 정보:
        시작점, 도착점,
        경로,
        충돌 여부 (충돌 시, 충돌한 agent들의 번호, 충돌한 시간, 충돌한 위치)
    */
    public:
        // int MAP_WIDTH, MAP_HEIGHT;
        // int* world_map;
        Agents(int num) : agent_num(num), paths(num), search_count(0), search_state(0), search_steps(0) {
            startNodes = new MapSearchNode[agent_num];
            goalNodes = new MapSearchNode[agent_num];
        }
        int agent_num=5;
        MapSearchNode* startNodes;
        MapSearchNode* goalNodes;
        int node_x, node_y;
        vector<vector<pair<int, int>>> paths;

        void taskAllocation();
        void setNode(int start_x, int start_y, int goal_x, int goal_y, int agent_num);
        void readMapFromFile(string mapFilePath);
        void findPath();
        void checkCollision();
        void savePath();
        
        ~Agents(){
            delete[] startNodes;
            delete[] goalNodes;
        }
    private:
        unsigned int search_count = 0;
        const unsigned int num_searches = 1;
        unsigned int search_state;
        unsigned int search_steps = 0;
};
void Agents::savePath(){
    for (int i = 0; i < agent_num; i++){
        ofstream pathFile;
        pathFile.open("path" + to_string(i) + ".txt");
        for (int j = 0; j < paths[i].size(); j++){
            pathFile << paths[i][j].first << " " << paths[i][j].second << endl;
        }
        pathFile.close();
    }
}

void Agents::checkCollision() {
    // 시점별 위치를 저장하는 맵
    std::unordered_map<std::pair<int, int>, std::vector<int>, boost::hash<std::pair<int, int>>> position_map;

    // 각 에이전트의 모든 경로를 맵에 추가
    for (int i = 0; i < agent_num; ++i) {
        for (int j = 0; j < paths[i].size(); ++j) {
            auto pos = paths[i][j];
            position_map[pos].push_back(i);
        }
    }

    // 충돌 여부 확인
    for (const auto& entry : position_map) {
        if (entry.second.size() > 1) {
            std::cout << "Collision detected at (" << entry.first.first << ", " << entry.first.second << ")\n";
            for (int agent : entry.second) {
                std::cout << "Agent " << agent << " ";
            }
            std::cout << "collided at (" << entry.first.first << ", " << entry.first.second << ")\n";
        }
    }
}

void Agents::findPath(){
    for (int i = 0; i < agent_num; i++){
        AStarSearch<MapSearchNode> astarSearch;
        astarSearch.SetStartAndGoalStates( startNodes[i], goalNodes[i] );
        search_count = 0;
        while (search_count < num_searches){
            search_steps = 0;
            do
            {
                search_state = astarSearch.SearchStep();
                search_steps++;
            }while (search_state == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

                if ( search_state == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED)
                {
                    cout << "Search found goal state\n";
                    
                    MapSearchNode *node = astarSearch.GetSolutionStart();
                    int steps = 0; 
                    node -> PrintNodeInfo();
                    node -> GetNodeInfo( &node_x, &node_y );
                    paths[i].emplace_back(node_x, node_y);
                    for( ;; )
                    {
                        node = astarSearch.GetSolutionNext();
                        if ( !node )
                        {
                            break;
                        }
                        node -> PrintNodeInfo();
                        node -> GetNodeInfo( &node_x, &node_y );
                        paths[i].emplace_back(node_x, node_y);
                        steps ++;
                    };
                    cout << "Solution steps " << steps << endl;
                    astarSearch.FreeSolutionNodes();
                }
                else if (search_state == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED)
                {
                    cout << "Search terminated. Did not find goal state\n";
                }
                cout << "SearchSteps : " << search_steps << "\n";
                search_count++;
                astarSearch.EnsureMemoryFreed();
        }
    }
}
void Agents::taskAllocation(){
    /*
    각 agent들에게 할당된 task를 저장하는 함수
    */
    // for (int i = 0; i < agent_num; i++){
    //     setNode(0, 0, 0, 0, i);
    // }
    setNode(0, 0, 31, 31, 0);
    setNode(3, 3, 5, 15, 1);
    setNode(5, 5, 10, 10, 2);
    setNode(10, 10, 15, 5, 3);
}
void Agents::setNode(int start_x, int start_y, int goal_x, int goal_y, int agent_num){
    startNodes[agent_num].x = start_x;
    startNodes[agent_num].y = start_y;
    goalNodes[agent_num].x = goal_x;
    goalNodes[agent_num].y = goal_y;
}

void Agents::readMapFromFile(string mapFilepPath)
{
	ifstream mapFile(mapFilepPath);
	if(!mapFile.is_open())
	{
		cout << "Error opening file: " << mapFilepPath << endl;
		return ;
	}

	string line;
	int width, height;
	while (getline(mapFile, line))
	{
		if (line.find("height") != string::npos){
			cout<<line<<endl;
			cout<<line.find(" ")<<endl;
			cout<<line.substr(line.find(" ") + 1)<<endl;
			width = stoi(line.substr(line.find(" ") + 1));
		}
		else if (line.find("width") != string::npos){
			height = stoi(line.substr(line.find(" ") + 1));
		}
		else if (line.find("map") != string::npos){
			break;
		}
	}
	MAP_WIDTH = width;
	MAP_HEIGHT = height;
	world_map = new int[MAP_WIDTH * MAP_HEIGHT];

	int row = 0;
	while (getline(mapFile, line)&& row < MAP_HEIGHT) {
		for (int col =0; col < MAP_WIDTH; col++){
			if (line[col] =='@')
				world_map[row * (MAP_WIDTH) + col] = 9;
			else if (line[col] =='.')
				world_map[row * (MAP_WIDTH) + col] = 1;
			else 
				world_map[row * (MAP_WIDTH) + col] = 0;
		}
		row++;
	}
	mapFile.close();
}