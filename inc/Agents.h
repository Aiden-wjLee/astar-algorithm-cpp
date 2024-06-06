#include "MapSearchNode.h"
#include "global.h"

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
        int agent_num=5;
        MapSearchNode* startNodes = new MapSearchNode[agent_num];
        MapSearchNode* goalNodes = new MapSearchNode[agent_num];

        
        unsigned int search_count = 0;
        const unsigned int num_searches = 1;

        void taskAllocation();
        void setNode(int start_x, int start_y, int goal_x, int goal_y, int agent_num);
        
        void readMapFromFile(string mapFilePath);
        void findPath();

        ~Agents(){
            delete[] startNodes;
            delete[] goalNodes;
        }
};
void Agents::findPath(){
    for (int i = 0; i < agent_num; i++){
        AStarSearch<MapSearchNode> astarSearch;
        astarSearch.SetStartAndGoalStates( startNodes[i], goalNodes[i] );
        while (search_count < num_searches){
            unsigned int search_state;
            unsigned int search_steps = 0;
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
                    for( ;; )
                    {
                        node = astarSearch.GetSolutionNext();
                        if ( !node )
                        {
                            break;
                        }
                        node -> PrintNodeInfo();
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
    setNode(0, 0, 20, 15, 0);
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