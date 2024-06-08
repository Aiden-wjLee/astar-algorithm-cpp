#include "MapSearchNode.h"
#include "global.h"

#include <boost/functional/hash.hpp>
#include <tuple>
#include <unordered_map>
#include <vector>

int MAP_WIDTH, MAP_HEIGHT;
int* world_map;
struct CollisionInfo {
    int time;
    int x, y;
    int agent1, agent2;
    CollisionInfo(int time, int x, int y, int agent1, int agent2)
        : time(time), x(x), y(y), agent1(agent1), agent2(agent2) {}
};
class Agents {
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
    Agents(int num, string map_file_path)
        : agent_num(num), map_file_path(map_file_path), paths(num), search_count(0), search_state(0), search_steps(0) {
        startNodes = new MapSearchNode[agent_num];
        goalNodes = new MapSearchNode[agent_num];
    }
    int agent_num = 5;
    string map_file_path;

    MapSearchNode* startNodes;
    MapSearchNode* goalNodes;
    int node_x, node_y;
    vector<vector<pair<int, int>>> paths;
    vector<CollisionInfo> collisions;

    void taskAllocation();
    void setNode(int start_x, int start_y, int goal_x, int goal_y, int agent_num);
    void readMapFromFile();
    void findPath();
    void checkCollision();
    void savePathAll();
    void savePathAgent(int agent_index);
    void saveCollision();

    ~Agents() {
        delete[] startNodes;
        delete[] goalNodes;
    }

   private:
    unsigned int search_count = 0;
    const unsigned int num_searches = 1;
    unsigned int search_state;
    unsigned int search_steps = 0;
};
void Agents::saveCollision() {
    ofstream collisionFile;
    collisionFile.open("./collision.txt");
    collisionFile << "Agent Num : " << agent_num << endl;
    collisionFile << "Map File Path : " << map_file_path << endl << endl;
    for (int i = 0; i < collisions.size(); i++) {
        collisionFile << "Time : " << collisions[i].time << "  Location : [" << collisions[i].x << ","
                      << collisions[i].y << "]" << endl;
        collisionFile << "Agent " << collisions[i].agent1 << " and Agent " << collisions[i].agent2 << " collided"
                      << endl;
    }
    collisionFile.close();
}
void Agents::savePathAll() {
    ofstream pathFile;
    pathFile.open("./all.txt");
    pathFile << "Agent Num : " << agent_num << endl;
    pathFile << "Map File Path : " << map_file_path << endl << endl;
    for (int i = 0; i < agent_num; i++) {
        pathFile << "Agent " << i << endl
                 << "Start[x,y] : [" << startNodes[i].x << "," << startNodes[i].y << "]  Goal[x,y] : ["
                 << goalNodes[i].x << "," << goalNodes[i].y << "]" << endl;
        for (int j = 0; j < paths[i].size(); j++) {
            pathFile << paths[i][j].first << " " << paths[i][j].second << endl;
        }
        pathFile << endl;
    }
    pathFile.close();
}
void Agents::savePathAgent(int agent_index) {
    ofstream pathFile;
    pathFile.open("./" + to_string(agent_index) + ".txt");
    for (int j = 0; j < paths[agent_index].size(); j++) {
        pathFile << paths[agent_index][j].first << " " << paths[agent_index][j].second << endl;
    }
    pathFile.close();
}
void Agents::checkCollision() {
    // 시간과 위치를 저장하는 3차원 벡터
    vector<vector<vector<vector<int>>>> position_map; // time, x, y, agent_num
    int max_time = 0;
    int max_x = 0;
    int max_y = 0;

    // 맵의 크기 결정 (최대 시간과 좌표 계산)
    for (int i = 0; i < agent_num; ++i) {
        for (int j = 0; j < paths[i].size(); ++j) {
            auto pos = paths[i][j];
            if (j > max_time)
                max_time = j;
            if (pos.first > max_x)
                max_x = pos.first;
            if (pos.second > max_y)
                max_y = pos.second;
        }
    }

    // 크기에 맞게 벡터 초기화
    position_map.resize(max_time + 1, vector<vector<vector<int>>>(max_x + 1, vector<vector<int>>(max_y + 1)));

    // 각 에이전트의 모든 경로를 맵에 추가
    for (int i = 0; i < agent_num; ++i) {
        for (int j = 0; j < paths[i].size(); ++j) {
            auto pos = paths[i][j];
            position_map[j][pos.first][pos.second].push_back(i);
        }
    }

    // 충돌 여부 확인
    for (int t = 0; t <= max_time; ++t) {
        for (int x = 0; x <= max_x; ++x) {
            for (int y = 0; y <= max_y; ++y) {
                if (position_map[t][x][y].size() > 1) {
                    std::cout << "Collision detected at time " << t << " at (" << x << ", " << y << ")\n";
                    for (size_t i = 0; i < position_map[t][x][y].size(); ++i) {
                        for (size_t j = i + 1; j < position_map[t][x][y].size(); ++j) {
                            int agent1 = position_map[t][x][y][i];
                            int agent2 = position_map[t][x][y][j];
                            collisions.emplace_back(t, x, y, agent1, agent2);
                            std::cout << "Agent " << agent1 << " and Agent " << agent2 << " collided at time " << t
                                      << " at (" << x << ", " << y << ")\n";
                        }
                    }
                }
            }
        }
    }
}
void Agents::findPath() {
    for (int i = 0; i < agent_num; i++) {
        AStarSearch<MapSearchNode> astarSearch;
        astarSearch.SetStartAndGoalStates(startNodes[i], goalNodes[i]);
        search_count = 0;
        while (search_count < num_searches) {
            search_steps = 0;
            do {
                search_state = astarSearch.SearchStep();
                search_steps++;
            } while (search_state == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

            if (search_state == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) {
                cout << "Search found goal state\n";

                MapSearchNode* node = astarSearch.GetSolutionStart();
                int steps = 0;
                node->PrintNodeInfo();
                node->GetNodeInfo(&node_x, &node_y);
                paths[i].emplace_back(node_x, node_y);
                for (;;) {
                    node = astarSearch.GetSolutionNext();
                    if (!node) {
                        break;
                    }
                    node->PrintNodeInfo();
                    node->GetNodeInfo(&node_x, &node_y);
                    paths[i].emplace_back(node_x, node_y);
                    steps++;
                };
                cout << "Solution steps " << steps << endl;
                astarSearch.FreeSolutionNodes();
            } else if (search_state == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED) {
                cout << "Search terminated. Did not find goal state\n";
            }
            cout << "SearchSteps : " << search_steps << "\n";
            search_count++;
            astarSearch.EnsureMemoryFreed();
        }
    }
}
void Agents::taskAllocation() {
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
    setNode(15, 15, 31, 31, 4);
}
void Agents::setNode(int start_x, int start_y, int goal_x, int goal_y, int agent_num) {
    startNodes[agent_num].x = start_x;
    startNodes[agent_num].y = start_y;
    goalNodes[agent_num].x = goal_x;
    goalNodes[agent_num].y = goal_y;
}

void Agents::readMapFromFile() {
    ifstream mapFile(map_file_path);
    if (!mapFile.is_open()) {
        cout << "Error opening file: " << map_file_path << endl;
        return;
    }

    string line;
    int width, height;
    while (getline(mapFile, line)) {
        if (line.find("height") != string::npos) {
            cout << line << endl;
            cout << line.find(" ") << endl;
            cout << line.substr(line.find(" ") + 1) << endl;
            width = stoi(line.substr(line.find(" ") + 1));
        } else if (line.find("width") != string::npos) {
            height = stoi(line.substr(line.find(" ") + 1));
        } else if (line.find("map") != string::npos) {
            break;
        }
    }
    MAP_WIDTH = width;
    MAP_HEIGHT = height;
    world_map = new int[MAP_WIDTH * MAP_HEIGHT];

    int row = 0;
    while (getline(mapFile, line) && row < MAP_HEIGHT) {
        for (int col = 0; col < MAP_WIDTH; col++) {
            if (line[col] == '@')
                world_map[row * (MAP_WIDTH) + col] = 9;
            else if (line[col] == '.')
                world_map[row * (MAP_WIDTH) + col] = 1;
            else
                world_map[row * (MAP_WIDTH) + col] = 0;
        }
        row++;
    }
    mapFile.close();
}