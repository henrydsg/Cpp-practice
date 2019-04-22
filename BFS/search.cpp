#include<iostream>
#include <utility>
#include <vector>
#include <queue>
#include <deque>
#include <math.h>
using namespace std;

vector<pair<int,int>> Random_search(vector<vector<int>> world_state,pair<int, int> robot_pose,pair<int, int> goal_pose);
vector<pair<int,int>> Random_get_legal_poses(vector<vector<int>> world_state,pair<int, int> robot_pose, deque<pair<int, int>> visited);
vector<pair<int,int>> BFS_optimal_search(vector<vector<int>> world_state,pair<int, int> robot_pose,pair<int, int> goal_pose);
vector<pair<int,int>> BFS_get_legal_poses(vector<vector<int>> world_state,pair<int, int> robot_pose);
bool Is_goal_pose(pair<int, int> robot_pose, pair<int, int> goal_pose);
bool BFS_is_visited(pair<int, int> robot_pose, vector<vector<bool>> visited);



int main(){
    /*test case 1
     vector<vector<int>> world_state = { {0,0,0,0,0,0},
     {0,0,0,0,0,0},
     {0,0,0,0,0,0},
     {0,0,0,0,0,0},
     {0,0,0,1,1,1},
     {0,0,0,1,0,0} };
     
     pair<int, int> robot_pose(4, 0), goal_pose(4, 1);
     */
    
    /*test case 2
    vector<vector<int>> world_state = { {} };
    pair<int, int> robot_pose(0, 0), goal_pose(0, 0);
    */
    
    /*test case 3
     vector<vector<int>> world_state = { {0} };
     pair<int, int> robot_pose(0, 0), goal_pose(0, 0);
     */
    
    /*test case 4
     vector<vector<int>> world_state = { {0,0} };
     pair<int, int> robot_pose(0, 0), goal_pose(0, 1);
     */
    
    /*test case 5
     vector<vector<int>> world_state = { {0, 0, 0, 0} };
     pair<int, int> robot_pose(0, 0), goal_pose(0, 3);
     */
    
    //test case 6
     vector<vector<int>> world_state = { {1, 0, 0, 1},
     {0, 0, 0, 0} };
     pair<int, int> robot_pose(1, 0), goal_pose(1, 3);
    
    
    
    srand(time(0)); // sample the rand() function seed.
    
    
    // Execute random search and print the solution
    // Time Complexity: O(max_step_number*sqrt(max_step_number))
    vector<pair<int,int>> Random_path = Random_search(world_state,robot_pose,goal_pose);
    
    if(Random_path.size())
        cout << "The random planner's solution is:" << endl;
    
    for(auto pose : Random_path){
        cout << "{" << pose.first << ", " << pose.second << "}, ";
    }
    
    cout << endl;
    
    // Execute random search and print the solution
    // Time Complexity: O(size of the world poses)
    vector<pair<int,int>> BFS_path = BFS_optimal_search(world_state,robot_pose,goal_pose);
    
    if(BFS_path.size())
        cout << "The optimal planner's solution is:" << endl;
    
    for(auto pose : BFS_path){
        cout << "{" << pose.first << ", " << pose.second << "}, ";
    }
    cout << endl;
    
    /*
     The random search is a bad algorithm and robot can easily get stuck due to the rule that shot-term visited state can't be visited again. and BFS is the best solver for this uninformed problem.
    */
}




/*
 
 Random_search find path by randomly picking next pose.
 
 INPUTS:
 vector<vector<int>> world_state: The grid representing the world's state.
 pair<int, int> robot_pose: Robot's start pose.
 pair<int, int> goal_pose: The goal pose.
 
 OUTPUT:
 vector<pair<int,int>> a poses sequence that representing the solution path.
 
 Time Complexity: O(max_step_number*sqrt(max_step_number)).
 
 */
vector<pair<int,int>> Random_search(vector<vector<int>> world_state,pair<int, int> robot_pose,pair<int, int> goal_pose){
    vector<pair<int,int>> path; // Variable for final path
    if(!world_state[0].size()){
        cout << "The random planner's world is null." << endl;
        return path;
    }
    // Check if start state valid
    if(robot_pose.first >= world_state.size() || robot_pose.second >= world_state[0].size() || world_state[robot_pose.first][robot_pose.second]){
        cout << "The start state is not valid." << endl;
        return path;
    }
    
    // Check if goal state valid
    if(goal_pose.first >= world_state.size() || goal_pose.second >= world_state[0].size() || world_state[goal_pose.first][goal_pose.second]){
        cout << "The goal state is not valid." << endl;
        return path;
    }
    int max_step_number = 100, max_mem_size = sqrt(max_step_number); // Max step number and max memorizing length of visited poses;
    deque<pair<int, int>> visited; // Short term memory of visited poses
    int step_count = 0; // counting steps
    
    // Run random search till a solution is found or steps exceed max step number or the algorithm walks into deadend caused by the rule that the short-term visted poses can't be visited.
    // Time Complexity O(max_step_number*sqrt(max_step_number)) and solution is not gauranteed
    while(step_count < max_step_number){
        path.push_back(robot_pose);
        
        // If the robot pose is the goal pose, then return the solution path.
        if(Is_goal_pose(robot_pose, goal_pose))
            return path;
        
        // Updating short-term visited poses
        if(visited.size() <  max_mem_size){
            visited.push_back(robot_pose);
        }
        else{
            visited.push_back(robot_pose);
            visited.pop_front();
        }
        
        
        vector<pair<int, int>> legal_poses = Random_get_legal_poses(world_state, robot_pose, visited); // get legal poses
        
        // If no legal pose, then it implies that the robot walks into a deadend
        if(legal_poses.empty()){
            cout << "The random planner has no legal movement available at" << "{" << robot_pose.first << ", " << robot_pose.second << "}"<< endl;
            return {};
        }
        
        robot_pose = legal_poses[rand() % legal_poses.size()]; // Pick a pose randomly.
        step_count++;
    }
    
    // No solution is found after max number of steps.
    cout << "There's no connection between start state and goal state or can't find solution within" << max_step_number<< "." << endl;
    return {};
}


/*
 
 Random_get_legal_poses is a function that helps Random_search find legal moves at specific robot_pose (X,Y).
 
 INPUTS:
 vector<vector<int>> world_state: The grid representing the world's state.
 pair<int, int> robot_pose: Robot's current pose.
 deque<pair<int, int>> visited: a deque that records the short-term visited poses.
 
 OUTPUT:
 vector<pair<int,int>> a poses vector that contains all legal poses.
 
 Time Complexity: O(sqrt(max_step_number))
 
 */
vector<pair<int,int>> Random_get_legal_poses(vector<vector<int>> world_state,pair<int, int> robot_pose, deque<pair<int, int>> visited){
    int m = world_state.size();
    int n = world_state[0].size();
    vector<pair<int,int>> legal_poses;
    deque<pair<int,int>>::iterator it;
    pair<int,int> pose(-1,-1);
    
    // Check if Up, Down, Left, Right, 4 legal poses candidates, are in valid point and if they are visited within sqrt(max_step_number), respectively.
    // Time complexity O(sqrt(max_step_number))
    if(robot_pose.first - 1 >= 0 && !world_state[robot_pose.first-1][robot_pose.second]){
        pose.first = robot_pose.first - 1;
        pose.second = robot_pose.second;
        it = find(visited.begin(),visited.end(),pose);
        if(it == visited.end()) legal_poses.push_back(pose);
    }
    if(robot_pose.first + 1 < m && !world_state[robot_pose.first+1][robot_pose.second]){
        pose.first = robot_pose.first + 1;
        pose.second = robot_pose.second;
        it = find(visited.begin(),visited.end(),pose);
        if(it == visited.end()) legal_poses.push_back(pose);
    }
    if(robot_pose.second - 1 >= 0 && !world_state[robot_pose.first][robot_pose.second-1]){
        pose.first = robot_pose.first;
        pose.second = robot_pose.second - 1;
        it = find(visited.begin(),visited.end(),pose);
        if(it == visited.end()) legal_poses.push_back(pose);
    }
    if(robot_pose.second + 1 < n && !world_state[robot_pose.first][robot_pose.second+1]){
        pose.first = robot_pose.first;
        pose.second = robot_pose.second + 1;
        it = find(visited.begin(),visited.end(),pose);
        if(it == visited.end()) legal_poses.push_back(pose);
    }
    return legal_poses;
}




/*
 
 BFS_optimal_search search optimal path using BFS algorithm.
 
 INPUTS:
 vector<vector<int>> world_state: The grid representing the world's state.
 pair<int, int> robot_pose: Robot's start pose.
 pair<int, int> goal_pose: The goal pose.
 
 OUTPUT:
 vector<pair<int,int>> a poses sequence that representing the optimal path.
 
 Time Complexity: O(m*n) where m*n is the size of the world poses
 
 */

vector<pair<int,int>> BFS_optimal_search(vector<vector<int>> world_state,pair<int, int> robot_pose,pair<int, int> goal_pose){
    vector<pair<int,int>> path; // Variable for final path
    
    if(!world_state[0].size()){
        cout << "The optimal planner's world is null." << endl;
        return path;
    }
    
    // Check if start state valid
    if(robot_pose.first >= world_state.size() || robot_pose.second >= world_state[0].size() || world_state[robot_pose.first][robot_pose.second]){
        cout << "The start state is not valid." << endl;
        return path;
    }
    
    // Check if goal state valid
    if(goal_pose.first >= world_state.size() || goal_pose.second >= world_state[0].size() || world_state[goal_pose.first][goal_pose.second]){
        cout << "The goal state is not valid." << endl;
        return path;
    }
    
    vector<vector<bool>> visited(world_state.size(), vector<bool>(world_state[0].size(),false)); // Array to memorize which states are visited
    queue<vector<pair<int, int>>> fringe; // Fringe to be expand
    
    // Setup initial conditions
    visited[robot_pose.first][robot_pose.second] = true;
    path.push_back(robot_pose);
    fringe.push(path);
    
    // Run BFS while memorizing all paths
    // Time Complexity O(m*n)
    while(!fringe.empty()){
        path = fringe.front();
        fringe.pop();
        if(Is_goal_pose(path[path.size()-1], goal_pose)) return path; // If optimal solution is found, return the path.
        robot_pose = path[path.size()-1];
        visited[robot_pose.first][robot_pose.second] = true;
        vector<pair<int, int>> legal_poses = BFS_get_legal_poses(world_state, robot_pose);
        
        for(auto legal_pose : legal_poses){
            if(!BFS_is_visited(legal_pose, visited)){
                vector<pair<int,int>> tmp_path = path;
                tmp_path.push_back(legal_pose);
                fringe.push(tmp_path);
            }
        }
    }
    
    // If there's no solution, return an empty vector;
    cout << "There's no connection between start state and goal state." << endl;
    return {};
}



/*
 
 BFS_get_legal_poses is a function that helps BFS_optimal_search find legal moves at specific robot_pose (X,Y).
 
 INPUTS:
 vector<vector<int>> world_state: The grid representing the world's state.
 pair<int, int> robot_pose: Robot's current pose.
 
 OUTPUT:
 vector<pair<int,int>> a poses vector that contains all legal poses.
 
 Time Complexity: O(sqrt(max_step_number)).
 
 */
vector<pair<int,int>> BFS_get_legal_poses(vector<vector<int>> world_state,pair<int, int> robot_pose){
    int m = world_state.size();
    int n = world_state[0].size();
    vector<pair<int,int>> legal_poses;
    
    // Check if Up, Down, Left, Right, 4 legal poses candidates, are in valid point and if they are visited within sqrt(max_step_number), respectively.
    // If legal, push into legal_poses
    // Time complexity O(sqrt(max_step_number))
    pair<int,int> pose(-1,-1);
    if(robot_pose.first - 1 >= 0 && !world_state[robot_pose.first-1][robot_pose.second]){
        pose.first = robot_pose.first - 1;
        pose.second = robot_pose.second;
        legal_poses.push_back(pose);
    }
    if(robot_pose.first + 1 < m && !world_state[robot_pose.first+1][robot_pose.second]){
        pose.first = robot_pose.first + 1;
        pose.second = robot_pose.second;
        legal_poses.push_back(pose);
    }
    if(robot_pose.second - 1 >= 0 && !world_state[robot_pose.first][robot_pose.second-1]){
        pose.first = robot_pose.first;
        pose.second = robot_pose.second - 1;
        legal_poses.push_back(pose);
    }
    if(robot_pose.second + 1 < n && !world_state[robot_pose.first][robot_pose.second+1]){
        pose.first = robot_pose.first;
        pose.second = robot_pose.second + 1;
        legal_poses.push_back(pose);
    }
    return legal_poses;
}




/*
 
 Is_goal_pose is a function that check if robot's current pose is equal to goal pose.
 
 INPUTS:
 pair<int, int> robot_pose: Robot's current pose.
 pair<int, int> goal_pose: The goal pose.
 
 OUTPUT:
 bool true if the robot is at goal pose and false for the opposite.
 
 Time Complexity: O(1)
 
 */
bool Is_goal_pose(pair<int, int> robot_pose, pair<int, int> goal_pose){
    // Time complexity O(1)
    if(robot_pose == goal_pose) return true;
    else return false;
}




/*
 
 BFS_is_visited is a function that check if robot's next pose is already visited.
 
 INPUTS:
 pair<int, int> robot_pose: Robot's next pose.
 vector<vector<bool>> visited: The 2-D vector that recording which poses are visited.
 
 OUTPUT:
 bool true if the pose is visited false for the opposite.
 
 Time Complexity: O(1)
 
 */
bool BFS_is_visited(pair<int, int> robot_pose, vector<vector<bool>> visited){
    // Time complexity O(1)
    if(visited[robot_pose.first][robot_pose.second]) return true;
    else return false;
}
