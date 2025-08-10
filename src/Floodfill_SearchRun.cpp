// Micromouse floodfill maze-solving algorithm
// Compatible with MMS Simulator
// Written in C++ for robotics competitions
#include "API.h"
#include <array>
#include <queue>
#include <string>
#include <iostream>
#include <utility>
#include "Floodfill.h"
#include <vector>

using namespace std;
Floodfill floodfill;

enum Action { FORWARD, LEFT, RIGHT, IDLE, AROUND};

const int SIZE = 16;

int curRow = 0, curCol = 0, curDir = 0;  // 0=north,1=east,2=south,3=west

float getStat(const std::string& statName) {
    std::cout << "getStat " << statName << std::endl;
    std::string response;
    std::cin >> response;
    try {
        return std::stof(response); // handles both int and float
    } catch (...) {
        std::cerr << "Failed to parse stat: " << statName << std::endl;
        return -1;
    }
}

bool atGoal(int row, int col) {
    return (row == 7 || row == 8) && (col == 7 || col == 8);
}

void Floodfill::setWall(int row, int col, int direction) {
    if(direction == 0) {
        maze.vertical_walls[row][col].first = 1; // North wall
        if(row+1 < 16) maze.vertical_walls[row + 1][col].second = 1; // South wall of the cell above
    }
    if(direction == 1) {
        maze.horizontal_walls[row][col].second = 1; // East wall
        if(col + 1 < 16) maze.horizontal_walls[row][col + 1].first = 1; // West wall of the cell to the right
    }
    if(direction == 2) {
        maze.vertical_walls[row][col].second = 1; // South wall
        if(row - 1 >= 0) maze.vertical_walls[row - 1][col].first = 1; // North wall of the cell below
    }
    if(direction == 3) {
        maze.horizontal_walls[row][col].first = 1; // West wall
        if(col - 1 >= 0) maze.horizontal_walls[row][col - 1].second = 1; // East wall of the cell to the left
    }
    char cdir = "nesw"[direction];
    API::setWall(col, row, cdir);
}
void Floodfill::detectWalls(vector<int> sensorDistances, int row, int col, int direction){
    if(row < 0 || row >= 16 || col < 0 || col >= 16) return; // out of bounds
    if(API::wallFront()) { // North Wall
        setWall(row, col, direction);
        // Serial.print("Wall detected at ");
        // Serial.print(direction);
        // Serial.print(" with distance: ");
        // Serial.println(sensorDistances[2]);
    }
    if(API::wallLeft()) {
        setWall(row, col, (direction + 3) % 4);
        // Serial.print("Wall detected at ");
        // Serial.print((direction + 3) % 4);
        // Serial.print(" with distance: ");
        // Serial.println(sensorDistances[0]);
    }
    //if(sensorDistances[1] < wall_threshhold) setWall(row, col, direction);
    if(API::wallRight()) {
        setWall(row, col, (direction + 1) % 4);
        // Serial.print("Wall detected at ");
        // Serial.print((direction + 1) % 4);
        // Serial.print(" with distance: ");
        // Serial.println(sensorDistances[4]);
    }
}
bool Floodfill::hasWall(int row, int col, int dir) {
    if(dir == 0) return maze.vertical_walls[row][col].first; // North wall
    if(dir == 1) return maze.horizontal_walls[row][col].second; // East wall
    if(dir == 2) return maze.vertical_walls[row][col].second; // South wall
    if(dir == 3) return maze.horizontal_walls[row][col].first; // West wall
    return false; // No walls
}

void Floodfill::floodfill() {
    queue<pair<int, int>> q;    

    for (auto& row : maze.manhattan_distances)
        row.fill(255); // Re-initialize all distances to a large number

    maze.manhattan_distances[8][8] = 0; //row,column
    maze.manhattan_distances[8][7] = 0;
    maze.manhattan_distances[7][8] = 0;
    maze.manhattan_distances[7][7] = 0; // Goal
    
    q.push({8, 8}); q.push({8, 7}); q.push({7, 8}); q.push({7, 7});       

    array<array<bool, 16>, 16> reached = {};
    reached[8][8] = reached[8][7] = reached[7][8] = reached[7][7] = true; // Marking the goal cells as reached
    
    // BFS to fill the manhattan distances
    while(!q.empty()){
        pair<int,int> cell = q.front();
        q.pop();

        if(cell.first > 15 || cell.second > 15 || cell.first < 0 || cell.second < 0) continue; // out of bounds

        if(cell.second > 0 && !maze.horizontal_walls[cell.first][cell.second].first && !reached[cell.first][cell.second - 1]) {// No wall to the left
            maze.manhattan_distances[cell.first][cell.second - 1] = maze.manhattan_distances[cell.first][cell.second] + 1;
            q.push({cell.first, cell.second - 1});
            reached[cell.first][cell.second - 1] = true; // Marking the cell as reached            
        }
        if(cell.first < 15 && !maze.vertical_walls[cell.first][cell.second].first && !reached[cell.first + 1][cell.second]) {// No wall above
            maze.manhattan_distances[cell.first + 1][cell.second] = maze.manhattan_distances[cell.first][cell.second] + 1;
            q.push({cell.first + 1, cell.second});
            reached[cell.first + 1][cell.second] = true; // Marking the cell as reached
        }
        if(cell.second < 15 && !maze.horizontal_walls[cell.first][cell.second].second && !reached[cell.first][cell.second + 1]) {// No wall to the right
            maze.manhattan_distances[cell.first][cell.second + 1] = maze.manhattan_distances[cell.first][cell.second] + 1;
            q.push({cell.first, cell.second + 1});
            reached[cell.first][cell.second + 1] = true; // Marking the cell as reached
        }
        if(cell.first > 0 && !maze.vertical_walls[cell.first][cell.second].second && !reached[cell.first - 1][cell.second]) {// No wall below
            maze.manhattan_distances[cell.first - 1][cell.second] = maze.manhattan_distances[cell.first][cell.second] + 1;
            q.push({cell.first - 1, cell.second});
            reached[cell.first - 1][cell.second] = true; // Marking the cell as reached
        }
    }

    for (int r = 0; r < 16; ++r) {
        for (int c = 0; c < 16; ++c) {
            int dist = maze.manhattan_distances[r][c];
            if (dist != 255) {
                API::setText(c, r, to_string(dist));  // NOTE: col=x, row=y
            }
        }
    }
}

int Floodfill::getNextMove(int row, int col /*int direction //for optimize movement(for later improvement)*/) {
    int minDist = 255;
    int bestDirection = -1;

    // int nextRow = row;
    // int nextCol = col;

    if(!hasWall(row, col, 0) && maze.manhattan_distances[row + 1][col] < minDist) {
        minDist = maze.manhattan_distances[row + 1][col];
        bestDirection = 0;
    }
    if(!hasWall(row, col, 1) && maze.manhattan_distances[row][col + 1] < minDist) {
        minDist = maze.manhattan_distances[row][col + 1];
        bestDirection = 1;
    }
    if(!hasWall(row, col, 2) && maze.manhattan_distances[row - 1][col] < minDist) {
        minDist = maze.manhattan_distances[row - 1][col];
        bestDirection = 2;
    }
    if(!hasWall(row, col, 3) && maze.manhattan_distances[row][col - 1] < minDist) {
        minDist = maze.manhattan_distances[row][col - 1];
        bestDirection = 3;
    }

    // Serial.print("Let's move to the direction coded as ");
    // Serial.println(bestDirection);
    cout << "Let's move to the direction coded as " << bestDirection << endl;

    return bestDirection; // Return the best direction to move based on the minimum distance
}


Action rotateTo(int newDir) {
    int diff = (newDir - curDir + 4) % 4;
    if (diff == 0) return FORWARD;
    if (diff == 1) return RIGHT;
    if (diff == 3) return LEFT;
    if (diff == 2) return AROUND;  // U-turn
    return IDLE;
}


void moveForwardUpdatePos() {
    if (curDir == 0) curRow++;
    if (curDir == 1) curCol++;
    if (curDir == 2) curRow--;
    if (curDir == 3) curCol--;

}


Action solver() {
    floodfill.detectWalls({}, curRow, curCol, curDir);
    floodfill.floodfill();
    int bestDir = floodfill.getNextMove(curRow, curCol);

    Action action = rotateTo(bestDir);
    std::cerr << "Moved to: (" << curRow << ", " << curCol << ")" << std::endl;
    //API::setColor(curRow, curCol, 'g');
    std::cerr << "Action decided: " << action << std::endl;
    if (action == IDLE) {
        std::cerr << "No valid move found, staying idle." << std::endl;
    }
    return action;
}

void log(const string& text) {
    cerr << text << endl;
}
int main(int argc, char* argv[]) {
    log("Running...");
    while (true) {

        if (atGoal(curRow, curCol)) {
            log("Goal reached!");
            std::cerr << "\n==== MMS SIMULATOR STATS ====" << std::endl;
            std::cerr << "Total Distance               : " << getStat("total-distance") << std::endl;
            std::cerr << "Total Turns                  : " << getStat("total-turns") << std::endl;
            std::cerr << "Best Run Distance            : " << getStat("best-run-distance") << std::endl;
            std::cerr << "Best Run Turns               : " << getStat("best-run-turns") << std::endl;
            std::cerr << "Current Run Distance         : " << getStat("current-run-distance") << std::endl;
            std::cerr << "Current Run Turns            : " << getStat("current-run-turns") << std::endl;
            std::cerr << "Total Effective Distance     : " << getStat("total-effective-distance") << std::endl;
            std::cerr << "Best Run Effective Distance  : " << getStat("best-run-effective-distance") << std::endl;
            std::cerr << "Current Run Effective Dist   : " << getStat("current-run-effective-distance") << std::endl;
            std::cerr << "Final Score                  : " << 2000 - getStat("score") << std::endl;
            std::cerr << "Highest Possible score is 2000 " << std::endl;
            break;
        }

        Action action = solver();
        
        if (action == FORWARD) {
            API::moveForward();
            API::setColor(curCol, curRow, 'c');
            API::setText(curCol, curRow, to_string(floodfill.maze.manhattan_distances[curRow][curCol]));
            moveForwardUpdatePos();
        } else if (action == LEFT) {
            API::turnLeft();
            curDir = (curDir + 3) % 4;
        } else if (action == RIGHT) {
            API::turnRight();
            curDir = (curDir + 1) % 4;
        } else if (action == AROUND) {
            API::turnRight();
            API::turnRight();
            curDir = (curDir + 2) % 4;
        }
        
    }
    return 0;
}
