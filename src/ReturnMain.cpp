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

int idx[] = {0,1,3,2};
int curRow = 0, curCol = 0, curDir = 0;  // 0=north,1=east,2=south,3=west
bool reachedCenter = false;


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

    maze.visited[8][8] = true;
    maze.visited[8][7] = true;
    maze.visited[7][8] = true;
    maze.visited[7][7] = true;
    maze.visited[0][0] = true;

    
    q.push({8, 8}); q.push({8, 7}); q.push({7, 8}); q.push({7, 7});       

    array<array<bool, 16>, 16> reached = {};
    reached[8][8] = reached[8][7] = reached[7][8] = reached[7][7] = true; // Marking the goal cells as reached
    
    // BFS to fill the manhattan distances
    while(!q.empty()){
        pair<int,int> cell = q.front();
        q.pop();
        
        int row = cell.first;
        int col = cell.second;

        if (row < 0 || row > 15 || col < 0 || col > 15) continue;

        if (col > 0 && !maze.horizontal_walls[row][col].first && !reached[row][col - 1]) {
            maze.manhattan_distances[row][col - 1] = maze.manhattan_distances[row][col] + 1;
            q.push({row, col - 1});
            reached[row][col - 1] = true;
        }

        // NORTH
        if (row < 15 && !maze.vertical_walls[row][col].first && !reached[row + 1][col]) {
            maze.manhattan_distances[row + 1][col] = maze.manhattan_distances[row][col] + 1;
            q.push({row + 1, col});
            reached[row + 1][col] = true;
        }

        // EAST
        if (col < 15 && !maze.horizontal_walls[row][col].second && !reached[row][col + 1]) {
            maze.manhattan_distances[row][col + 1] = maze.manhattan_distances[row][col] + 1;
            q.push({row, col + 1});
            reached[row][col + 1] = true;
        }

        // SOUTH
        if (row > 0 && !maze.vertical_walls[row][col].second && !reached[row - 1][col]) {
            maze.manhattan_distances[row - 1][col] = maze.manhattan_distances[row][col] + 1;
            q.push({row - 1, col});
            reached[row - 1][col] = true;
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

void Floodfill::floodfillToStart() {
    queue<pair<int, int>> q;

    for (auto& row : maze.reverse_manhattan_distances)
        row.fill(255); // Reset distances

    // New goal is the start: bottom-left of the maze
    maze.reverse_manhattan_distances[0][0] = 0;
    q.push({0, 0});

    array<array<bool, 16>, 16> reached = {};
    reached[0][0] = true;

    while (!q.empty()) {
        pair<int, int> cell = q.front();
        q.pop();

        int row = cell.first;
        int col = cell.second;

        if (row < 0 || row > 15 || col < 0 || col > 15) continue;

        // WEST
        if (col > 0 && !maze.horizontal_walls[row][col].first && !reached[row][col - 1]) {
            maze.reverse_manhattan_distances[row][col - 1] = maze.reverse_manhattan_distances[row][col] + 1;
            q.push({row, col - 1});
            reached[row][col - 1] = true;
        }

        // NORTH
        if (row < 15 && !maze.vertical_walls[row][col].first && !reached[row + 1][col]) {
            maze.reverse_manhattan_distances[row + 1][col] = maze.reverse_manhattan_distances[row][col] + 1;
            q.push({row + 1, col});
            reached[row + 1][col] = true;
        }

        // EAST
        if (col < 15 && !maze.horizontal_walls[row][col].second && !reached[row][col + 1]) {
            maze.reverse_manhattan_distances[row][col + 1] = maze.reverse_manhattan_distances[row][col] + 1;
            q.push({row, col + 1});
            reached[row][col + 1] = true;
        }

        // SOUTH
        if (row > 0 && !maze.vertical_walls[row][col].second && !reached[row - 1][col]) {
            maze.reverse_manhattan_distances[row - 1][col] = maze.reverse_manhattan_distances[row][col] + 1;
            q.push({row - 1, col});
            reached[row - 1][col] = true;
        }
    }
}
void Floodfill::final_floodfill() {
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
        
        int row = cell.first;
        int col = cell.second;

        if (row < 0 || row > 15 || col < 0 || col > 15) continue;

        // WEST
        if (col > 0 && !maze.horizontal_walls[row][col].first && maze.visited[row][col - 1] && !reached[row][col - 1]) {
            maze.manhattan_distances[row][col - 1] = maze.manhattan_distances[row][col] + 1;
            q.push({row, col - 1});
            reached[row][col - 1] = true;
        }

        // NORTH
        if (row < 15 && !maze.vertical_walls[row][col].first && maze.visited[row + 1][col] && !reached[row + 1][col]) {
            maze.manhattan_distances[row + 1][col] = maze.manhattan_distances[row][col] + 1;
            q.push({row + 1, col});
            reached[row + 1][col] = true;
        }

        // EAST
        if (col < 15 && !maze.horizontal_walls[row][col].second && maze.visited[row][col + 1] && !reached[row][col + 1]) {
            maze.manhattan_distances[row][col + 1] = maze.manhattan_distances[row][col] + 1;
            q.push({row, col + 1});
            reached[row][col + 1] = true;
        }

        // SOUTH
        if (row > 0 && !maze.vertical_walls[row][col].second && maze.visited[row - 1][col] && !reached[row - 1][col]) {
            maze.manhattan_distances[row - 1][col] = maze.manhattan_distances[row][col] + 1;
            q.push({row - 1, col});
            reached[row - 1][col] = true;
        }

    }
    for (int r = 0; r < 16; ++r) {
        for (int c = 0; c < 16; ++c) {
            int dist = maze.manhattan_distances[r][c];
            if (dist != 255) {
                API::setText(c, r, to_string(dist));  
            }
        }
    }
}

int Floodfill::getNextMove(int row, int col) {
    int minDist = 255;
    int bestDirection = -1;

    for (int dir = 0; dir < 4; ++dir) {
        int r = row, c = col;

        if (dir == 0 && !hasWall(row, col, 0)) r++;
        else if (dir == 1 && !hasWall(row, col, 1)) c++;
        else if (dir == 2 && !hasWall(row, col, 2)) r--;
        else if (dir == 3 && !hasWall(row, col, 3)) c--;
        else continue;  // skip if there's a wall

        if (r < 0 || r >= 16 || c < 0 || c >= 16) continue;

        int dist = maze.manhattan_distances[r][c];

        if (dist < minDist || (dist == minDist && dir == curDir)) {
            minDist = dist;
            bestDirection = dir;
        }
    }

    return bestDirection;
}
int Floodfill::reverse_getNextMove(int row, int col) {
    int minDist = 255;
    int bestDirection = -1;

    for (int dir = 0; dir < 4; ++dir) {
        int r = row, c = col;

        if (dir == 0 && !hasWall(row, col, 0)) r++;
        else if (dir == 1 && !hasWall(row, col, 1)) c++;
        else if (dir == 2 && !hasWall(row, col, 2)) r--;
        else if (dir == 3 && !hasWall(row, col, 3)) c--;
        else continue;  // skip if there's a wall

        if (r < 0 || r >= 16 || c < 0 || c >= 16) continue;

        int dist = maze.reverse_manhattan_distances[r][c];

        if (dist < minDist || (dist == minDist && dir == curDir)) {
            minDist = dist;
            bestDirection = dir;
        }
    }

    return bestDirection;
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
    if (!reachedCenter && (
        (curRow == 8 && curCol == 8) ||
        (curRow == 8 && curCol == 7) ||
        (curRow == 7 && curCol == 8) ||
        (curRow == 7 && curCol == 7))) {
        reachedCenter = true;
        std::cerr << "Reached center! Recomputing best path to start...\n";
    }
    int bestDir=0;
    if(reachedCenter){
        floodfill.floodfillToStart();
        floodfill.floodfill();
        bestDir = floodfill.reverse_getNextMove(curRow, curCol);
    }
    else {
        floodfill.floodfill();
        bestDir = floodfill.getNextMove(curRow, curCol);
    }
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

        if (reachedCenter && curRow == 0 && curCol == 0) {
            std::cerr << "Returned to start!\n";
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

            API::turnRight();
            API::turnRight();
            curDir = (curDir + 2) % 4;

            // Highlight explored shortest path from start to center using known walls only
            floodfill.final_floodfill();
            int row = 0, col = 0;
            API::setColor(col, row, 'C');  // Highlight starting point

            while (!atGoal(row, col)) {
                int currentDist = floodfill.maze.manhattan_distances[row][col];
                int nextRow = row, nextCol = col;

                for (int dir = 0; dir < 4; ++dir) {
                    int r = row, c = col;

                    if (dir == 0 && !floodfill.hasWall(row, col, 0)) r++;
                    else if (dir == 1 && !floodfill.hasWall(row, col, 1)) c++;
                    else if (dir == 2 && !floodfill.hasWall(row, col, 2)) r--;
                    else if (dir == 3 && !floodfill.hasWall(row, col, 3)) c--;
                    else continue;

                    if (r >= 0 && r < 16 && c >= 0 && c < 16 && floodfill.maze.manhattan_distances[r][c] < currentDist) {
                        nextRow = r;
                        nextCol = c;
                        break;
                    }
                }

                row = nextRow;
                col = nextCol;
                API::setColor(col, row, 'C');  // Color path step
            }

            break;
        }
        if (!reachedCenter && atGoal(curRow, curCol)) {
            log("Reached center. Reversing goal to return home...");
            reachedCenter = true;
        }

        Action action = solver();
        
        if (action == FORWARD) {
            API::moveForward();            
            API::setColor(curCol, curRow, 'c');
            floodfill.maze.visited[curRow][curCol] = true;
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