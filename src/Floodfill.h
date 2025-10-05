#ifndef Floodfill_h
#define Floodfill_h
#include <queue>
#include <array>
#include <utility>
#include <vector>
using namespace std;

struct grid {
        array<array<pair<int, int>, 16>, 16> horizontal_walls; // (left,right)
        array<array<pair<int, int>, 16>, 16> vertical_walls; // (up,down)
        array<array<int, 16>, 16> manhattan_distances = {255}; // assigned false at the beginning
        array<array<int, 16>, 16> reverse_manhattan_distances = {255}; // assigned false at the beginning
        array<array<bool, 16>, 16> visited{};

    };

class Floodfill {
private:   
    vector<int> sensorDistances;
    int wall_threshhold;
    int direction;
    int row, col;
    bool front, left, right;
public:
    grid maze;
    void setThreshhold(int threshhold) {wall_threshhold = threshhold;}
    void setWall(int row, int col, int direction);
    void detectWalls(vector<int> sensorDistances, int row, int col, int direction);    
    void updateWall(int row, int col, bool front, bool left, bool right, int direction);
    void floodfill();
    void floodfillToStart();
    void final_floodfill();
    bool hasWall(int row, int col, int dir);
    int getNextMove(int row, int col); //next row column will also be automatically update from this function
    int reverse_getNextMove(int row, int col); //next row column will also be automatically update from this function

    void setGoal(int row, int col);
    bool isGoal(int row, int col);
    bool isCenter(int row, int col);
};
//both detectWall and updateWall do the same thing but with different parameters
#endif
