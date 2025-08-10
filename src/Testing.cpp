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

enum Action { FORWARD, LEFT, RIGHT, IDLE, AROUND };

int curRow = 0, curCol = 0, curDir = 0;  // 0=north,1=east,2=south,3=west
bool reachedCenter = false;

static inline bool inB(int r,int c){ return r>=0&&r<16&&c>=0&&c<16; }
static inline bool atGoal(int r,int c){ return (r==7||r==8)&&(c==7||c==8); }
static const int dr[4]={+1,0,-1,0};
static const int dc[4]={0,+1,0,-1};

float getStat(const std::string& statName) {
    std::cout << "getStat " << statName << std::endl;
    std::string response; std::cin >> response;
    try { return std::stof(response); } catch (...) { std::cerr<<"Failed to parse stat: "<<statName<<std::endl; return -1; }
}

void Floodfill::setWall(int row, int col, int direction) {
    if(direction==0){ maze.vertical_walls[row][col].first=1;  if(row+1<16) maze.vertical_walls[row+1][col].second=1; }
    if(direction==1){ maze.horizontal_walls[row][col].second=1; if(col+1<16) maze.horizontal_walls[row][col+1].first=1; }
    if(direction==2){ maze.vertical_walls[row][col].second=1; if(row-1>=0)  maze.vertical_walls[row-1][col].first=1; }
    if(direction==3){ maze.horizontal_walls[row][col].first=1; if(col-1>=0)  maze.horizontal_walls[row][col-1].second=1; }
    API::setWall(col, row, "nesw"[direction]);
}

void Floodfill::detectWalls(vector<int>, int row, int col, int direction){
    if(!inB(row,col)) return;
    if(API::wallFront()) setWall(row,col,direction);
    if(API::wallLeft())  setWall(row,col,(direction+3)%4);
    if(API::wallRight()) setWall(row,col,(direction+1)%4);
}

bool Floodfill::hasWall(int row,int col,int dir){
    if(dir==0) return maze.vertical_walls[row][col].first;
    if(dir==1) return maze.horizontal_walls[row][col].second;
    if(dir==2) return maze.vertical_walls[row][col].second;
    if(dir==3) return maze.horizontal_walls[row][col].first;
    return false;
}

static void fillTextFrom(const array<array<int,16>,16>& dist){
    for(int r=0;r<16;++r) for(int c=0;c<16;++c){
        int d=dist[r][c]; if(d!=255) API::setText(c,r,to_string(d));
    }
}

void Floodfill::floodfill() {
    queue<pair<int,int>> q; for(auto& r:maze.manhattan_distances) r.fill(255);
    maze.manhattan_distances[8][8]=0;
    maze.manhattan_distances[8][7]=0;
    maze.manhattan_distances[7][8]=0;
    maze.manhattan_distances[7][7]=0;
    q.push({8,8}); 
    q.push({8,7}); 
    q.push({7,8}); 
    q.push({7,7});
    maze.visited[8][8]=true;
    maze.visited[8][7]=true;
    maze.visited[7][8]=true;
    maze.visited[7][7]=true; 
    maze.visited[0][0]=true;
    array<array<bool,16>,16> seen={}; 
    seen[8][8]=true;
    seen[8][7]=true;
    seen[7][8]=true;
    seen[7][7]=true;

    while(!q.empty()){
    std::pair<int,int> p = q.front(); q.pop();
    int r = p.first, c = p.second;

        for(int d=0; d<4; ++d){
            if(hasWall(r,c,d)) continue;
            int nr = r + dr[d], nc = c + dc[d];
            if(!inB(nr,nc) || seen[nr][nc]) continue;

            maze.manhattan_distances[nr][nc] = maze.manhattan_distances[r][c] + 1;
            q.push(std::make_pair(nr, nc));
            seen[nr][nc] = true;
        }
    }

    fillTextFrom(maze.manhattan_distances);
}

void Floodfill::floodfillToStart() {
    queue<pair<int,int>> q; for(auto& r:maze.reverse_manhattan_distances) r.fill(255);
    maze.reverse_manhattan_distances[0][0]=0; 
    q.push({0,0});
    array<array<bool,16>,16> seen={}; 
    seen[0][0]=true;
    while(!q.empty()){
        std::pair<int,int> p = q.front(); q.pop();
        int r = p.first, c = p.second;

        for(int d=0; d<4; ++d){
            if(hasWall(r,c,d)) continue;
            int nr = r + dr[d], nc = c + dc[d];
            if(!inB(nr,nc) || seen[nr][nc]) continue;

            maze.reverse_manhattan_distances[nr][nc] = maze.reverse_manhattan_distances[r][c] + 1;
            q.push(std::make_pair(nr, nc));
            seen[nr][nc] = true;
        }
    }

}

void Floodfill::final_floodfill() {
    queue<pair<int,int>> q; for(auto& r:maze.manhattan_distances) r.fill(255);
    maze.manhattan_distances[8][8]=0;
    maze.manhattan_distances[8][7]=0;
    maze.manhattan_distances[7][8]=0;
    maze.manhattan_distances[7][7]=0;
    q.push({8,8}); 
    q.push({8,7}); 
    q.push({7,8}); 
    q.push({7,7});
    array<array<bool,16>,16> seen={}; seen[8][8]=seen[8][7]=seen[7][8]=seen[7][7]=true;

    while(!q.empty()){
        std::pair<int,int> p = q.front(); q.pop();
        int r = p.first, c = p.second;

        for(int d=0; d<4; ++d){
            if(hasWall(r,c,d)) continue;
            int nr = r + dr[d], nc = c + dc[d];
            if(!inB(nr,nc) || seen[nr][nc]) continue;
            if(!maze.visited[nr][nc]) continue;

            maze.manhattan_distances[nr][nc] = maze.manhattan_distances[r][c] + 1;
            q.push(std::make_pair(nr, nc));
            seen[nr][nc] = true;
        }
    }

    fillTextFrom(maze.manhattan_distances);
}

int Floodfill::getNextMove(int row, int col) {
    int best=-1,md=255;
    for(int d=0;d<4;++d){
        if(hasWall(row,col,d)) continue;
        int r=row+dr[d], c=col+dc[d]; if(!inB(r,c)) continue;
        int dist=maze.manhattan_distances[r][c];
        if(dist<md || (dist==md && d==curDir)){ md=dist; best=d; }
    }
    return best;
}

int Floodfill::reverse_getNextMove(int row, int col) {
    int best=-1,md=255;
    for(int d=0;d<4;++d){
        if(hasWall(row,col,d)) continue;
        int r=row+dr[d], c=col+dc[d]; if(!inB(r,c)) continue;
        int dist=maze.reverse_manhattan_distances[r][c];
        if(dist<md || (dist==md && d==curDir)){ md=dist; best=d; }
    }
    return best;
}

Action rotateTo(int newDir){
    int diff=(newDir-curDir+4)%4;
    if(diff==0) return FORWARD;
    if(diff==1) return RIGHT;
    if(diff==3) return LEFT;
    if(diff==2) return AROUND;
    return IDLE;
}

void moveForwardUpdatePos(){
    if(curDir==0) ++curRow;
    else if(curDir==1) ++curCol;
    else if(curDir==2) --curRow;
    else if(curDir==3) --curCol;
}

Action solver(){
    floodfill.detectWalls({},curRow,curCol,curDir);
    if(!reachedCenter && atGoal(curRow,curCol)){
        reachedCenter=true;
        std::cerr<<"Reached center! Recomputing best path to start...\n";
    }
    int bestDir=0;
    if(reachedCenter){
        floodfill.floodfillToStart();
        floodfill.floodfill();                 // keep your behavior
        bestDir=floodfill.reverse_getNextMove(curRow,curCol);
    }else{
        floodfill.floodfill();
        bestDir=floodfill.getNextMove(curRow,curCol);
    }
    Action a=rotateTo(bestDir);
    std::cerr<<"Moved to: ("<<curRow<<", "<<curCol<<")\n";
    std::cerr<<"Action decided: "<<a<<"\n";
    if(a==IDLE) std::cerr<<"No valid move found, staying idle.\n";
    return a;
}

void log(const string& text){ cerr<<text<<endl; }

int main(int argc,char* argv[]){
    log("Running...");
    while(true){
        if(reachedCenter && curRow==0 && curCol==0){
            std::cerr<<"Returned to start!\n"; log("Goal reached!");
            std::cerr<<"\n==== MMS SIMULATOR STATS ====\n";
            std::cerr<<"Total Distance               : "<<getStat("total-distance")<<'\n';
            std::cerr<<"Total Turns                  : "<<getStat("total-turns")<<'\n';
            std::cerr<<"Best Run Distance            : "<<getStat("best-run-distance")<<'\n';
            std::cerr<<"Best Run Turns               : "<<getStat("best-run-turns")<<'\n';
            std::cerr<<"Current Run Distance         : "<<getStat("current-run-distance")<<'\n';
            std::cerr<<"Current Run Turns            : "<<getStat("current-run-turns")<<'\n';
            std::cerr<<"Total Effective Distance     : "<<getStat("total-effective-distance")<<'\n';
            std::cerr<<"Best Run Effective Distance  : "<<getStat("best-run-effective-distance")<<'\n';
            std::cerr<<"Current Run Effective Dist   : "<<getStat("current-run-effective-distance")<<'\n';
            std::cerr<<"Final Score                  : "<<2000 - getStat("score")<<'\n';
            std::cerr<<"Highest Possible score is 2000 \n";

            API::turnRight(); API::turnRight(); curDir=(curDir+2)%4;

            // Highlight explored shortest path from start to center (visited-only distances)
            floodfill.final_floodfill();
            int r=0,c=0; API::setColor(c,r,'C');
            while(!atGoal(r,c)){
                int cur=floodfill.maze.manhattan_distances[r][c];
                int nr=r, nc=c; bool moved=false;
                for(int d=0;d<4;++d){
                    if(floodfill.hasWall(r,c,d)) continue;
                    int rr=r+dr[d], cc=c+dc[d]; if(!inB(rr,cc)) continue;
                    if(floodfill.maze.manhattan_distances[rr][cc] < cur){ nr=rr; nc=cc; moved=true; break; }
                }
                if(!moved) break;
                r=nr; c=nc; API::setColor(c,r,'C');
            }
            break;
        }

        Action a=solver();
        if(a==FORWARD){
            API::moveForward();
            API::setColor(curCol,curRow,'c');                 // keep order
            floodfill.maze.visited[curRow][curCol]=true;       // keep order
            API::setText(curCol,curRow,to_string(floodfill.maze.manhattan_distances[curRow][curCol]));
            moveForwardUpdatePos();                            // keep order
        }else if(a==LEFT){
            API::turnLeft();  curDir=(curDir+3)%4;
        }else if(a==RIGHT){
            API::turnRight(); curDir=(curDir+1)%4;
        }else if(a==AROUND){
            API::turnRight(); API::turnRight(); curDir=(curDir+2)%4;
        }
    }
    return 0;
}
