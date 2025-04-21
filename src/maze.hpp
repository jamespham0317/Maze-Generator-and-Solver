#pragma once
#include <vector>
#include <random>
#include <string>
#include <chrono>

struct Cell {
    bool visited = false;
    bool walls[4] = { true, true, true, true };
};

enum Direction { TOP, RIGHT, BOTTOM, LEFT };
enum Generator { DFS, PRIMS, KRUSKALS };
enum Solver { BFS = 1, ASTAR, GREEDY, WALL_LEFT, WALL_RIGHT, DEAD_END };
enum Stage { INITIALIZED, GENERATING, GENERATED, SOLVING, SOLVED };

const int dx[4] = {0, 1, 0, -1}; 
const int dy[4] = {-1, 0, 1, 0};

class Maze {
    public:
        Maze(int width, int height, int cellSize);
        int stage = 0;
        int generator;
        int solver;

        void generate();
        void solve();

        void generateDFS();
        void generatePrims();
        void generateKruskals();

        bool solveDFS(int x = 0, int y = 0);
        bool solveBFS();
        bool solveAStar();
        bool solveGreedyBFS();
        bool solveWallFollower(int x, int y, int dir, bool leftHanded);
        bool solveDeadEndFiller();

        void reset();
        void resetMaze();
        void resetVisited();

        void animate(int currentX = -1, int currentY = -1);
        void draw(int currentX, int currentY);
    private:
        int width, height;
        int cellSize;

        std::vector<std::vector<Cell>> grid;
        std::vector<std::pair<int, int>> solutionPath;

        std::mt19937 rng;

        std::chrono::high_resolution_clock::time_point timerStart;
        double generatorRuntime = 0;
        double solverRuntime = 0;

        void removeWall(int x1, int y1, int x2, int y2);
        bool isValid(int x, int y);
        bool isStart(int x, int y);
        bool isGoal(int x, int y);
        bool isInSolutionPath(int x, int y);

        float heuristic(int x, int y);
        int turnLeft(int dir);
        int turnRight(int dir);
        int turnBack(int dir);
        bool isDeadEnd(int x, int y, const std::vector<std::vector<bool>>& isSolution);

        int nodesVisited();
        std::string generatorString();
        std::string solverString();
};