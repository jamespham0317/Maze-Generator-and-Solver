#pragma once
#include <vector>

struct Cell {
    bool visited = false;
    bool walls[4] = {true, true, true, true};
};

enum Direction { TOP = 0, RIGHT, BOTTOM, LEFT };

class Maze {
    public:
        Maze(int width, int height);
        void generate();
        void solve();
        void reset();
        void draw(int currentX = -1, int currentY = -1, bool solved = false);
    private:
        int width, height;
        std::vector<std::vector<Cell>> grid;
        std::vector<std::pair<int, int>> solutionPath;
        void removeWall(int x1, int y1, int x2, int y2);
        bool solve(int x, int y);
};