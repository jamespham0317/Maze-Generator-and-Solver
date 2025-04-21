#include "maze.hpp"
#include <raylib.h>
#include <stack>
#include <queue>
#include <map>
#include <random>
#include <algorithm>

void Maze::generateDFS()
{
    std::stack<std::pair<int, int>> stack;
    stack.emplace(0, 0);
    grid[0][0].visited = true;
    
    while (!stack.empty()) {
        auto [x, y] = stack.top();

        animate(x, y);
        
        std::vector<std::pair<int, int>> neighbors;
        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (isValid(nx, ny) && !grid[ny][nx].visited) {
                neighbors.emplace_back(nx, ny);
            }
        }

        if (!neighbors.empty()) {
            int idx = rng() % neighbors.size();
            auto [nx, ny] = neighbors[idx];
            removeWall(x, y, nx, ny);
            grid[ny][nx].visited = true;
            stack.emplace(nx, ny);
        } else {
            stack.pop();
        }
    } 
}

void Maze::generatePrims() 
{
    std::vector<std::tuple<int, int, int, int>> edges;

    int x = 0;
    int y = 0;
    grid[0][0].visited = true;

    for (int i = 0; i < 4; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (isValid(nx, ny)) {
            edges.emplace_back(x, y, nx, ny);
        }
    }

    while (!edges.empty()) {
        int idx = rng() % edges.size();
        auto [x1, y1, x2, y2] = edges[idx];
        edges.erase(edges.begin() + idx);

        if (isValid(x2, y2) && !grid[y2][x2].visited) {
            removeWall(x1, y1, x2, y2);
            grid[y2][x2].visited = true;

            for (int i = 0; i < 4; ++i) {
                int nx = x2 + dx[i];
                int ny = y2 + dy[i];
                if (isValid(nx, ny) && !grid[ny][nx].visited) {
                    edges.emplace_back(x2, y2, nx, ny);
                }
            }
            animate(x2, y2);
        }
    }
}

void Maze::generateKruskals() 
{
    std::vector<std::tuple<int, int, int, int>> edges;

    std::vector<int> cellSet(width * height);
    for (int i = 0; i < width * height; ++i) {
        cellSet[i] = i;
    }

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (isValid(x + 1, y)) edges.emplace_back(x, y, x + 1, y); 
            if (isValid(x, y + 1)) edges.emplace_back(x, y, x, y + 1); 
        }
    }

    std::shuffle(edges.begin(), edges.end(), rng);

    for (auto [x1, y1, x2, y2] : edges) {
        int idx1 = y1 * width + x1;
        int idx2 = y2 * width + x2;

        if (cellSet[idx1] != cellSet[idx2]) {
            removeWall(x1, y1, x2, y2);

            int oldSet = cellSet[idx2];
            int newSet = cellSet[idx1];

            for (int i = 0; i < width * height; ++i) {
                if (cellSet[i] == oldSet) cellSet[i] = newSet;
            }

            grid[y1][x1].visited = true;
            grid[y2][x2].visited = true;
            animate(x2, y2);
        }
    }
}

void Maze::removeWall(int x1, int y1, int x2, int y2)
{
    int dx = x2 - x1;
    int dy = y2 - y1;

    if (dx == 1) { 
        grid[y1][x1].walls[RIGHT] = false;
        grid[y2][x2].walls[LEFT] = false;
    } else if (dx == -1) {
        grid[y1][x1].walls[LEFT] = false;
        grid[y2][x2].walls[RIGHT] = false;
    } else if (dy == 1) {
        grid[y1][x1].walls[BOTTOM] = false;
        grid[y2][x2].walls[TOP] = false;
    } else if (dy == -1) {
        grid[y1][x1].walls[TOP] = false;
        grid[y2][x2].walls[BOTTOM] = false;
    }
}
