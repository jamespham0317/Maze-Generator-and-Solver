#include "maze.hpp"
#include <raylib.h>
#include <stack>
#include <queue>
#include <map>
#include <random>
#include <algorithm>
#include <iostream>

Maze::Maze(int width, int height): width(width), height(height)
{
    grid.resize(height, std::vector<Cell>(width));
}

void Maze::generate()
{
    std::stack<std::pair<int, int>> stack;
    stack.emplace(0, 0);
    grid[0][0].visited = true;

    std::random_device rd;  
    std::mt19937 gen(rd());
    
    while (!stack.empty()) {
        auto [x, y] = stack.top();

        BeginDrawing();
        ClearBackground(LIGHTGRAY);
        drawGenerate(x, y);
        EndDrawing();
        WaitTime(0.01);
        
        std::vector<std::pair<int, int>> neighbors;
        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (isValid(nx, ny)) {
                neighbors.emplace_back(nx, ny);
            }
        }

        if (!neighbors.empty()) {
            std::uniform_int_distribution<> dist(0, neighbors.size() - 1);
            int randomNumber = dist(gen);

            auto [nx, ny] = neighbors[randomNumber];
            removeWall(x, y, nx, ny);
            grid[ny][nx].visited = true;
            stack.emplace(nx, ny);
        } else {
            stack.pop();
        }
    } 
}

bool Maze::solveDFS(int x, int y)
{
    grid[y][x].visited = true;

    BeginDrawing();
    ClearBackground(RAYWHITE);
    drawSolve(x, y);
    EndDrawing();
    WaitTime(0.01);

    if (x == width - 1 && y == height - 1) {
        solutionPath.emplace_back(x, y);
        return true;
    }

    for (int i = 0; i < 4; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];

        if (isValid(nx, ny) && !grid[y][x].walls[i]) {
            if (solveDFS(nx, ny)) {
                solutionPath.emplace_back(x, y);
                return true;
            }
        }
    }
    return false;
}

bool Maze::solveBFS()
{
    std::queue<std::pair<int, int>> q;
    std::map<std::pair<int, int>, std::pair<int, int>> parent;

    q.emplace(0, 0);
    grid[0][0].visited = true;

    while (!q.empty()) {
        auto [x, y] = q.front();

        BeginDrawing();
        ClearBackground(RAYWHITE);
        drawSolve(x, y); 
        EndDrawing();
        WaitTime(0.01);
        
        q.pop();

        if (x == width - 1 && y == height - 1) {
            std::pair<int, int> cur = {x, y};
            while (cur != std::make_pair(0, 0)) {
                solutionPath.push_back(cur);
                cur = parent[cur];
            }
            solutionPath.emplace_back(0, 0);
            std::reverse(solutionPath.begin(), solutionPath.end());
            return true;
        }

        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (isValid(nx, ny) && !grid[y][x].walls[i]) {
                q.emplace(nx, ny);
                grid[ny][nx].visited = true;
                parent[{nx, ny}] = {x, y};
            }
        }
    }

    return false;
}

void Maze::reset()
{
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            grid[y][x].visited = false;
        }
    }
    solutionPath.clear();
}

void Maze::drawGenerate(int currentX, int currentY)
{
    int cellSize = 20;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int px = x * cellSize;
            int py = y * cellSize;

            if (grid[y][x].visited) {
                DrawRectangle(px, py, cellSize, cellSize, RAYWHITE);
            }

            if (x == currentX && y == currentY) {
                DrawRectangle(px, py, cellSize, cellSize, BLUE);
            }

            if (grid[y][x].walls[0]) DrawRectangle(px, py, cellSize, 2, BLACK);
            if (grid[y][x].walls[1]) DrawRectangle(px + cellSize - 1, py, 2, cellSize, BLACK);
            if (grid[y][x].walls[2]) DrawRectangle(px, py + cellSize - 1, cellSize, 2, BLACK);
            if (grid[y][x].walls[3]) DrawRectangle(px, py, 2, cellSize, BLACK);
        }
    }
}

void Maze::drawSolve(int currentX, int currentY)
{
    int cellSize = 20;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int px = x * cellSize;
            int py = y * cellSize;

            if (solutionPath.empty()) {
                if (grid[y][x].visited) {
                    DrawRectangle(px, py, cellSize, cellSize, LIGHTGRAY);
                }
            } else if (std::find(solutionPath.begin(), solutionPath.end(), std::pair{x, y}) != solutionPath.end()) {
                DrawRectangle(px, py, cellSize, cellSize, BLUE);
            }

            if (x == currentX && y == currentY) {
                DrawRectangle(px, py, cellSize, cellSize, BLUE);
            }
            if (x == 0 && y == 0) {
                DrawRectangle(px, py, cellSize, cellSize, GREEN);
            }
            if (x == width - 1 && y == height - 1) {
                DrawRectangle(px, py, cellSize, cellSize, RED);
            }

            if (grid[y][x].walls[0]) DrawRectangle(px, py, cellSize, 2, BLACK);
            if (grid[y][x].walls[1]) DrawRectangle(px + cellSize - 1, py, 2, cellSize, BLACK);
            if (grid[y][x].walls[2]) DrawRectangle(px, py + cellSize - 1, cellSize, 2, BLACK);
            if (grid[y][x].walls[3]) DrawRectangle(px, py, 2, cellSize, BLACK);
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

bool Maze::isValid(int x, int y)
{
    return (x >= 0 && y >= 0 && x < width && y < height && !grid[y][x].visited);
}
