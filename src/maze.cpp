#include "maze.hpp"
#include <raylib.h>
#include <stack>
#include <random>
#include <algorithm>

Maze::Maze(int width, int height): width(width), height(height)
{
    grid.resize(height, std::vector<Cell>(width));
}

void Maze::generate()
{
    std::stack<std::pair<int, int>> stack;
    stack.push({0, 0});
    grid[0][0].visited = true;

    std::random_device rd;  
    std::mt19937 gen(rd());
    
    while (!stack.empty()) {
        auto [x, y] = stack.top();

        BeginDrawing();
        ClearBackground(RAYWHITE);
        draw(x, y);
        EndDrawing();
        WaitTime(0.01);
        
        std::vector<std::pair<int, int>> neighbors;

        if (y - 1 >= 0 && !grid[y-1][x].visited) neighbors.push_back({x, y - 1}); 
        if (x + 1 < width && !grid[y][x + 1].visited) neighbors.push_back({x + 1, y});
        if (y + 1 < height && !grid[y + 1][x].visited) neighbors.push_back({x, y + 1});
        if (x - 1 >= 0 && !grid[y][x - 1].visited) neighbors.push_back({x - 1, y});

        if (!neighbors.empty()) {
            std::uniform_int_distribution<> dist(0, neighbors.size() - 1);
            int randomNumber = dist(gen);

            auto [nx, ny] = neighbors[randomNumber];
            removeWall(x, y, nx, ny);
            grid[ny][nx].visited = true;
            stack.push({nx, ny});
        } else {
            stack.pop();
        }
    } 
}

void Maze::solve()
{
    solve(0, 0);
}

void Maze::reset()
{
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            grid[y][x].visited = false;
        }
    }
}

void Maze::draw(int currentX, int currentY, bool solved)
{
    int cellSize = 20;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int px = x * cellSize;
            int py = y * cellSize;

            if (solved) {
                if (std::find(solutionPath.begin(), solutionPath.end(), std::pair{x, y}) != solutionPath.end()) {
                    DrawRectangle(px, py, cellSize, cellSize, BLUE);
                }
            } else {
                if (grid[y][x].visited) {
                    DrawRectangle(px, py, cellSize, cellSize, LIGHTGRAY);
                }
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

bool Maze::solve(int x, int y)
{
    if (x == width - 1 && y == height - 1) {
        solutionPath.emplace_back(x, y);
        return true;
    }

    grid[y][x].visited = true;

    BeginDrawing();
    ClearBackground(RAYWHITE);
    draw(x, y);
    EndDrawing();
    WaitTime(0.01);

    if (y - 1 >= 0 && !grid[y][x].walls[0] && !grid[y-1][x].visited) {
        if (solve(x, y - 1)) {
            solutionPath.emplace_back(x, y);
            return true;
        }  
    }
    if (x + 1 < width && !grid[y][x].walls[1] && !grid[y][x + 1].visited) {
        if (solve(x + 1, y)) {
            solutionPath.emplace_back(x, y);
            return true;
        } 
    }
    if (y + 1 < height && !grid[y][x].walls[2] && !grid[y+1][x].visited) {
        if (solve(x, y + 1)) {
            solutionPath.emplace_back(x, y);
            return true;
        }
    }
    if (x - 1 >= 0 && !grid[y][x].walls[3] && !grid[y][x - 1].visited) {
        if (solve(x - 1, y)) {
            solutionPath.emplace_back(x, y);
            return true;
        }
    }
    return false;
}

