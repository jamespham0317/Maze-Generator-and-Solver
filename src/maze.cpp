#include "maze.hpp"
#include <raylib.h>
#include <stack>
#include <queue>
#include <map>
#include <random>
#include <algorithm>
#include <iostream>

Maze::Maze(int width, int height, int cellSize): width(width), height(height), cellSize(cellSize)
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
        WaitTime(0.0001);
        
        std::vector<std::pair<int, int>> neighbors;
        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (isValid(nx, ny) && !grid[ny][nx].visited) {
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

    if (isGoal(x, y)) {
        solutionPath.emplace_back(x, y);
        return true;
    }

    for (int i = 0; i < 4; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];

        if (isValid(nx, ny) && !grid[ny][nx].visited && !grid[y][x].walls[i]) {
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
    std::queue<std::pair<int, int>> queue;
    std::map<std::pair<int, int>, std::pair<int, int>> parent;

    queue.emplace(0, 0);
    grid[0][0].visited = true;

    while (!queue.empty()) {
        auto [x, y] = queue.front();

        BeginDrawing();
        ClearBackground(RAYWHITE);
        drawSolve(x, y); 
        EndDrawing();
        WaitTime(0.01);
        
        queue.pop();

        if (isGoal(x, y)) {
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

            if (isValid(nx, ny) && !grid[ny][nx].visited && !grid[y][x].walls[i]) {
                queue.emplace(nx, ny);
                grid[ny][nx].visited = true;
                parent[{nx, ny}] = {x, y};
            }
        }
    }

    return false;
}

bool Maze::solveAStar()
{
    std::map<std::pair<int, int>, float> fScore;
    std::vector<std::vector<float>> gScore(height, std::vector<float>(width, INFINITY));
    std::map<std::pair<int, int>, std::pair<int, int>> parent;

    gScore[0][0] = 0;
    fScore[{0, 0}] = heuristic(0, 0);

    auto cmp = [&](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return fScore[a] > fScore[b];
    };

    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, decltype(cmp)> openSet(cmp);

    openSet.emplace(0, 0);

    while (!openSet.empty()) {
        auto [x, y] = openSet.top();
        openSet.pop();

        if (grid[y][x].visited) continue;
        grid[y][x].visited = true;

        BeginDrawing();
        ClearBackground(RAYWHITE);
        drawSolve(x, y);
        EndDrawing();
        WaitTime(0.01);

        if (isGoal(x, y)) {
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

            if (isValid(nx, ny) && !grid[ny][nx].visited && !grid[y][x].walls[i]) {
                float tentativeG = gScore[y][x] + 1;

                if (tentativeG < gScore[ny][nx]) {
                    gScore[ny][nx] = tentativeG;
                    parent[{nx, ny}] = {x, y};
                    fScore[{nx, ny}] = tentativeG + heuristic(nx, ny);
                    openSet.emplace(nx, ny);
                }
            }
        }
    }
    return false;
}

bool Maze::solveGreedyBFS()
{
    std::map<std::pair<int, int>, std::pair<int, int>> parent;

    auto cmp = [&](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return heuristic(a.first, a.second) > heuristic(b.first, b.second);
    };

    std::priority_queue< std::pair<int, int>, std::vector<std::pair<int, int>>, decltype(cmp)> openSet(cmp);

    openSet.emplace(0, 0);
    grid[0][0].visited = true;

    while (!openSet.empty()) {
        auto [x, y] = openSet.top();

        BeginDrawing();
        ClearBackground(RAYWHITE);
        drawSolve(x, y);
        EndDrawing();
        WaitTime(0.01);

        openSet.pop();

        if (isGoal(x, y)) {
            std::pair<int, int> cur = {x, y};
            while (cur != std::make_pair(0, 0)) {
                solutionPath.push_back(cur);
                cur = parent[cur];
            }
            solutionPath.push_back({0, 0});
            std::reverse(solutionPath.begin(), solutionPath.end());
            return true;
        }

        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (isValid(nx, ny) && !grid[y][x].walls[i] && !grid[ny][nx].visited) {
                grid[ny][nx].visited = true;
                parent[{nx, ny}] = {x, y};
                openSet.emplace(nx, ny);
            }
        }
    }

    return false;
}

bool Maze::solveWallFollower(int x, int y, int dir, bool leftHanded)
{ 
    if (isGoal(x, y)) {
        solutionPath.emplace_back(x, y);
        return true;
    }

    grid[y][x].visited = true;

    BeginDrawing();
    ClearBackground(RAYWHITE);
    drawSolve(x, y);
    EndDrawing();
    WaitTime(0.01);

    int preferred[4];

    if (leftHanded) {
        preferred[0] = turnLeft(dir);
        preferred[1] = dir;
        preferred[2] = turnRight(dir);
        preferred[3] = turnBack(dir);
    } else {
        preferred[0] = turnRight(dir);
        preferred[1] = dir;
        preferred[2] = turnLeft(dir);
        preferred[3] = turnBack(dir);
    }

    for (int i = 0; i < 4; i++) {
        int ndir = preferred[i];
        int nx = x + dx[ndir];
        int ny = y + dy[ndir];

        if (isValid(nx, ny) && !grid[y][x].walls[ndir]) {
            if (solveWallFollower(nx, ny, ndir, leftHanded)) {
                solutionPath.emplace_back(x, y);
                return true;
            }
        }
    }
    return false;
}

bool Maze::solveDeadEndFiller()
{
    std::vector<std::vector<bool>> isSolution(height, std::vector<bool>(width, true));
    bool changed = true;

    while (changed) {
        changed = false;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (isSolution[y][x] && isDeadEnd(x, y, isSolution)) {
                    isSolution[y][x] = false;
                    grid[y][x].visited = true;
                    changed = true;

                    BeginDrawing();
                    ClearBackground(RAYWHITE);
                    drawSolve(x, y);
                    EndDrawing();
                    WaitTime(0.01);
                }
            }
        }
    }

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (isSolution[y][x]) {
                solutionPath.emplace_back(x, y);
            }
        }
    }

    return true;
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
            if (isStart(x, y)) {
                DrawRectangle(px, py, cellSize, cellSize, GREEN);
            }
            if (isGoal(x, y)) {
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
    return (x >= 0 && y >= 0 && x < width && y < height);
}

bool Maze::isStart(int x, int y)
{
    return (x == 0 && y == 0);
}

bool Maze::isGoal(int x, int y)
{
    return (x == width - 1 && y == height - 1);
}

float Maze::heuristic(int x, int y)
{
    return abs(x - (width - 1)) + abs(y - (height - 1));
}

int Maze::turnLeft(int dir) { return (dir + 3) % 4; }
int Maze::turnRight(int dir) { return (dir + 1) % 4; }
int Maze::turnBack(int dir) { return (dir + 2) % 4; }

bool Maze::isDeadEnd(int x, int y, const std::vector<std::vector<bool>>& isSolution)
{
    if (isStart(x, y) || isGoal(x, y)) return false; 

    int exits = 0;
    for (int i = 0; i < 4; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];

        if (isValid(nx, ny) && !grid[y][x].walls[i] && isSolution[ny][nx]) {
            exits++;
        }
    }
    return exits <= 1;
}
