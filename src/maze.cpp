#include "maze.hpp"
#include <raylib.h>
#include <stack>
#include <queue>
#include <map>
#include <random>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <chrono>

Maze::Maze(int width, int height, int cellSize): width(width), height(height), cellSize(cellSize)
{
    grid.resize(height, std::vector<Cell>(width));
    std::mt19937 rng(std::random_device{}());
}

void Maze::generate() 
{
    reset();
    stage = GENERATING;
    timerStart = std::chrono::high_resolution_clock::now();
    switch (generator) {
    case DFS:
        generateDFS();
        break;
    case PRIMS:
        generatePrims();
        break;
    case KRUSKALS:
        generateKruskals();
        break;
    default:
        break;
    }
    auto timerEnd = std::chrono::high_resolution_clock::now();
    stage = GENERATED;
    resetVisited();

    std::chrono::duration<double> elapsed = timerEnd - timerStart;
    generatorRuntime = elapsed.count();
}

void Maze:: solve() 
{
    resetMaze();
    stage = SOLVING;
    timerStart = std::chrono::high_resolution_clock::now();
    switch (solver) {
    case DFS:
        solveDFS();
        break;
    case BFS:
        solveBFS();
        break;
    case ASTAR:
        solveAStar();
        break;
    case GREEDY:
        solveGreedyBFS();
        break;
    case WALL_LEFT:
        solveWallFollower(0, 0, RIGHT, true);
        break;
    case WALL_RIGHT:
        solveWallFollower(0, 0, RIGHT, false);
        break;
    case DEAD_END:
        solveDeadEndFiller();
        break;
    default:
        break;
    }
    auto timerEnd = std::chrono::high_resolution_clock::now();
    stage = SOLVED;

    std::chrono::duration<double> elapsed = timerEnd - timerStart;
    solverRuntime = elapsed.count();
}

void Maze::reset() 
{
    grid.clear();
    grid.resize(height, std::vector<Cell>(width));
    solutionPath.clear();
    stage = 0;
}

void Maze::resetMaze()
{
    resetVisited();
    solutionPath.clear();
}

void Maze::resetVisited() 
{
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            grid[y][x].visited = false;
        }
    }
}

void Maze::animate(int currentX, int currentY) 
{
    BeginDrawing();
    ClearBackground(DARKGRAY);
    draw(currentX, currentY);
    EndDrawing();
    WaitTime(0.01);
}

void Maze::draw(int currentX, int currentY)
{
    int offset = 6;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int px = (x + offset) * cellSize;
            int py = (y + offset) * cellSize;

            Color tileColor;
            if (stage < 2) {
                tileColor = LIGHTGRAY;
                if (grid[y][x].visited) tileColor = GRAY;
            } else {
                tileColor = GRAY;
                if (isStart(x, y)) tileColor = GREEN;
                else if (isGoal(x, y)) tileColor = {255, 46, 67, 255};
                else if (isInSolutionPath(x, y)) tileColor = {84, 182, 247, 255};
                else if (grid[y][x].visited) tileColor = LIGHTGRAY;
            }
            if (x == currentX && y == currentY) tileColor = BLUE;

            DrawRectangle(px, py, cellSize, cellSize, tileColor);

            if (grid[y][x].walls[0]) DrawRectangle(px, py, cellSize, 1, RAYWHITE);
            if (grid[y][x].walls[1]) DrawRectangle(px + cellSize, py, 1, cellSize, RAYWHITE);
            if (grid[y][x].walls[2]) DrawRectangle(px, py + cellSize, cellSize, 1, RAYWHITE);
            if (grid[y][x].walls[3]) DrawRectangle(px, py, 1, cellSize, RAYWHITE);
        }
    }
    std::ostringstream oss1;
    std::ostringstream oss2;
    std::ostringstream oss3;
    std::ostringstream oss4;
    std::ostringstream oss5;
    std::ostringstream oss6;

    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> live = now - timerStart;

    if (stage == 1) {
        oss1 << "Generating using " << generatorString() << "...";
        oss2 << "Nodes visited: " << nodesVisited() << "/" << width * height << " (" << static_cast<float>(nodesVisited()) / (width * height) * 100 << "%)";
        oss3 << "Runtime: " << live.count() << "s";
    } else if (stage > 1) {
        oss1 << "Generated using " << generatorString();
        oss2 << "Nodes visited: " << width * height << "/" << width * height << " (100%)";
        oss3 << "Runtime: " << generatorRuntime << "s";
    }
        
    if (stage == 3) {
        oss4 << "Solving using " << solverString() << "...";
        oss5 << "Nodes visited: " << nodesVisited() << "/" << width * height << " (" << static_cast<float>(nodesVisited()) / (width * height) * 100 << "%)";
        oss6 << "Runtime: " << live.count() << "s";
    }
    if (stage > 3) {
        oss4 << "Solved using " << solverString();
        oss5 << "Nodes visited: " << nodesVisited() << "/" << width * height << " (" << static_cast<float>(nodesVisited()) / (width * height) * 100 << "%)";
        oss6 << "Runtime: " << solverRuntime << "s";
    }

    DrawText(oss1.str().c_str(), 0, 0, 15, RAYWHITE);
    DrawText(oss2.str().c_str(), 0, cellSize, 15, RAYWHITE);
    DrawText(oss3.str().c_str(), 0, cellSize * 2, 15, RAYWHITE);
    DrawText(oss4.str().c_str(), GetScreenWidth() / 2, 0, 15, RAYWHITE);
    DrawText(oss5.str().c_str(), GetScreenWidth() / 2, cellSize, 15, RAYWHITE);
    DrawText(oss6.str().c_str(), GetScreenWidth() / 2, cellSize * 2, 15, RAYWHITE);
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

bool Maze:: isInSolutionPath(int x, int y) {
    return (std::find(solutionPath.begin(), solutionPath.end(), std::pair{x, y}) != solutionPath.end());
}

int Maze::nodesVisited()
{
    int nodes = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (grid[y][x].visited) nodes++;
        }
    }
    return nodes;
}

std::string Maze::generatorString()
{
    switch (generator) {
    case DFS:
        return "Depth-First Search";
        break;
    case PRIMS:
        return "Prim\'s Algorithm";
        break;
    case KRUSKALS:
        return "Kruskal\'s Algorithm";
        break;
    default:
        return "";
        break;
    }
}

std::string Maze::solverString()
{
    switch (solver) {
    case DFS:
        return "Depth-First Search";
        break;
    case BFS:
        return "Breadth-First Search";
        break;
    case ASTAR:
        return "A* Search";
        break;
    case GREEDY:
        return "Greedy Best-First Search";
        break;
    case WALL_LEFT:
        return "Left-Hand Wall Follower";
        break;
    case WALL_RIGHT:
        return "Right-Hand Wall Follower";
        break;
    case DEAD_END:
        return "Dead-End Filler";
        break;
    default:
        return "";
        break;
    }
}
