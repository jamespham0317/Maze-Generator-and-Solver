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

Maze::Maze(int width, int height, int cellSize) : width(width), height(height), cellSize(cellSize)
{
    grid.resize(height, std::vector<Cell>(width));
    std::mt19937 rng(std::random_device{}());
}

void Maze::generate()
{
    reset();
    stage = GENERATING;
    timerStart = std::chrono::high_resolution_clock::now();
    switch (generator)
    {
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

void Maze::solve()
{
    resetMaze();
    stage = SOLVING;
    timerStart = std::chrono::high_resolution_clock::now();
    switch (solver)
    {
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
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            grid[y][x].visited = false;
        }
    }
}

void Maze::animate(int currentX, int currentY)
{
    BeginDrawing();
    ClearBackground({12, 12, 22, 255});
    draw(currentX, currentY);
    EndDrawing();
    WaitTime(0.01);
}

void Maze::draw(int currentX, int currentY)
{
    // ---- Colour palette ----
    Color bgColor = {12, 12, 22, 255};
    Color cellEmpty = {28, 28, 50, 255};
    Color cellVisitedGen = {50, 50, 90, 255};
    Color cellVisitedSol = {55, 85, 110, 255};
    Color wallColor = {185, 185, 215, 255};
    Color cellCurrent = {255, 195, 50, 255};
    Color cellStart = {45, 195, 95, 255};
    Color cellGoal = {215, 55, 75, 255};
    Color cellSolution = {65, 155, 235, 255};
    Color sidebarBgColor = {18, 18, 32, 255};
    Color panelBgColor = {22, 22, 42, 255};
    Color borderColor = {55, 55, 95, 255};
    Color accentColor = {95, 115, 240, 255};
    Color textColor = {215, 215, 235, 255};
    Color dimColor = {110, 110, 145, 255};
    Color activeKeyBg = {50, 50, 85, 255};
    Color activeKeyText = {255, 195, 50, 255};

    // ---- Layout ----
    const int offset = 6;
    const int mazeAreaW = (width + 12) * cellSize;
    const int sidebarX = mazeAreaW;
    const int sidebarW = GetScreenWidth() - mazeAreaW;

    DrawRectangle(0, 0, mazeAreaW, GetScreenHeight(), bgColor);
    DrawRectangle(sidebarX, 0, sidebarW, GetScreenHeight(), sidebarBgColor);
    DrawRectangle(sidebarX, 0, 2, GetScreenHeight(), borderColor);

    // ---- Draw maze grid ----
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int px = (x + offset) * cellSize;
            int py = (y + offset) * cellSize;

            Color tile = cellEmpty;
            if (stage < GENERATED)
            {
                if (grid[y][x].visited)
                    tile = cellVisitedGen;
            }
            else
            {
                if (isStart(x, y))
                    tile = cellStart;
                else if (isGoal(x, y))
                    tile = cellGoal;
                else if (isInSolutionPath(x, y))
                    tile = cellSolution;
                else if (grid[y][x].visited)
                    tile = cellVisitedSol;
            }
            if (x == currentX && y == currentY)
                tile = cellCurrent;

            DrawRectangle(px, py, cellSize, cellSize, tile);
            if (grid[y][x].walls[0])
                DrawRectangle(px, py, cellSize, 1, wallColor);
            if (grid[y][x].walls[1])
                DrawRectangle(px + cellSize, py, 1, cellSize, wallColor);
            if (grid[y][x].walls[2])
                DrawRectangle(px, py + cellSize, cellSize, 1, wallColor);
            if (grid[y][x].walls[3])
                DrawRectangle(px, py, 1, cellSize, wallColor);
        }
    }

    // Idle overlay
    if (stage == INITIALIZED)
    {
        const char *prompt = "Press D, P, or K to generate";
        int tw = MeasureText(prompt, 18);
        int tx = mazeAreaW / 2 - tw / 2;
        int ty = GetScreenHeight() / 2 - 15;
        DrawRectangle(tx - 14, ty - 12, tw + 28, 46, {22, 22, 42, 220});
        DrawRectangleLines(tx - 14, ty - 12, tw + 28, 46, borderColor);
        DrawText(prompt, tx, ty, 18, textColor);
    }

    // ---- Sidebar ----
    const int pad = 14;
    int sx = sidebarX + pad;
    int sw = sidebarW - pad * 2;
    int cy = pad;

    DrawText("MAZE", sx, cy, 26, accentColor);
    DrawText("GENERATOR & SOLVER", sx, cy + 32, 12, dimColor);
    cy += 56;
    DrawRectangle(sx, cy, sw, 1, borderColor);
    cy += 12;

    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> live = now - timerStart;

    auto panelHeader = [&](const char *label, int y)
    {
        DrawRectangle(sx, y, sw, 24, panelBgColor);
        DrawRectangleLines(sx, y, sw, 24, borderColor);
        DrawRectangle(sx, y, 3, 24, accentColor);
        DrawText(label, sx + 10, y + 5, 13, accentColor);
    };

    // ---- Generator panel ----
    panelHeader("GENERATOR", cy);
    cy += 28;
    {
        const int bodyH = 76;
        DrawRectangle(sx, cy, sw, bodyH, panelBgColor);
        DrawRectangleLines(sx, cy, sw, bodyH, borderColor);
        if (stage >= GENERATING)
        {
            int total = width * height;
            int visited = (stage == GENERATING) ? nodesVisited() : total;
            double rtime = (stage == GENERATING) ? live.count() : generatorRuntime;
            int pct = (int)((float)visited / total * 100);
            DrawText(generatorString().c_str(), sx + 8, cy + 8, 13, textColor);
            std::ostringstream ns;
            ns << "Nodes:  " << visited << " / " << total << "  (" << pct << "%)";
            DrawText(ns.str().c_str(), sx + 8, cy + 30, 13, textColor);
            std::ostringstream rt;
            rt.precision(4);
            rt << std::fixed << "Time:   " << rtime << "s";
            DrawText(rt.str().c_str(), sx + 8, cy + 52, 13, textColor);
        }
        else
        {
            DrawText("No maze generated yet.", sx + 8, cy + 28, 12, dimColor);
        }
        cy += bodyH + 10;
    }

    // ---- Solver panel ----
    panelHeader("SOLVER", cy);
    cy += 28;
    {
        const int bodyH = 76;
        DrawRectangle(sx, cy, sw, bodyH, panelBgColor);
        DrawRectangleLines(sx, cy, sw, bodyH, borderColor);
        if (stage >= SOLVING)
        {
            int total = width * height;
            int visited = nodesVisited();
            double rtime = (stage == SOLVING) ? live.count() : solverRuntime;
            int pct = (int)((float)visited / total * 100);
            DrawText(solverString().c_str(), sx + 8, cy + 8, 13, textColor);
            std::ostringstream ns;
            ns << "Nodes:  " << visited << " / " << total << "  (" << pct << "%)";
            DrawText(ns.str().c_str(), sx + 8, cy + 30, 13, textColor);
            std::ostringstream rt;
            rt.precision(4);
            rt << std::fixed << "Time:   " << rtime << "s";
            DrawText(rt.str().c_str(), sx + 8, cy + 52, 13, textColor);
        }
        else
        {
            DrawText("No solution yet.", sx + 8, cy + 28, 12, dimColor);
        }
        cy += bodyH + 10;
    }

    // ---- Controls panel ----
    panelHeader("CONTROLS", cy);
    cy += 28;
    {
        struct Binding
        {
            const char *key;
            const char *desc;
            bool active;
        };
        Binding bindings[] = {
            {"D", "Generate  (DFS)", stage == INITIALIZED},
            {"P", "Generate  (Prim's)", stage == INITIALIZED},
            {"K", "Generate  (Kruskal's)", stage == INITIALIZED},
            {"1", "Solve  (DFS)", stage >= GENERATED},
            {"2", "Solve  (BFS)", stage >= GENERATED},
            {"3", "Solve  (A*)", stage >= GENERATED},
            {"4", "Solve  (Greedy BFS)", stage >= GENERATED},
            {"5", "Solve  (Wall Left)", stage >= GENERATED},
            {"6", "Solve  (Wall Right)", stage >= GENERATED},
            {"7", "Solve  (Dead End Fill)", stage >= GENERATED},
            {"ENTER", "Reset", stage >= GENERATED},
        };
        const int n = (int)(sizeof(bindings) / sizeof(bindings[0]));
        const int bodyH = n * 20 + 12;
        DrawRectangle(sx, cy, sw, bodyH, panelBgColor);
        DrawRectangleLines(sx, cy, sw, bodyH, borderColor);
        int ky = cy + 6;
        for (int i = 0; i < n; i++)
        {
            bool on = bindings[i].active;
            Color kBg = on ? activeKeyBg : Color{28, 28, 48, 255};
            Color kText = on ? activeKeyText : dimColor;
            Color dText = on ? textColor : dimColor;
            int kw = MeasureText(bindings[i].key, 11) + 8;
            DrawRectangle(sx + 6, ky, kw, 16, kBg);
            DrawText(bindings[i].key, sx + 10, ky + 2, 11, kText);
            DrawText(bindings[i].desc, sx + 6 + kw + 8, ky + 2, 11, dText);
            ky += 20;
        }
        cy += bodyH + 10;
    }

    // ---- Legend panel ----
    panelHeader("LEGEND", cy);
    cy += 28;
    {
        struct LegendItem
        {
            const char *label;
            Color color;
        };
        LegendItem items[] = {
            {"Start cell", cellStart},
            {"Goal cell", cellGoal},
            {"Solution", cellSolution},
            {"Visited", cellVisitedSol},
            {"Current cell", cellCurrent},
        };
        const int n = (int)(sizeof(items) / sizeof(items[0]));
        const int bodyH = n * 20 + 12;
        DrawRectangle(sx, cy, sw, bodyH, panelBgColor);
        DrawRectangleLines(sx, cy, sw, bodyH, borderColor);
        int ly = cy + 6;
        for (int i = 0; i < n; i++)
        {
            DrawRectangle(sx + 6, ly + 1, 14, 14, items[i].color);
            DrawText(items[i].label, sx + 26, ly + 2, 11, textColor);
            ly += 20;
        }
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

bool Maze::isInSolutionPath(int x, int y)
{
    return (std::find(solutionPath.begin(), solutionPath.end(), std::pair{x, y}) != solutionPath.end());
}

int Maze::nodesVisited()
{
    int nodes = 0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (grid[y][x].visited)
                nodes++;
        }
    }
    return nodes;
}

std::string Maze::generatorString()
{
    switch (generator)
    {
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
    switch (solver)
    {
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
