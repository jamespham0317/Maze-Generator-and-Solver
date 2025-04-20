#include "maze.hpp"
#include <raylib.h>

void handleInput(Maze& maze) {
    if (IsKeyDown(KEY_ONE)) {
        maze.reset();
        maze.solveDFS();
    } else if (IsKeyDown(KEY_TWO)) {
        maze.reset();
        maze.solveBFS();
    } else if (IsKeyDown(KEY_THREE)) {
        maze.reset();
        maze.solveAStar();
    } else if (IsKeyDown(KEY_FOUR)) {
        maze.reset();
        maze.solveGreedyBFS();
    } else if (IsKeyDown(KEY_FIVE)) {
        maze.reset();
        maze.solveWallFollower(0, 0, RIGHT, true);
    } else if (IsKeyDown(KEY_SIX)) {
        maze.reset();
        maze.solveWallFollower(0, 0, RIGHT, false);
    } else if (IsKeyDown(KEY_SEVEN)) {
        maze.reset();
        maze.solveDeadEndFiller();
    }
}

int main() {
    const int width = 60;
    const int height = 30;
    const int cellSize = 20;

    InitWindow(cellSize * width, cellSize * height, "Maze Generator and Solver");
    SetTargetFPS(60);

    Maze maze(width, height, cellSize);
    maze.generate();
    maze.reset();

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        maze.drawSolve();
        EndDrawing();
        handleInput(maze);
    }

    CloseWindow();
    return 0;
}