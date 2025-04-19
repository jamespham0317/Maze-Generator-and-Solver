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
        maze.solveWallFollower(true);
    } else if (IsKeyDown(KEY_FIVE)) {
        maze.reset();
        maze.solveWallFollower(false);
    }
}

int main() {
    const int width = 30;
    const int height = 30;

    InitWindow(20 * width, 20 * height, "Maze Generator and Solver");

    Maze maze(width, height);

    SetTargetFPS(60);

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