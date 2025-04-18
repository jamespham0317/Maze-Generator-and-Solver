#include "maze.hpp"
#include <raylib.h>

void handleInput(Maze& maze) {
    if (IsKeyDown(KEY_ONE)) {
        maze.solveDFS();
    } else if (IsKeyDown(KEY_TWO)) {
        maze.solveBFS();
    } else if (IsKeyDown(KEY_ENTER)) {
        maze.reset();
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