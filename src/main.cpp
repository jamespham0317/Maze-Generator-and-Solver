#include "maze.hpp"
#include <raylib.h>

int main() {
    const int width = 30;
    const int height = 30;

    InitWindow(20 * width, 20 * height, "Maze Generator and Solver");

    Maze maze(width, height);

    SetTargetFPS(60);

    maze.generate();
    maze.reset();
    maze.solve();

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        maze.draw(-1, -1, true);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}