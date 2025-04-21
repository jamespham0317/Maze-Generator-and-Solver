#include "maze.hpp"
#include <raylib.h>

void handleInput(Maze& maze) {
    if (maze.stage == 0) {
        if (IsKeyDown(KEY_D)) {
            maze.generator = DFS;
            maze.generate();
        } else if (IsKeyDown(KEY_P)) {
            maze.generator = PRIMS;
            maze.generate();
        } else if (IsKeyDown(KEY_K)) {
            maze.generator = KRUSKALS;
            maze.generate();
        }
    } else {
         if (IsKeyDown(KEY_ONE)) {
            maze.solver = DFS;
            maze.solve();
        } else if (IsKeyDown(KEY_TWO)) {
            maze.solver = BFS;
            maze.solve();
        } else if (IsKeyDown(KEY_THREE)) {
            maze.solver = ASTAR;
            maze.solve();
        } else if (IsKeyDown(KEY_FOUR)) {
            maze.solver = GREEDY;
            maze.solve();
        } else if (IsKeyDown(KEY_FIVE)) {
            maze.solver = WALL_LEFT;
            maze.solve();
        } else if (IsKeyDown(KEY_SIX)) {
            maze.solver = WALL_RIGHT;
            maze.solve();
        } else if (IsKeyDown(KEY_SEVEN)) {
            maze.solver = DEAD_END;
            maze.solve();
        } else if (IsKeyDown(KEY_ENTER)) {
            maze.reset();
        }
    }
}

int main() {
    const int width = 20;
    const int height = 20;
    const int cellSize = 20;

    InitWindow(cellSize * (width + 12), cellSize * (height + 12), "Maze Generator and Solver");
    SetTargetFPS(60);

    Maze maze(width, height, cellSize);

    while (!WindowShouldClose()) {
        maze.animate();
        handleInput(maze);
    }

    CloseWindow();
    return 0;
}