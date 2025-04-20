#include "maze.hpp"
#include <raylib.h>

void handleInput(Maze& maze) {
    if (!maze.isGenerated) {
        if (IsKeyDown(KEY_D)) {
            maze.generateDFS();
        } else if (IsKeyDown(KEY_P)) {
            maze.generatePrims();
        } else if (IsKeyDown(KEY_K)) {
            maze.generateKruskals();
        }
    } else {
         if (IsKeyDown(KEY_ONE)) {
            maze.resetMaze();
            maze.solveDFS();
        } else if (IsKeyDown(KEY_TWO)) {
            maze.resetMaze();
            maze.solveBFS();
        } else if (IsKeyDown(KEY_THREE)) {
            maze.resetMaze();
            maze.solveAStar();
        } else if (IsKeyDown(KEY_FOUR)) {
            maze.resetMaze();
            maze.solveGreedyBFS();
        } else if (IsKeyDown(KEY_FIVE)) {
            maze.resetMaze();
            maze.solveWallFollower(0, 0, RIGHT, true);
        } else if (IsKeyDown(KEY_SIX)) {
            maze.resetMaze();
            maze.solveWallFollower(0, 0, RIGHT, false);
        } else if (IsKeyDown(KEY_SEVEN)) {
            maze.resetMaze();
            maze.solveDeadEndFiller();
        } else if (IsKeyDown(KEY_ENTER)) {
            maze.reset();
        }
    }
}

int main() {
    const int width = 20;
    const int height = 20;
    const int cellSize = 20;

    InitWindow(cellSize * width, cellSize * height, "Maze Generator and Solver");
    SetTargetFPS(60);

    Maze maze(width, height, cellSize);

    while (!WindowShouldClose()) {
        maze.animate();
        handleInput(maze);
    }

    CloseWindow();
    return 0;
}