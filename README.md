# Maze Generator and Solver 

An interactive maze generator and visual pathfinding tool written in C++ using [Raylib](https://www.raylib.com/). Mazes are generated using randomized Depth-First Search (DFS), and solved using a variety of algorithms with real-time animation.

---

## Features

- Maze generation with randomized DFS
- Animated visualization of solving algorithms
- Solvers include:
  - Depth-First Search (DFS)
  - Breadth-First Search (BFS)
  - A* Search
  - Greedy Best-First Search
  - Wall Follower (Left/Right Hand Rule)
  - Dead-End Filler
  - Bidirectional BFS
- Real-time rendering with `raylib`
- Keyboard input for switching solvers
- Colored highlights for start, goal, visited, and solution paths

---

## Controls

| Key        | Action                              |
|------------|-------------------------------------|
| `1`        | Solve using DFS                     |
| `2`        | Solve using BFS                     |
| `3`        | Solve using A*                      |
| `4`        | Solve using Greedy BFS              |
| `5`        | Solve using Left-Hand Wall Follower |
| `6`        | Solve using Right-Hand Wall Follower|
| `7`        | Solve using Dead-End Filler         |

---

## Requirements

- C++17 or later
- [Raylib](https://www.raylib.com/) (installed locally)
- `make` (for building)

---

## Building the Project

```bash
# Clone the repo
git clone https://github.com/yourusername/mazegeneratorandsolver.git
cd mazegeneratorandsolver

# Build
make

# Run the program
./maze
