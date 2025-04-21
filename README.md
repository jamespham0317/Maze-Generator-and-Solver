# Maze Generator and Solver 

An interactive maze generator and visual pathfinding tool written in C++ using [Raylib](https://www.raylib.com/). Mazes are generated and solved using a variety of algorithms with real-time animation and performance metrics.

---

## Features

- Animated visualization of generation and solving algorithms
- Displays live runtime and number of visited nodes during generation and solving
- Generators include:
  - Randomized Depth-First Search (DFS)
  - Randomized Prim's Algorithm
  - Randomized Kruskal's Algorithm
- Solvers include:
  - Depth-First Search (DFS)
  - Breadth-First Search (BFS)
  - A* Search
  - Greedy Best-First Search
  - Wall Follower (Left/Right Hand Rule)
  - Dead-End Filler
- Real-time rendering with `raylib`
- Keyboard input for switching generators and solvers
- Colored highlights for start, goal, visited, and solution paths

---

## Controls

| Key           | Action                              |
|---------------|-------------------------------------|
| `G`           | Generate using DFS                  |
| `P`           | Generate using Prim's Algorithm     |
| `K`           | Generate using Kruskal's Algorithm  |
| `1`           | Solve using DFS                     |
| `2`           | Solve using BFS                     |
| `3`           | Solve using A*                      |
| `4`           | Solve using Greedy BFS              |
| `5`           | Solve using Left-Hand Wall Follower |
| `6`           | Solve using Right-Hand Wall Follower|
| `7`           | Solve using Dead-End Filler         |
| `↵` or `Enter`| Reset maze                          |

---

## Requirements

- C++ compiler (C++17 or later)
- [Raylib](https://www.raylib.com/) (installed locally)

---

## Building the Project

```bash
# Clone the repo
git clone https://github.com/jamespham0317/Maze-Generator-and-Solver.git
cd mazegeneratorandsolver

# Build
make

# Run the program
./main
