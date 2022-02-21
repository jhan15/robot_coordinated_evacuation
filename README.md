# robot_coordinated_evacuation
The project is about designing a coordinated evacuation of a site cluttered by several obstacles. Three robots have to react to an emergency reaching a gate in minimum time.

**Requirements**

* The robots move at a constant speed (so they can execute Dubins Manoeuvres).
* The robots have to move without touching the border of the map and the obstacles.
* The robots must not collide with each other.

**Solution**

* A roadmap is created via Vertical Cell Decomposition algorithm
* A path is computed from roadmap via Breadth First Search algorithm
* Path optimization is conducted by looking-ahead the path vertices
* Collision-free Multipoint Markov-Dubins curves are computed via Iterative Dynamic Programming
* Asynchronous coordination is conducted via a time-step-based collision checking for the three robots
* If a collision occurs, the second best path from BFS is employed for one robot

**Demo**

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/154967519-4408b8aa-4c82-4196-9789-e3fb88f9ccbb.gif?raw=true">
</p>

## Structure

    ~/workspace
        |__ project
        |__ simulator

## Setup

### Simulator

```bash
$ mkdir ~/workspace
$ cd workspace
$ git clone https://github.com/AlexRookie/AppliedRoboticsEnvironment.git simulator/
$ cd simulator
$ catkin build
$ source ./environment.sh
```

### Planner
```bash
$ cd workspace
$ git clone https://github.com/jhan15/robot_coordinated_evacuation.git project
$ cd project
$ mkdir build
$ cd build
$ cmake ..
$ make
$ source ../environment.sh
```

## Usage

### Run simulation
```bash
# terminor 1
$ AR_simulator
# terminal 2
$ AR_pipeline
# terminal 3
$ AR_rviz
# terminal 4
$ AR_run
```

### Other usage
```bash
# Modify the environment
$ gedit $AR_map_file
```
