# robot_coordinated_evacuation
The project is about designing a coordinated evacuation of a site cluttered by several obstacles. Three robots have to react to an emergency reaching a gate in minimum time.

**Requirements**

* The robots move at a constant speed (so they can execute Dubins Manoeuvres).
* The robots have to move without touching the border of the map and the obstacles.
* The robots must not collide with each other.

## Structure

  ~/workspace
        |__ project
        |__ simulator

## Setup

```bash
$ mkdir ~/workspace
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
