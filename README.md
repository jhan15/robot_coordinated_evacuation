# Laboratory of Applied Robotics Student Interface
Package used by student to complete the assignment of the course.

## Setup

```bash
$ cd workspace/project
$ git pull https://github.com/ymarkova/theEITgroup.git
$ mkdir build
$ cd build
$ cmake ..
$ make
$ source ../environment.sh
```

## Usage

```bash
$ cd workspace/simulator
$ AR_simulator_gui # AR_simulator
$ AR_pipeline
# Select the 4 black corners counter-clockwise starting from the one near the red line
# Then stop (ctrl + C)
$ AR_pipeline
$ AR_rviz
$ AR_run
```

## Other usage

```bash
# Modify the environment
$ gedit $AR_map_file

# Disable demo implementation
$ cd $AR_config_dir
$ gedit default_implementation.config
# Then set flag to <False> in order to implement our own solution
```