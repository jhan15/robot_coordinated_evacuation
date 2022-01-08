#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include <iostream>
#include <string>
#include <vector>
#include "utils.hpp"
#include "../src/clipper/clipper.hpp"

std::vector<Polygon> inflate_obstacles(const std::vector<Polygon> &obstacle_list,int inflate_value);
Polygon inflate_borders(const Polygon &borders, int inflate_value);