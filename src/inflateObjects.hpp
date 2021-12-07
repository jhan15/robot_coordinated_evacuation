#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include <iostream>
#include <string>
#include <vector>
#include "utils.hpp"
#include "clipper/clipper.hpp"

std::vector<Polygon> inflateObstacles(const std::vector<Polygon> &obstacle_list, const Polygon &borders);
Polygon inflateBorders(const Polygon &borders, cv::Mat &plot);