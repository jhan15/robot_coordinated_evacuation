#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include <iostream>
#include <string>
#include <vector>
#include "utils.hpp"
#include "../src/clipper/clipper.hpp"

std::vector<Polygon> inflate_obstacles(const std::vector<Polygon> &obstacle_list,float inflate_value, cv::Mat plot);
Polygon inflate_borders(const Polygon &borders, float inflate_value, cv::Mat plot);
std::vector<Polygon> trim_obstacles(const std::vector<Polygon>& obstacle_list,const Polygon &borders, cv::Mat plot);
bool overlap_check(Polygon &r1, Polygon &r2);
std::vector<Polygon> merge_obstacles(const std::vector<Polygon>& obstacle_list,cv::Mat plot);
