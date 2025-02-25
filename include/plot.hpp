#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <iterator>
#include <string> 

#include <cmath>
#include "dubins.h"
#include "inflate_objects.hpp"
#include "vertical_cell_decomposition.hpp"

void plot_map (cv::Mat plot,int robot_num, std::vector<POINT> sorted_vertices, std::vector<std::vector<POINT>> cells, POINT start_point, POINT end_point, std::vector<std::vector<int>> graph, std::vector<POINT> graph_vertices, std::vector<int> my_path,std::vector<POINT> graph_chosen, std::vector<int> path);
void plot_dubins (cv::Mat plot,std::vector<Path> path, int robots_number); 
void plot_lines (cv::Mat plot,std::vector<std::vector<robotPos>> path, int robots_number);
