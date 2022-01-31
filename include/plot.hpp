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

void plot_map (cv::Mat plot, std::vector<POINT> sorted_vertices, std::vector<std::vector<POINT>> cells, std::vector<POINT> start_point, std::vector<POINT> end_point, std::vector<std::vector<int>> graph, std::vector<POINT> graph_vertices, std::vector<POINT> new_graph_vertices, std::vector<int> optimized_path, std::vector<int> my_path); 



