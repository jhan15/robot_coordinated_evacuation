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
	
using namespace std;

std::vector<std::vector<robotPos>> coordinate_motion(std::vector<std::vector<robotPos>> initial_paths);