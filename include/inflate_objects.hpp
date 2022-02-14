#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include <iostream>
#include <string>
#include <vector>
#include "utils.hpp"
#include "clipper/clipper.hpp"
#include "boost/geometry/geometry.hpp"
#include <boost/assign/std/vector.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/dsv/write.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/foreach.hpp>

std::vector<Polygon> inflate_obstacles(const std::vector<Polygon> &obstacle_list,float inflate_value,bool simplify, cv::Mat plot);
Polygon inflate_borders(const Polygon &borders, float inflate_value, cv::Mat plot);
std::vector<Polygon> trim_obstacles(const std::vector<Polygon>& obstacle_list,const Polygon &borders, cv::Mat plot);
std::vector<Polygon> trim_obstacles_old(const std::vector<Polygon>& obstacle_list,const Polygon &borders, cv::Mat plot);
bool overlap_check(Polygon &r1, Polygon &r2);
std::vector<Polygon> merge_obstacles(const std::vector<Polygon>& obstacle_list,bool simplify, cv::Mat plot);

using namespace boost::assign;
typedef boost::geometry::model::d2::point_xy<double> point_xy;
typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
using boost::geometry::correct;