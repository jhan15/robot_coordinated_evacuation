#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "dubins.h"
	
using namespace std;
typedef boost::geometry::model::d2::point_xy<double> point_xy;

#ifndef ROADMAP
#define ROADMAP

struct POINT {
    float x;
    float y;
    float theta;
    int obs;
};

struct SEGMENT {
    POINT a;
    POINT b;
};


std::vector<POINT> sort_vertices(std::vector< std::vector<POINT> > obstacles, std::vector<POINT> sorted_vertices, int vertices_num);
std::vector<std::vector<POINT>> close_polygons (std::vector< std::vector<POINT> > polygons);
POINT intersection_trial(SEGMENT sigment1, SEGMENT sigment2);
std::vector<SEGMENT> find_lines(std::vector<POINT> sorted_vertices, std::vector< std::vector<POINT> > obstacles, float y_limit_lower, float y_limit_upper);
POINT centroid(std::vector<POINT> vertices);
std::vector<std::vector<POINT>> find_cells(std::vector<SEGMENT> open_line_segments, std::vector<POINT> sorted_vertices, std::vector< std::vector<POINT> > obstacles);
float polygon_area(std::vector<POINT> vertices, int vertices_num);
std::vector<std::vector<POINT>> merge_polygons(std::vector< std::vector<POINT> > cells);
std::vector<std::vector<POINT>> boundary_cells(std::vector<POINT> boundary, std::vector<std::vector<POINT>> cells, std::vector<POINT> sorted_vertices, float y_limit_lower, float y_limit_upper);
tuple <std::vector<POINT>, std::vector<POINT>> get_graph(std::vector< std::vector<POINT> > cells);
float find_dist(POINT pt1, POINT pt2);
bool check_obstruction(std::vector< std::vector<POINT> > obstacles, SEGMENT segment);
tuple <std::vector<POINT>, std::vector<POINT>> add_start_end(std::vector<POINT> graph_vertices, std::vector<POINT> graph_edges, POINT start_point, POINT end_point, std::vector<std::vector<POINT>> obstacles);
std::vector<std::vector<int> > graph_construction(std::vector<POINT> graph_vertices, std::vector<POINT> graph_edges);
std::vector<int> backtrace(std::vector<int> parent, int start, int end);
std::vector<int> bfs(std::vector< std::vector<int>> graph, int source, int target);
std::vector<std::vector<int>> optimize_graph(std::vector<int> my_path, std::vector<POINT> graph_vertices, std::vector<std::vector<POINT>> obstacles);
std::vector<int> look_ahead_optimize(std::vector<int> my_path, std::vector<POINT> graph_vertices, std::vector<std::vector<POINT>> obstacles, float look_ahead, float gamma);
std::vector<robotPos> index_to_coordinates(std::vector<int> index_path, std::vector<POINT> coordinates);
void print_data(std::vector<POINT> boundary, std::vector<POINT> start_point, POINT end_point, std::vector< std::vector<POINT> > obstacles, std::vector<POINT> graph_vertices, std::vector< std::vector<int> > graph, std::vector<POINT> new_graph_vertices, std::vector< std::vector<int> > optimized_graph, std::vector<int> path, std::vector<int> optimized_path, std::vector<robotPos> path_points);    
std::vector<std::vector<robotPos>> coordinate_motion(std::vector<std::vector<robotPos>> initial_paths);
std::vector<SEGMENT> get_boundary_lines(std::vector<POINT> boundary);
std::vector<POINT> offset_end_points (std::vector<SEGMENT> boundary_lines,int robots_number, std::vector<POINT>end_point);
std::vector<std::vector<int>> findpaths(std::vector< std::vector<int> > g, int src, int dst, int path_count);
// bool points_successive (POINT a, POINT b, std::vector<POINT> obstacle);
// int points_from_same_obs (POINT a, POINT b, std::vector<POINT> obstacles);
// POINT intersection(SEGMENT segment1, SEGMENT segment2);
// float determinant( POINT a , POINT b);
// bool counter_clockwise(POINT A,POINT B,POINT C);
// bool intersect(POINT A,POINT B,POINT C,POINT D,bool print);
// POINT line_intersection(POINT A, POINT B, POINT C, POINT D) ;
// POINT segment_intersection(SEGMENT sigment1, SEGMENT sigment2,bool print);

#endif