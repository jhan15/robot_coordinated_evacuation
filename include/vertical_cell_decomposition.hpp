#include "structs.h"
#include "collision_detect.hpp"
using namespace std;

std::vector<POINT> sort_vertices(std::vector< std::vector<POINT> > obstacles, std::vector<POINT> sorted_vertices, int vertices_num);
std::vector<std::vector<POINT>> close_polygons (std::vector< std::vector<POINT> > polygons);
std::vector<SEGMENT> find_lines(std::vector<POINT> sorted_vertices, std::vector< std::vector<POINT> > obstacles, float y_limit_lower, float y_limit_upper);
POINT centroid(std::vector<POINT> vertices);
std::vector<std::vector<POINT>> find_cells(std::vector<SEGMENT> open_line_segments, std::vector<POINT> sorted_vertices, std::vector< std::vector<POINT> > obstacles);
float polygon_area(std::vector<POINT> vertices, int vertices_num);
std::vector<std::vector<POINT>> merge_polygons(std::vector< std::vector<POINT> > cells);
std::vector<std::vector<POINT>> boundary_cells(std::vector<POINT> boundary, std::vector<std::vector<POINT>> cells, std::vector<POINT> sorted_vertices, float y_limit_lower, float y_limit_upper);
tuple <std::vector<POINT>, std::vector<POINT>> get_graph(std::vector< std::vector<POINT> > cells);
float find_distance(POINT pt1, POINT pt2);
tuple <std::vector<POINT>, std::vector<POINT>> add_start_end(std::vector<POINT> graph_vertices, std::vector<POINT> graph_edges, POINT start_point, POINT end_point, std::vector<std::vector<POINT>> obstacles);
std::vector<std::vector<int> > graph_construction(std::vector<POINT> graph_vertices, std::vector<POINT> graph_edges);
std::vector<int> bfs(std::vector< std::vector<int>> graph, int source, int target);
std::vector<std::vector<int>> bfs_multiple(std::vector< std::vector<int> > graph, int source, int target, int path_count);
std::vector<std::vector<int>> optimize_graph(std::vector<int> my_path, std::vector<POINT> graph_vertices, std::vector<std::vector<POINT>> obstacles);
std::vector<int> look_ahead_optimize(std::vector<int> my_path, std::vector<POINT> graph_vertices, std::vector<std::vector<POINT>> obstacles, float look_ahead, float gamma);
std::vector<robotPos> index_to_coordinates(std::vector<int> index_path, std::vector<POINT> coordinates);
std::vector<SEGMENT> get_boundary_lines(std::vector<POINT> boundary);
std::vector<POINT> offset_end_points (std::vector<SEGMENT> boundary_lines, int robots_number, std::vector<POINT>end_point, float offset_gate_width, float offset_away_from_gate);
void print_data(std::vector<POINT> boundary, POINT start_point, POINT end_point, std::vector< std::vector<POINT> > obstacles, std::vector<POINT> graph_vertices, std::vector<std::vector<int>> graph, std::vector<int> path, std::vector<int> optimized_path, std::vector<robotPos> path_points);    