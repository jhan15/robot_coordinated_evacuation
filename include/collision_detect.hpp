#include "structs.h"
#include "dubins.h"
#include "utils.hpp"


POINT segment_intersection(SEGMENT sigment1, SEGMENT sigment2);

bool check_obstruction(std::vector< std::vector<POINT> > obstacles, SEGMENT segment);

std::vector<cv::Point2d> line_line_coll(std::vector<cv::Point2d> line_a, std::vector<cv::Point2d> line_b);

//checks if a line segment and a circle are intersected.
std::vector<cv::Point2d> circle_line_coll(double a, double b, double r, std::vector<cv::Point2d> line);

//checks if a line segment and an arc of circle are intersected.
bool arc_line_coll(double a, double b, double r, double s, double e, std::vector<cv::Point2d> line);

//check if two arcs are intersected (not used)
bool arc_arc_coll(double a1, double b1, double r1, double s1, double e1,
                  double a2, double b2, double r2, double s2, double e2);

bool overlap_check(const Polygon &pol1, const Polygon &pol2);
std::tuple<std::vector<std::vector<float> >,std::vector<std::vector<float> >,std::vector<float>,std::vector<std::vector<SEGMENT> >> calculate_distances(std::vector<std::vector<robotPos>> path);
std::vector<std::vector<int> > path_intersect_check(std::vector<std::vector<float> > segment_distnace,std::vector<std::vector<float> > cumulative_distance,std::vector<float> total_path_dist,std::vector<std::vector<SEGMENT> > path_segments,cv::Mat plot,bool debug);
