#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>

using namespace cv;

//checks if 2 line segments are intersect.
std::vector<Point2d> line_line_coll(std::vector<Point2d> line_a, std::vector<Point2d> line_b);

//checks if a line segment and a circle are intersected.
std::vector<Point2d> circle_line_coll(double a, double b, double r, std::vector<Point2d> line);

//checks if a line segment and an arc of circle are intersected.
bool arc_line_coll(double a, double b, double r, double s, double e, std::vector<Point2d> line);

//check if two arcs are intersected (not used)
bool arc_arc_coll(double a1, double b1, double r1, double s1, double e1,
                  double a2, double b2, double r2, double s2, double e2);