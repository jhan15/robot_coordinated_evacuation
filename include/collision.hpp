#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>

using namespace cv;

std::vector<Point2d> line_line_coll(std::vector<Point2d> line_a, std::vector<Point2d> line_b);
std::vector<Point2d> circle_line_coll(double a, double b, double r, std::vector<Point2d> line);
bool arc_line_coll(double a, double b, double r, double s, double e, std::vector<Point2d> line);
bool arc_arc_coll(double a1, double b1, double r1, double s1, double e1,
                  double a2, double b2, double r2, double s2, double e2);