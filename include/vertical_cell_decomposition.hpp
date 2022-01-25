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


POINT centroid(std::vector<POINT> vertices);
float polygon_area(std::vector<POINT> vertices, int vertices_num);
float find_dist(POINT pt1, POINT pt2);
bool check_obstruction(std::vector< std::vector<POINT> > obstacles, SEGMENT segment);
std::vector<int> backtrace(std::vector<int> parent, int start, int end);
std::vector<int> bfs(std::vector< std::vector<int> > graph, int source, int target);
POINT intersection_trial(SEGMENT sigment1, SEGMENT sigment2);
// bool points_successive (POINT a, POINT b, std::vector<POINT> obstacle);
// int points_from_same_obs (POINT a, POINT b, std::vector<POINT> obstacles);
// POINT intersection(SEGMENT segment1, SEGMENT segment2);
// float determinant( POINT a , POINT b);
// bool counter_clockwise(POINT A,POINT B,POINT C);
// bool intersect(POINT A,POINT B,POINT C,POINT D,bool print);
// POINT line_intersection(POINT A, POINT B, POINT C, POINT D) ;
// POINT segment_intersection(SEGMENT sigment1, SEGMENT sigment2,bool print);

#endif