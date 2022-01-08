#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <algorithm>
	
using namespace std;

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

POINT intersection(SEGMENT segment1, SEGMENT segment2);
POINT centroid(std::vector<POINT> vertices);
float polygon_area(std::vector<POINT> vertices, int vertices_num);
float find_dist(POINT pt1, POINT pt2);
int check_obstruction(std::vector< std::vector<POINT> > obstacles, SEGMENT segment);
std::vector<int> backtrace(std::vector<int> parent, int start, int end);
std::vector<int> bfs(std::vector< std::vector<int> > graph, int source, int target);

#endif