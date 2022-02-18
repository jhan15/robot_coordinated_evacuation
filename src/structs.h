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
#include <bits/stdc++.h>
#include "collision_detect.hpp"




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


typedef boost::geometry::model::d2::point_xy<double> point_boost;
#endif