#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <queue>
#include <opencv2/opencv.hpp>

#include "boost/geometry/geometry.hpp"
#include <boost/assign/std/vector.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/io/dsv/write.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/foreach.hpp>


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
typedef boost::geometry::model::polygon<point_boost > polygon_boost;
using namespace boost::assign;
using boost::geometry::correct;

static float enlarge = 600.0;
#endif