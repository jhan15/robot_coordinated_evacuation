#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>
#include "collision.hpp"
#include "dubins.h"

using namespace cv;


/*
function that performs a cross product
*/
double cross_prod(Point2d a, Point2d b){
	return a.x * b.y - a.y * b.x;
}
/*
function that performs a dot product
*/
double dot_prod(Point2d a, Point2d b){
	return a.x * b.x + a.y * b.y;
}
/*
function to calculate distance between two points
*/
double eucl_distance(Point2d a, Point2d b){
	return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2));
}
/*
checks if 2 lines intersect [collide]. returns a vector of points of intersection if any
*/
std::vector<Point2d> line_line_coll(std::vector<Point2d> line_a, std::vector<Point2d> line_b){
    std::vector<Point2d> pts;
	double t;
	std::vector<double> ts;

	double x_a1 = line_a[0].x;
	double y_a1 = line_a[0].y;

	double x_a2 = line_a[1].x;
	double y_a2 = line_a[1].y;

	double x_b1 = line_b[0].x;
	double y_b1 = line_b[0].y;

	double x_b2 = line_b[1].x;
	double y_b2 = line_b[1].y;

	double minX_a = min(x_a1,x_a2);
	double minY_a = min(y_a1,y_a2);
	double maxX_a = max(x_a1,x_a2);
	double maxY_a = max(y_a1,y_a2);
	   
	double minX_b = min(x_b1,x_b2);
	double minY_b = min(y_b1,y_b2);
	double maxX_b = max(x_b1,x_b2);
	double maxY_b = max(y_b1,y_b2);


	if (maxX_b<minX_a || minX_b>maxX_a || maxY_b<minY_a || minY_b>maxY_a) {
		return pts;
	}

	Point2d q = Point2d(x_a1, y_a1);
	Point2d s = Point2d(x_a2-q.x, y_a2-q.y);

	Point2d p = Point2d(x_b1, y_b1);
	Point2d r = Point2d(x_b2-p.x, y_b2-p.y);

	Point2d diffPQ = Point2d(q.x-p.x, q.y-p.y);

	double crossRS = cross_prod(r, s);
	double crossDiffR = cross_prod(diffPQ,r);
   	double crossDiffS = cross_prod(diffPQ,s);

   	if(crossRS == 0 && crossDiffR == 0){
   		double dotRR = dot_prod(r,r);
       	double dotSR = dot_prod(s,r);
       	double t0 = dot_prod(diffPQ,r)/dotRR;
       	double t1 = t0+dotSR/dotRR;
       	if(dotSR<0){
       		if(t0>=0 && t0<=1){
       			ts = {max(t1,0.0), min(t0,1.0)};
       		}
       	}else{
       		if(t1>=0 && t0<=1){
       			ts = {max(t0,0.0), min(t1,1.0)};
       		}
       	}
   	}else{
   		if(crossRS == 0 && crossDiffR != 0){
   			return pts;
   		}else{
   			t = crossDiffS/crossRS;
   			double u = crossDiffR/crossRS;
   			if(t>=0 && t<=1 && u>=0 && u<=1){
   				ts = {t};
   			}
   		}
   	}

   	for(int t=0; t<ts.size(); t++){
   		Point2d pt = Point2d(p.x+ts[t]*r.x, p.y+ts[t]*r.y);
   		pts.emplace_back(pt);
   	}
   	
   	return pts;
}
/*
checks if a line and a circle intersect [collide]. returns a vector of points of intersection if any
*/
std::vector<Point2d> circle_line_coll(double a, double b, double r, std::vector<Point2d> line){
    std::vector<Point2d> pts;
	std::vector<double> t;

	double line_x1 = line[0].x;
	double line_y1 = line[0].y;

	double line_x2 = line[1].x;
	double line_y2 = line[1].y;

	double p1 = 2*line_x1*line_x2;
    double p2 = 2*line_y1*line_y2;
    double p3 = 2*a*line_x1;
    double p4 = 2*a*line_x2;
    double p5 = 2*b*line_y1;
    double p6 = 2*b*line_y2;

    double c1 = pow(line_x1,2)+pow(line_x2,2)-p1+ pow(line_y1,2)+pow(line_y2,2)-p2;
    double c2 = -2*pow(line_x2,2)+p1-p3+p4-2*pow(line_y2,2)+p2-p5+p6;
    double c3 = pow(line_x2,2)-p4+pow(a,2)+pow(line_y2,2)-p6+pow(b,2)-pow(r,2);

    double delta = pow(c2,2)-4*c1*c3;
    double t1, t2, x, y;

    if(delta<0){
    	return pts;
    }
    else{
    	if(delta>0){
    		double deltaSq = sqrt(delta);
    		t1 = (-c2+deltaSq)/(2*c1);
        	t2 = (-c2-deltaSq)/(2*c1);
    	}
    	else{
    		t1 = -c2/(2*c1);
        	t2 = t1;
    	}
    }

    if(t1>=0 && t1<=1){
    	x = line_x1*t1+line_x2*(1-t1);
    	y = line_y1*t1+line_y2*(1-t1);
    	pts.emplace_back(Point2d(x,y));
    	t.emplace_back(t1);
    }

    if(t2 >=0 && t2<=1 && t2!=t1){
	    x = line_x1*t2+line_x2*(1-t2);
	    y = line_y1*t2+line_y2*(1-t2);
	    pts.emplace_back(Point2d(x,y));
    	t.emplace_back(t2);
	}

    return pts;

}

bool arc_pass_intersection(float theta, double s, double e){
	if (s < e && theta >= s && theta <= e)
	{
		return true;
	}
	if (s > e && !(theta > e && theta < s))
	{
		return true;
	}

	return false;
}

bool arc_line_coll(double a, double b, double r, double s, double e, std::vector<Point2d> line)
{
	bool result = false;

	std::vector<Point2d> pts = circle_line_coll(a, b, r, line);

	if (pts.size() > 0)
	{
		for (auto it = pts.begin(); it != pts.end(); it++)
		{
			float theta = atan2((*it).y-b, (*it).x-a);
			theta = mod2Pi(theta);
			result = arc_pass_intersection(theta, s, e);
		}
	}

	return result;
}

bool arc_arc_coll(double a1, double b1, double r1, double s1, double e1,
                  double a2, double b2, double r2, double s2, double e2)
{
	double d = eucl_distance(Point2d(a1,b1), Point2d(a2,b2));
	
	if (d <= (r1+r2) and d >= abs(r1-r2)){
		double l = (r1*r1 - r2*r2 + d*d) / (2*d);
		double h = sqrt(r1*r1 - l*l);

		if (h > 0) {
			double x1 = l*(a2-a1)/d + h*(b2-b1)/d + a1;
			double y1 = l*(b2-b1)/d - h*(a2-a1)/d + b1;
			double x2 = l*(a2-a1)/d - h*(b2-b1)/d + a1;
			double y2 = l*(b2-b1)/d + h*(a2-a1)/d + b1;
			//std::cout<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<std::endl;

			float theta11 = mod2Pi(atan2(y1-b1,x1-a1));
			float theta12 = mod2Pi(atan2(y1-b2,x1-a2));
			float theta21 = mod2Pi(atan2(y2-b1,x2-a1));
			float theta22 = mod2Pi(atan2(y2-b2,x2-a2));

			// both pass (x1,y1) or both pass (x2,y2)
			bool re11 = arc_pass_intersection(theta11, s1, e1);
			bool re12 = arc_pass_intersection(theta12, s2, e2);
			bool re21 = arc_pass_intersection(theta21, s1, e1);
			bool re22 = arc_pass_intersection(theta22, s2, e2);

			if ((re11 && re12) || (re21 && re22)) return true;
		}
		else{
			double x = l*(a2-a1)/d + a1;
			double y = l*(b2-b1)/d + b1;

			float theta1 = mod2Pi(atan2(y-b1,x-a1));
			float theta2 = mod2Pi(atan2(y-b2,x-a2));

			bool re1 = arc_pass_intersection(theta1, s1, e1);
			bool re2 = arc_pass_intersection(theta2, s2, e2);

			if (re1 && re2) return true;
		}
	}
	
	return false;
}
