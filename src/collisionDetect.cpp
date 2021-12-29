#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>

using namespace cv;

std::vector<Point2d> LineLineColl(std::vector<Point2d> line_a, std::vector<Point2d> line_b);
std::vector<Point2d> CircleLineColl(double a, double b, int r, std::vector<Point2d> line);



/*
function that performs a cross product
*/
double crossProd(Point2d a, Point2d b){
	return a.x * b.y - a.y * b.x;
}
/*
function that performs a dot product
*/
double dotProd(Point2d a, Point2d b){
	return a.x * b.x + a.y * b.y;
}
/*
checks if 2 lines intersect [collide]. returns a vector of points of intersection if any
*/
std::vector<Point2d> LineLineColl(std::vector<Point2d> line_a, std::vector<Point2d> line_b){
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

	double crossRS = crossProd(r, s);
	double crossDiffR = crossProd(diffPQ,r);
   	double crossDiffS = crossProd(diffPQ,s);

   	if(crossRS == 0 && crossDiffR == 0){
   		double dotRR = dotProd(r,r);
       	double dotSR = dotProd(s,r);
       	double t0 = dotProd(diffPQ,r)/dotRR;
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
std::vector<Point2d> CircleLineColl(double a, double b, int r, std::vector<Point2d> line){
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
