#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

using namespace cv;

double crossProd(Point2d a, Point2d b){
	return a.x * b.y - a.y * b.x;
}

double dotProd(Point2d a, Point2d b){
	return a.x * b.x + a.y * b.y;
}

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