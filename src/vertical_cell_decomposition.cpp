#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <algorithm>
#include "vertical_cell_decomposition.hpp"
	
using namespace std;

//Function to determine the intersection of two segments; source https://flassari.is/2008/11/line-line-intersection-in-cplusplus/
POINT intersection(SEGMENT segment1, SEGMENT segment2) {   
    POINT intersection_point;

    float d = (segment1.a.x - segment1.b.x) * (segment2.a.y - segment2.b.y) - (segment1.a.y - segment1.b.y) * (segment2.a.x - segment2.b.x);

    if (d == 0) {
		intersection_point.x = (-1);
    	intersection_point.y = (-1);
        return intersection_point;
    }

    // Get the x and y
    float pre = (segment1.a.x*segment1.b.y - segment1.a.y*segment1.b.x); 
    float post = (segment2.a.x*segment2.b.y - segment2.a.y*segment2.b.x);
    float x = ( pre * (segment2.a.x - segment2.b.x) - (segment1.a.x - segment1.b.x) * post ) / d;
    float y = ( pre * (segment2.a.y - segment2.b.y) - (segment1.a.y - segment1.b.y) * post ) / d;

    if ( int(1000000*x) < int(1000000*min(segment1.a.x, segment1.b.x)) || int(1000000*x) > int(1000000*max(segment1.a.x, segment1.b.x)) || int(1000000*x) < int(1000000*min(segment2.a.x, segment2.b.x)) || int(1000000*x) > int(1000000*max(segment2.a.x, segment2.b.x)) ) {
		intersection_point.x = (-1);
    	intersection_point.y = (-1);
        return intersection_point;
    }
    if ( int(1000000*y) < int(1000000*min(segment1.a.y, segment1.b.y)) || int(1000000*y) > int(1000000*max(segment1.a.y, segment1.b.y)) || int(1000000*y) < int(1000000*min(segment2.a.y, segment2.b.y)) || int(1000000*y) > int(1000000*max(segment2.a.y, segment2.b.y)) ) {
		intersection_point.x = (-1);
    	intersection_point.y = (-1);
        return intersection_point;
    }

    intersection_point.x = x; //(int(x+0.5));
    intersection_point.y = y; //(int(y+0.5));


    return intersection_point;
}

//Finding the centroid of a list of vertices
POINT centroid(std::vector<POINT> vertices) {   

    POINT centroid_point = {-1,-1};
	int num = vertices.size();
    float sum_x = 0;
    float sum_y = 0;

    if(num == 0) {
        return centroid_point;
    }

    for(int vert_idx = 0; vert_idx < num; vert_idx++) {
        sum_x += vertices[vert_idx].x;
        sum_y += vertices[vert_idx].y;
    }
    //centroid_point.x = (int(0.5 + sum_x/num));
    //centroid_point.y = (int(0.5 + sum_y/num));
    centroid_point = {sum_x/num,sum_y/num};

    return centroid_point;
}

float polygon_area(std::vector<POINT> vertices, int vertices_num) {
	float area = 0;

	if(vertices_num % 2 !=0 )
        vertices.push_back(vertices[0]);

    for(int i = 0; i < vertices_num; i += 2)
        area += vertices[i+1].x*(vertices[i+2].y-vertices[i].y) + vertices[i+1].y*(vertices[i].x-vertices[i+2].x);
 
	area = area/2;
	return area;
}

float find_dist(POINT pt1, POINT pt2) {
    return float (sqrt(pow((pt1.x - pt2.x),2) + pow(pt1.y-pt2.y,2))); 
}

bool check_obstruction(std::vector< std::vector<POINT> > obstacles, SEGMENT segment) {
    int res = true;
    int break_out = false;
    int n;
    SEGMENT obs_side;


    for(std::vector<POINT> &obs : obstacles){
        // check that the obstacle starts and ends with the same point
        
        if(obs[0].x != obs.back().x || obs[0].y != obs.back().y){
            obs.push_back(obs[0]);
        }
        n = obs.size()-1;
        for (int pt = 0; pt < n; pt++ ){
            obs_side.a = obs[pt];
            obs_side.b = obs[pt+1];
            if(segment_intersection(segment,obs_side,false).x != -1){
                res = false;
                break_out = true;
                break;
            }
        }
        if (break_out){
            break;
        }
    }
    // std::cout << "intersection result: " << res << "\n -----" << std::endl;

    return res;
}

std::vector<int> backtrace(std::vector<int> parent, int start, int end) {
	std::vector<int> path;
    path.push_back(end);

    while (path[path.size()-1] != start) {
        path.push_back(parent[path[path.size()-1]]);
    }
    std::reverse(path.begin(),path.end());

    return path;
}

//Breadth First Search on a graph with a given Source and Target
std::vector<int> bfs(std::vector< std::vector<int> > graph, int source, int target) {
	std::vector<int> path;
	std::vector<int> visited;
	std::vector<int> parent;
	std::vector<int> queue;
	int current;

	for(int node = 0; node < graph.size(); node++){
		visited.push_back(0);
		parent.push_back(-1);
	}
	queue.push_back(source);
	while(queue.size() > 0){
        // std::cout << "in queue now:" << std::endl;
        // for (int i = 0; i<queue.size();i++){
        //     std::cout  << queue[i] << " , ";
        // }
        // std::cout << "\n--------------" << std::endl;

		current = queue[0];
		queue.erase(queue.begin());
		if (current == target) {
			path = backtrace(parent, source, target);
			return path;
		}
        for (int neighbor : graph[current]){
            if(visited[neighbor] == 0){
                visited[neighbor] = 1;
                parent[neighbor] = current;
                queue.push_back(neighbor);
            }
        }
	}

	path.push_back(-1);	
	return path;
}

// function that returns the determinant of a 2x2 matrix
float determinant( POINT a , POINT b){
    return float ( (a.x * b.y) - (a.y * b.x) );
}

bool counter_clockwise(POINT A,POINT B,POINT C){
    return double ((C.y-A.y) * (B.x-A.x)) > double ((B.y-A.y) * (C.x-A.x));
}

// function to test if two lines intersect. returns true if they are
bool intersect(POINT A,POINT B,POINT C,POINT D,bool print = false){
    //Check if any three points are co-linear

    float t1 = double(A.x * (B.y - C.y)) + double(B.x * (C.y - A.y)) + double(C.x * (A.y - B.y));
    float t2 = double(A.x * (B.y - D.y)) + double(B.x * (D.y - A.y)) + double(D.x * (A.y - B.y));
    float t3 = double(A.x * (C.y - D.y)) + double(C.x * (D.y - A.y)) + double(D.x * (A.y - C.y));
    float t4 = double(B.x * (C.y - D.y)) + double(C.x * (D.y - B.y)) + double(D.x * (B.y - C.y));

    if (print){
        std::cout << "-t1: " <<  t1 << " -> " << (t1 == 0)  << std::endl;
        std::cout << "-t2: " <<  t2 << " -> " << (t2 == 0)  << std::endl;
        std::cout << "-t3: " <<  t3 << " -> " << (t3 == 0)  << std::endl;
        std::cout << "-t4: " <<  t4 << " -> " << (t4 == 0)  << std::endl;
        std::cout << "-t5: " << (counter_clockwise(A,C,D) != counter_clockwise(B,C,D) && counter_clockwise(A,B,C) != counter_clockwise(A,B,D)) << std::endl;
    }
    // if statements to check if any 3 points are co-linear
    // if( t1 == 0 || t2 == 0 || t3 == 0 || t4 ==0){
    //     return true;
    // }

    return counter_clockwise(A,C,D) != counter_clockwise(B,C,D) && counter_clockwise(A,B,C) != counter_clockwise(A,B,D);;
}

// calculates the point of intersection between two lines
// must be only triggered if the two lines are infact intersecting
POINT line_intersection(POINT A, POINT B, POINT C, POINT D) { 
    POINT inter_p = {-1,-1};

    POINT x_diff = {A.x - B.x , C.x - D.x};
    POINT y_diff = {A.y - B.y , C.y - D.y};
    float div = determinant(x_diff, y_diff);

    if (div == 0){
        return inter_p;
    }
    float h1 = determinant(A,B);
    float h2 = determinant(C,D);
    POINT d = {h1,h2};
    float x = determinant(d,x_diff) / div;
    float y = determinant(d, y_diff) / div;

    inter_p = {x,y};
    // std::cout << "intersection point: " << inter_p.x << inter_p.y << std::endl;
    return inter_p;
    }

// function to test if two segments of line are intersecting
// if they are not, it returns a point of -1. if they are, it calcuates the intersection point
POINT segment_intersection(SEGMENT sigment1, SEGMENT sigment2,bool print = false){
    POINT intersection_p = {-1,-1};
    POINT a = sigment1.a;
    POINT b = sigment1.b;
    POINT c = sigment2.a;
    POINT d = sigment2.b;
    int enlarge = 600;

    if (print){
        std::cout << "-- a: (" << a.x*enlarge << "," << a.y*enlarge << ") b: (" << b.x*enlarge << "," << b.y*enlarge << ")" << std::endl;
        std::cout << "-- c: (" << c.x*enlarge << "," << c.y*enlarge << ") d: (" << d.x*enlarge << "," << d.y*enlarge << ")" << std::endl;
        std::cout << "-- the intersect call func result: " << (intersect(a, b, c, d)) << std::endl;
    }
    
    if( intersect(a, b, c, d,print)){
        return line_intersection(a, b, c, d);
    }
    return intersection_p;  
}
