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

    // if ((segment1.a.x == segment2.a.x && segment1.a.y == segment2.a.y) ||
    // (segment1.b.x == segment2.b.x && segment1.b.y == segment2.b.y) ||
    // (segment1.a.x == segment2.b.x && segment1.a.y == segment2.b.y) ||
    // (segment1.b.x == segment2.a.x && segment1.b.y == segment2.a.y)){
    //     std::cout << "hi" << std::endl;
	// 	intersection_point.x = (-1);
    // 	intersection_point.y = (-1);
    //     return intersection_point;
    // }

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

float determinant( POINT a , POINT b){
    return float ( (a.x * b.y) - (a.y * b.x) );
}

bool counter_clockwise(POINT A,POINT B,POINT C){
    // std::cout << "counter_clock wise output: " << ((C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)) << std::endl;
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
}

bool intersect(POINT A,POINT B,POINT C,POINT D){
    //Check if any three points are co-linear

    
    if( ( (A.x * (B.y - C.y) ) + (B.x * (C.y - A.y) ) + (C.x * (A.y - B.y) ) )== 0 ){
        return true;
    }
    if( ( (A.x * (B.y - D.y) ) + (B.x * (D.y - A.y) ) + (D.x * (A.y - B.y) ) )== 0 ){
        return true;
    }
    if( ( (A.x * (C.y - D.y) ) + (C.x * (D.y - A.y) ) + (D.x * (A.y - C.y) ) )== 0 ){
        return true;
    }
    if( ( (B.x * (C.y - D.y) ) + (C.x * (D.y - B.y) ) + (D.x * (B.y - C.y) ) )== 0 ){
        return true;
    }   
    
    return counter_clockwise(A,C,D) != counter_clockwise(B,C,D) && counter_clockwise(A,B,C) != counter_clockwise(A,B,D);
}
POINT line_intersection(POINT A, POINT B, POINT C, POINT D) { 
    POINT inter_p;
    POINT x_diff = {A.x - B.x , C.x - D.x};
    POINT y_diff = {A.y - B.y , C.y - D.y};
    float div = determinant(x_diff, y_diff);

    if (div == 0){
        inter_p = {-1,-1};
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
POINT segment_intersection(SEGMENT sigment1, SEGMENT sigment2, POINT cur_pt){
    POINT intersection_p = {-1,-1};
    POINT a = sigment1.a;
    POINT b = sigment1.b;
    POINT c = sigment2.a;
    POINT d = sigment2.b;
    // check if the current vertex of the obsticale is equal to any of the sigment points
    // this is to stop counting the current vertex as an intersection
    if((a.x == cur_pt.x && a.y == cur_pt.y) ||
     (b.x == cur_pt.x && b.y == cur_pt.y) || 
     (c.x == cur_pt.x && c.y == cur_pt.y) ||
     (d.x == cur_pt.x && d.y == cur_pt.y)){
        return intersection_p;
     }

    if( intersect(a, b, c, d) == true){
        return line_intersection(a, b, c, d);
    }
    return intersection_p;  

}




//Finding the centroid of a list of vertices
POINT centroid(std::vector<POINT> vertices) {   

    POINT centroid_point;
	int num = vertices.size();
    float sum_x = 0;
    float sum_y = 0;

    if(num == 0) {
		centroid_point.x = -1;
		centroid_point.y = -1;
    }

    else {
		for(int vert_idx = 0; vert_idx < num; vert_idx++) {
            sum_x += vertices[vert_idx].x;
	    	sum_y += vertices[vert_idx].y;
		}

		//centroid_point.x = (int(0.5 + sum_x/num));
		//centroid_point.y = (int(0.5 + sum_y/num));

		centroid_point.x = sum_x/num;
		centroid_point.y = sum_y/num;
	}

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
    return sqrt(pow((pt1.x - pt2.x),2) + pow(pt1.y-pt2.y,2)); 
}

int check_obstruction(std::vector< std::vector<POINT> > obstacles, SEGMENT segment) {
    int res = 1;
    int break_out = 0;
    int n;
    SEGMENT obs_side;
    for(int obs = 0; obs < obstacles.size(); obs++) {
        n = obstacles[obs].size()-1;
        if((obstacles[obs][n].x != obstacles[obs][0].x) || (obstacles[obs][n].y != obstacles[obs][0].y)) {
            obstacles[obs].push_back(obstacles[obs][0]);
        }
        for(int pt = 0; pt < (obstacles[obs].size()-1); pt++) {
        	obs_side.a = obstacles[obs][pt];
        	obs_side.b = obstacles[obs][pt+1];
            if ((intersection(segment, obs_side)).x != -1) {            	
                res = 0;
                break_out = 1;
                break;
            }  
        }
        if(break_out) {
            break;
        }
    }
    return res;
}

std::vector<int> backtrace(std::vector<int> parent, int start, int end) {
	std::vector<int> path;
	std::vector<int> temp_path;
    temp_path.push_back(end);

    while (temp_path[temp_path.size()-1] != start) {
        temp_path.push_back(parent[temp_path[temp_path.size()-1]]);
    }

    for(int i = temp_path.size() - 1; i >= 0; i--){
    	path.push_back(temp_path[i]);
    }

    return path;
}

//Breadth First Search on a graph with a given Source and Target
std::vector<int> bfs(std::vector< std::vector<int> > graph, int source, int target) {
	std::vector<int> path;
	std::vector<int> visited;
	std::vector<int> parent;
	std::vector<int> queue;
	int current;

	int size = graph.size();
	for(int node = 0; node < size; node++){
		visited.push_back(0);
		parent.push_back(-1);
	}

	queue.push_back(source);
	while(queue.size() > 0){
		current = queue[0];
		queue.erase(queue.begin());
		if (current == target) {
			path = backtrace(parent, source, target);
			return path;
		}
		for (int neighbor = 0; neighbor < graph[current].size(); neighbor++){
            if (visited[graph[current][neighbor]] == 0) {
                visited[graph[current][neighbor]] = 1;
                parent[graph[current][neighbor]] = current;
                queue.push_back(graph[current][neighbor]);
            }    
        }
	}

	path.push_back(-1);	
	return path;
}
