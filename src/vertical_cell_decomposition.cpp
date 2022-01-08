#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <algorithm>
#include "../vertical_cell_decomposition.hpp"
	
using namespace std;

//Function to determine the intersection of two segments; source https://flassari.is/2008/11/line-line-intersection-in-cplusplus/
POINT intersection(SEGMENT segment1, SEGMENT segment2) {   
    POINT intersection_point;

    float d = (segment1.a.x - segment1.b.x) * (segment2.a.y - segment2.b.y) - (segment1.a.y - segment1.b.y) * (segment2.a.x - segment2.b.x);

    if (d == 0) {
		intersection_point.x = (-1);
    	intersection_point.y = (-1);
    }

    // Get the x and y
    float pre = (segment1.a.x*segment1.b.y - segment1.a.y*segment1.b.x); 
    float post = (segment2.a.x*segment2.b.y - segment2.a.y*segment2.b.x);
    float x = ( pre * (segment2.a.x - segment2.b.x) - (segment1.a.x - segment1.b.x) * post ) / d;
    float y = ( pre * (segment2.a.y - segment2.b.y) - (segment1.a.y - segment1.b.y) * post ) / d;

    if ( int(1000000*x) < int(1000000*min(segment1.a.x, segment1.b.x)) || int(1000000*x) > int(1000000*max(segment1.a.x, segment1.b.x)) || int(1000000*x) < int(1000000*min(segment2.a.x, segment2.b.x)) || int(1000000*x) > int(1000000*max(segment2.a.x, segment2.b.x)) ) {
		intersection_point.x = (-1);
    	intersection_point.y = (-1);
    }
    else if ( int(1000000*y) < int(1000000*min(segment1.a.y, segment1.b.y)) || int(1000000*y) > int(1000000*max(segment1.a.y, segment1.b.y)) || int(1000000*y) < int(1000000*min(segment2.a.y, segment2.b.y)) || int(1000000*y) > int(1000000*max(segment2.a.y, segment2.b.y)) ) {
		intersection_point.x = (-1);
    	intersection_point.y = (-1);
    }
    else {
        intersection_point.x = x; //(int(x+0.5));
        intersection_point.y = y; //(int(y+0.5));
    }

    return intersection_point;
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
