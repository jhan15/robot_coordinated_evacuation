#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <algorithm>
	
using namespace std;

struct POINT {
    int x;
    int y;
    int obs;
} point;

struct SEGMENT {
    POINT a;
    POINT b;
} segment;


//Function to determine the intersection of two segments; source https://flassari.is/2008/11/line-line-intersection-in-cplusplus/
POINT intersection(SEGMENT segment1, SEGMENT segment2) {   
    POINT intersection_point;

    float d = (segment1.a.x - segment1.b.x) * (segment2.a.y - segment2.b.y) - (segment1.a.y - segment1.b.y) * (segment2.a.x - segment2.b.x);
    
    // If d is zero, there is no intersection
    if (d == 0) {
		intersection_point.x = (-1);
    	intersection_point.y = (-1);
    }

    // Get the x and y
    float pre = (segment1.a.x*segment1.b.y - segment1.a.y*segment1.b.x); 
    float post = (segment2.a.x*segment2.b.y - segment2.a.y*segment2.b.x);
    float x = ( pre * (segment2.a.x - segment2.b.x) - (segment1.a.x - segment1.b.x) * post ) / d;
    float y = ( pre * (segment2.a.y - segment2.b.y) - (segment1.a.y - segment1.b.y) * post ) / d;
 
    // Check if the x and y coordinates are within both lines
    if ( x < min(segment1.a.x, segment1.b.x) || x > max(segment1.a.x, segment1.b.x) || x < min(segment2.a.x, segment2.b.x) || x > max(segment2.a.x, segment2.b.x) ) {
		intersection_point.x = (-1);
    	intersection_point.y = (-1);
    }
    else if ( y < min(segment1.a.y, segment1.b.y) || y > max(segment1.a.y, segment1.b.y) || y < min(segment2.a.y, segment2.b.y) || y > max(segment2.a.y, segment2.b.y) ) {
		intersection_point.x = (-1);
    	intersection_point.y = (-1);
    }
    else {
        intersection_point.x = (int(x+0.5));
        intersection_point.y = (int(y+0.5));
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

		centroid_point.x = (int(0.5 + sum_x/num));
		centroid_point.y = (int(0.5 + sum_y/num));
	}

    return centroid_point;
}

int polygon_area(std::vector<POINT> vertices, int vertices_num) {
	int area = 0;

	if(vertices_num % 2 !=0 )
        vertices.push_back(vertices[0]);

    for(int i = 0; i < vertices_num; i += 2)
        area += vertices[i+1].x*(vertices[i+2].y-vertices[i].y) + vertices[i+1].y*(vertices[i].x-vertices[i+2].x);
 
	area = area/2;
	return area;
}

int find_dist(POINT pt1, POINT pt2) {
    return int( sqrt(pow((pt1.x - pt2.x),2) + pow(pt1.y-pt2.y,2)) ); 
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


int main() {
    
    //Reading data from the file
    
    string input_line;
    string input_line_numbers;
    stringstream ss;
    string temp_str;
    int temp_int;

    std::vector<int> int_data;
    std::vector<int> lengths;

    int points_cnt = 0;
    int total_points = 0;
    int obj_cnt = 0;


    ifstream input_data("input_file.txt");

    while (getline (input_data, input_line)) {
	
		for (int i = 0; i < input_line.length(); i++) {
	    	if (std::isdigit(input_line[i]) || input_line[i] == ' ') {
            	input_line_numbers += input_line[i];
	    	}
		}

    	ss << input_line_numbers;
    	points_cnt = 0;

    	while(!ss.eof()) {
        	ss >> temp_str; //take words into temp_str one by one

        	if(stringstream(temp_str) >> temp_int) { //try to convert string to int
            	int_data.push_back(temp_int);
            	points_cnt ++;
        	}
        	temp_str = ""; //clear temp string
    	}

		ss.str(std::string());
		ss.seekg(0,ios::beg);
    	input_line_numbers = "";
    	lengths.push_back(points_cnt);
    	obj_cnt += 1;	
    	total_points += points_cnt;
    }

    input_data.close();
    

    //Separating the data between boundaries, obstacles, start and end point
    
    int data_ptr;
    POINT temp_point;

    std::vector<POINT> boundary;
    for(data_ptr = 0; data_ptr < 4; data_ptr++) {
    	temp_point.x = int_data[data_ptr*2];
    	temp_point.y = int_data[data_ptr*2 + 1];
    	boundary.push_back(temp_point);
    }


    POINT start_point;
    start_point.x = int_data[total_points - 4];
    start_point.y = int_data[total_points - 3];


    POINT end_point;
    end_point.x = int_data[total_points - 2];
    end_point.y = int_data[total_points - 1];


    std::vector< std::vector<POINT> > obstacles;
    std::vector<POINT> obstacle; 
    int points;

    for(int obj = 0; obj < obj_cnt-2; obj++) {
    	points = 0;
    	while(points < lengths[obj+1]) {
    		temp_point.x = int_data[data_ptr*2];
    		temp_point.y = int_data[data_ptr*2 + 1];
    		temp_point.obs = obj; 
    		obstacle.push_back(temp_point);
    		points += 2;
    		data_ptr += 1;
    	}
    	obstacles.push_back(obstacle);
    	obstacle.clear();
    }


    //sorting the vertices by their x value in increasing order
    std::vector<POINT> sorted_vertices;
    int vertices_num = total_points/2 - 6;

    //filling the vector with the required number of placeholder points 
    for(int curr_vertex = vertices_num - 1; curr_vertex >= 0; curr_vertex--) {
    	temp_point.x = -1;
    	temp_point.y = -1;
    	temp_point.obs = -1;
        sorted_vertices.push_back(temp_point);
    }
         
    for(int curr_vertex = vertices_num - 1; curr_vertex >= 0; curr_vertex--) {
		for(int obj = 0; obj < obj_cnt - 2; obj++) {
	    	for(int vertex = 0; vertex < obstacles[obj].size(); vertex++) {
				if(obstacles[obj][vertex].x > sorted_vertices[curr_vertex].x && //the x of the vertex is bigger than the current one in this position and
                  ((curr_vertex == vertices_num - 1) || // it is the last vertex or
                  (obstacles[obj][vertex].x < sorted_vertices[curr_vertex + 1].x) || //is smaller than the x of the next vertex or
		  		  ((obstacles[obj][vertex].x == sorted_vertices[curr_vertex + 1].x) &&  //is the same with the x of the next vertex and
		  		  ((sorted_vertices[curr_vertex + 1].y != obstacles[obj][vertex].y) || //its y is different
		  		  (sorted_vertices[curr_vertex + 1].obs != obstacles[obj][vertex].obs))))) {//or it belongs to another obstacle 
		  		   	sorted_vertices[curr_vertex].x = obstacles[obj][vertex].x;
		    		sorted_vertices[curr_vertex].y = obstacles[obj][vertex].y;
		    		sorted_vertices[curr_vertex].obs = obj;
	        	}
	    	}        	    
        } 
    }


    //Determining the vertical lines
    int y_limit_lower = min(min(boundary[0].y, boundary[1].y), min(boundary[2].y, boundary[3].y));
    int y_limit_upper = max(max(boundary[0].y, boundary[1].y), max(boundary[2].y, boundary[3].y));

    std::vector< SEGMENT > open_line_segments;
    SEGMENT curr_segment;

	//copy the first point of an obstacle as its last points
	for(int obs = 0; obs < obj_cnt - 2; obs++) {
	    obstacles[obs].push_back(obstacles[obs][0]);
	}

    for(int pt = 0; pt < vertices_num; pt++){
		int up = 0;
		int down = 0;
		int break_now = 0;
		POINT lower_obs_pt;
		POINT upper_obs_pt;
		POINT intersection_point;
	    SEGMENT temp_segment;


		temp_point.x = sorted_vertices[pt].x;
		temp_point.y = y_limit_lower;
		temp_point.obs = sorted_vertices[pt].obs;
		curr_segment.a = temp_point;
		lower_obs_pt = temp_point;
		
		temp_point.y = y_limit_upper;
		curr_segment.b = temp_point;
		upper_obs_pt = temp_point;
	

	    for(int obs = 0; obs < obj_cnt - 2; obs++) { 
	    	for(int vertex = 0; vertex < obstacles[obs].size()-1; vertex++) {
	    		temp_segment.a = obstacles[obs][vertex];
	    		temp_segment.b = obstacles[obs][vertex + 1];	    		
				intersection_point = intersection(curr_segment, temp_segment);
				if(intersection_point.x != -1) {
		    		if(obs == sorted_vertices[pt].obs) {
						if((intersection_point.x != sorted_vertices[pt].x) || (intersection_point.y != sorted_vertices[pt].y)) {
			    			if(intersection_point.y > sorted_vertices[pt].y) {up = 1;}
			    			if(intersection_point.y < sorted_vertices[pt].y) {down = 1;}
						}
		    		}
		    		else {
						if((intersection_point.x != sorted_vertices[pt].x) || (intersection_point.y != sorted_vertices[pt].y)) {
			    			if((up == 0) && (intersection_point.y > sorted_vertices[pt].y) && (intersection_point.y < upper_obs_pt.y)) {
								upper_obs_pt = intersection_point;
			    			}
			    			if((down == 0) && (intersection_point.y < sorted_vertices[pt].y) && (intersection_point.y > lower_obs_pt.y)) {
								lower_obs_pt = intersection_point;
			    			}
						}
		    		}
				}
				if(up && down) {break_now = 1;}
            }
	    	if(break_now) {break;}
		}

		temp_point.x = -1;
		temp_point.y = -1;
		temp_segment.a = temp_point;
		temp_segment.b = temp_point;

		if(up && down){
			//temp_segment default values of -1 remain unchanged
		}
		else if(down){
			temp_segment.b = upper_obs_pt;
		}
		else if(up){
			temp_segment.a = lower_obs_pt;	
		}
		else{
			temp_segment.b = upper_obs_pt;
			temp_segment.a = lower_obs_pt;	
		} 
		open_line_segments.push_back(temp_segment);
    }
    

    //Finding cells
    POINT curr_vertex;
    SEGMENT next_segment;
    POINT next_vertex;
    std::vector<SEGMENT> lines_to_check;
    std::vector<int> group;
    std::vector< std::vector<POINT> > trapezoids;
    std::vector<POINT> temp_points1;
    std::vector<POINT> temp_points2;
    SEGMENT temp_segment;
    std::vector< std::vector<POINT> > cells;
    int break_now;
    int done[3];
    int double_i;
    int double_j;

    for(int i = 0; i < open_line_segments.size(); i++) {
    	curr_segment = open_line_segments[i];
        curr_vertex = sorted_vertices[i];
		break_now = 0;
		done[0] = 0;
		done[1] = 0;
		done[2] = 1;
		if(curr_segment.a.x == -1) {done[0] = 1;}
		if(curr_segment.b.x == -1) {done[1] = 1;}
		if((curr_segment.a.x == -1) && (curr_segment.b.x == -1)) {done[2] = 0;}
	
		for(int j = i+1; j < open_line_segments.size(); j++) {
			lines_to_check.clear();
			group.clear();
			trapezoids.clear();	

	    	next_segment = open_line_segments[j];
            next_vertex = sorted_vertices[j];
	    
	    	if(done[0] == 0) {
				if((next_segment.a.x != -1) && (next_segment.b.x != -1)) {
					temp_points1.push_back(curr_segment.a);
					temp_points1.push_back(curr_vertex);
					temp_points2.push_back(next_segment.a);
					temp_points2.push_back(next_vertex);					
					temp_segment.a = centroid(temp_points1); 
					temp_segment.b = centroid(temp_points2); 
					lines_to_check.push_back(temp_segment);
					group.push_back(0);

					//.a remains the same
					temp_points2.clear();
					temp_points2.push_back(next_segment.b);
					temp_points2.push_back(next_vertex);					
					temp_segment.b = centroid(temp_points2); 
					lines_to_check.push_back(temp_segment);
					group.push_back(0);

					temp_points1.clear();
					temp_points1.push_back(curr_segment.a);
					temp_points1.push_back(next_segment.a);
					temp_points1.push_back(next_vertex);
					temp_points1.push_back(curr_vertex);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points1.push_back(curr_segment.a);
					temp_points1.push_back(next_vertex);
					temp_points1.push_back(next_segment.b);
					temp_points1.push_back(curr_vertex);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points2.clear();
				}
				else if(next_segment.a.x != -1) {
					temp_points1.push_back(curr_segment.a);
					temp_points1.push_back(curr_vertex);
					temp_points2.push_back(next_segment.a);
					temp_points2.push_back(next_vertex);					
					temp_segment.a = centroid(temp_points1); 
					temp_segment.b = centroid(temp_points2); 
					lines_to_check.push_back(temp_segment);
					group.push_back(0);

					temp_points1.clear();
					temp_points1.push_back(curr_segment.a);
					temp_points1.push_back(next_segment.a);
					temp_points1.push_back(next_vertex);
					temp_points1.push_back(curr_vertex);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points2.clear();
				}
				else if(next_segment.b.x != -1) {
					temp_points1.push_back(curr_segment.a);
					temp_points1.push_back(curr_vertex);
					temp_points2.push_back(next_segment.b);
					temp_points2.push_back(next_vertex);					
					temp_segment.a = centroid(temp_points1); 
					temp_segment.b = centroid(temp_points2); 
					lines_to_check.push_back(temp_segment);
					group.push_back(0);

					temp_points1.clear();
					temp_points1.push_back(curr_segment.a);
					temp_points1.push_back(next_vertex);
					temp_points1.push_back(next_segment.b);
					temp_points1.push_back(curr_vertex);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points2.clear();
				}
				else {
					temp_points1.push_back(curr_segment.a);
					temp_points1.push_back(curr_vertex);
					temp_segment.a = centroid(temp_points1); 
					temp_segment.b = next_vertex;
					lines_to_check.push_back(temp_segment);
					group.push_back(0);

					temp_points1.clear();
					temp_points1.push_back(curr_segment.a);
					temp_points1.push_back(next_vertex);
					temp_points1.push_back(curr_vertex);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points2.clear();
				}
	    	}

	    	if(done[1] == 0) {
				if((next_segment.a.x != -1) && (next_segment.b.x != -1)) {
					temp_points1.push_back(curr_segment.b);
					temp_points1.push_back(curr_vertex);
					temp_points2.push_back(next_segment.a);
					temp_points2.push_back(next_vertex);					
					temp_segment.a = centroid(temp_points1); 
					temp_segment.b = centroid(temp_points2); 
					lines_to_check.push_back(temp_segment);
					group.push_back(1);

					//.a remains the same
					temp_points2.clear();
					temp_points2.push_back(next_segment.b);
					temp_points2.push_back(next_vertex);					
					temp_segment.b = centroid(temp_points2); 
					lines_to_check.push_back(temp_segment);
					group.push_back(1);

					temp_points1.clear();
					temp_points1.push_back(curr_vertex);
					temp_points1.push_back(next_segment.a);
					temp_points1.push_back(next_vertex);
					temp_points1.push_back(curr_segment.b);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points1.push_back(curr_vertex);
					temp_points1.push_back(next_vertex);
					temp_points1.push_back(next_segment.b);
					temp_points1.push_back(curr_segment.b);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points2.clear();
				}
				else if(next_segment.a.x != -1) {
					temp_points1.push_back(curr_segment.b);
					temp_points1.push_back(curr_vertex);
					temp_points2.push_back(next_segment.a);
					temp_points2.push_back(next_vertex);					
					temp_segment.a = centroid(temp_points1); 
					temp_segment.b = centroid(temp_points2); 
					lines_to_check.push_back(temp_segment);
					group.push_back(1);

					temp_points1.clear();
					temp_points1.push_back(curr_vertex);
					temp_points1.push_back(next_segment.a);
					temp_points1.push_back(next_vertex);
					temp_points1.push_back(curr_segment.b);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points2.clear();
				}
				else if(next_segment.b.x != -1) {
					temp_points1.push_back(curr_segment.b);
					temp_points1.push_back(curr_vertex);
					temp_points2.push_back(next_segment.b);
					temp_points2.push_back(next_vertex);					
					temp_segment.a = centroid(temp_points1); 
					temp_segment.b = centroid(temp_points2); 
					lines_to_check.push_back(temp_segment);
					group.push_back(1);

					temp_points1.clear();
					temp_points1.push_back(curr_vertex);
					temp_points1.push_back(next_vertex);
					temp_points1.push_back(next_segment.b);
					temp_points1.push_back(curr_segment.b);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points2.clear();
				}
				else {
					temp_points1.push_back(curr_segment.b);
					temp_points1.push_back(curr_vertex);
					temp_segment.a = centroid(temp_points1); 
					temp_segment.b = next_vertex;
					lines_to_check.push_back(temp_segment);
					group.push_back(1);

					temp_points1.clear();
					temp_points1.push_back(curr_vertex);
					temp_points1.push_back(next_vertex);
					temp_points1.push_back(curr_segment.b);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points2.clear();
				}
	    	}

	    	if(done[2] == 0) {
				if((next_segment.a.x != -1) && (next_segment.b.x != -1)) {
					temp_points2.push_back(next_segment.a);
					temp_points2.push_back(next_vertex);					
					temp_segment.a = curr_vertex; 
					temp_segment.b = centroid(temp_points2); 
					lines_to_check.push_back(temp_segment);
					group.push_back(2);

					temp_points2.clear();
					temp_points2.push_back(next_segment.b);
					temp_points2.push_back(next_vertex);					
					temp_segment.b = centroid(temp_points2); 
					lines_to_check.push_back(temp_segment);
					group.push_back(2);

					temp_points1.clear();
					temp_points1.push_back(curr_vertex);
					temp_points1.push_back(next_segment.a);
					temp_points1.push_back(next_vertex);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points1.push_back(curr_vertex);
					temp_points1.push_back(next_vertex);
					temp_points1.push_back(next_segment.b);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points2.clear();
				}
				else if(next_segment.a.x != -1) {
					temp_points2.push_back(next_segment.a);
					temp_points2.push_back(next_vertex);					
					temp_segment.a = curr_vertex; 
					temp_segment.b = centroid(temp_points2); 
					lines_to_check.push_back(temp_segment);
					group.push_back(2);

					temp_points1.clear();
					temp_points1.push_back(curr_vertex);
					temp_points1.push_back(next_segment.a);
					temp_points1.push_back(next_vertex);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points2.clear();
				}
				else if(next_segment.b.x != -1) {
					temp_points2.push_back(next_segment.b);
					temp_points2.push_back(next_vertex);					
					temp_segment.a = curr_vertex; 
					temp_segment.b = centroid(temp_points2); 
					lines_to_check.push_back(temp_segment);
					group.push_back(2);

					temp_points1.clear();
					temp_points1.push_back(curr_vertex);
					temp_points1.push_back(next_vertex);
					temp_points1.push_back(next_segment.b);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points2.clear();
				}
				else {
					temp_segment.a = curr_vertex; 
					temp_segment.b = next_vertex;
					lines_to_check.push_back(temp_segment);
					group.push_back(2);

					temp_points1.clear();
					temp_points1.push_back(curr_vertex);
					temp_points1.push_back(next_vertex);
					trapezoids.push_back(temp_points1);

					temp_points1.clear();
					temp_points2.clear();
				}
	    	}

    		std::vector<int> temp_to_remove;
			for(int line = 0; line < lines_to_check.size(); line++) {			//for index5,q in enumerate(lines_to_check): 
				int no_intersection[3] = {1, 1, 1}; 								//ok = [True, True, True];
				for(int obs = 0; obs < obj_cnt-2; obs++) {							//for index3,obs in enumerate(new_obstacles): //obs.append( obs[0] ); <-already done
					for(int vertex = 0; vertex < obstacles[obs].size()-1; vertex++) {	//for index4 in range(len(obs)-1):
						temp_segment.a = obstacles[obs][vertex];
						temp_segment.b = obstacles[obs][vertex+1];
						temp_point = intersection(lines_to_check[line], temp_segment);
						if(temp_point.x != -1) {											//if (segment_intersection( q[0], q[1],  obs[index4],  obs[index4+1]) != -1):
							no_intersection[group[line]] = 0;									//ok[q[2]] = False;
							int found = 0;
							for(int idx = 0; idx < temp_to_remove.size(); idx++) {
								if(line == temp_to_remove[idx]) {
									found = 1;
								}
							}
							if(found == 0) {												//if(index5 not in temp_to_remove):
								temp_to_remove.push_back(line);									//temp_to_remove.append(index5);
							}	
						}
					}			 
				}

				if(no_intersection[group[line]] == 1) {done[group[line]] = 1;}      //if (  ok[q[2]] is True ):   //done[q[2]] = True;
			}

			for(int line = 0; line < lines_to_check.size(); line++) {
				int found = 0;
				for(int idx = 0; idx < temp_to_remove.size(); idx++) {
					if(line == temp_to_remove[idx]) {
						found = 1;
					}
				}
				if(found == 0) {
					cells.push_back(trapezoids[line]);
				}	
			}

	    	if(done[0] && done[1] && done[2]) break;
		}
    }

    //Merge overlapping polygons
    std::vector< std::vector<POINT> > quad_cells; 
    std::vector< std::vector<POINT> > tri_cells;
    std::vector< std::vector<POINT> > other_cells;

    for(int cell = 0; cell < cells.size(); cell++) {
    	if(cells[cell].size() > 3) {quad_cells.push_back(cells[cell]);}
    	else if(cells[cell].size() == 3) {tri_cells.push_back(cells[cell]);}
    	else {other_cells.push_back(cells[cell]);}
    }

    std::vector<int> quads_to_remove;
    std::vector< std::vector<POINT> > quads_to_add;
	std::vector<POINT> temp1;
	std::vector<POINT> temp2;
	std::vector<POINT> new_quad;
    int area1, area2, area3;

    for(int cell1 = 0; cell1 < quad_cells.size(); cell1++) {
		for(int cell2 = 0; cell2 < quad_cells.size(); cell2++) {
 			if(cell1 != cell2) {
 				if(quad_cells[cell1][0].x == quad_cells[cell2][0].x and quad_cells[cell1][1].x == quad_cells[cell2][1].x) {
 					
 					temp1 = quad_cells[cell1];
 					temp1.push_back(quad_cells[cell1][0]);
 					temp2 = quad_cells[cell2];
 					temp2.push_back(quad_cells[cell2][0]);
 					area1 = polygon_area(temp1, 4);
 					area2 = polygon_area(temp2, 4);

 					temp_point.x = temp1[0].x;
 					temp_point.y = min(temp1[0].y, temp2[0].y);
					new_quad.push_back(temp_point);
 					temp_point.x = temp1[1].x;
 					temp_point.y = min(temp1[1].y, temp2[1].y);
					new_quad.push_back(temp_point);
 					temp_point.x = temp1[1].x;
 					temp_point.y = max(temp1[2].y, temp2[2].y);
					new_quad.push_back(temp_point);
 					temp_point.x = temp1[0].x;
 					temp_point.y = max(temp1[3].y, temp2[3].y);
					new_quad.push_back(temp_point);
 					temp_point.x = temp1[0].x;
 					temp_point.y = min(temp1[0].y, temp2[0].y);
					new_quad.push_back(temp_point);
					area3 = polygon_area(new_quad, 4);

					if(area1 + area2 >= area3) {
						quads_to_remove.push_back(cell1);
						quads_to_remove.push_back(cell2);
						quads_to_add.push_back(new_quad);
					}
						
 					temp1.clear();
 					temp2.clear();
 					new_quad.clear();
 				}
 			}
    	} 
    }
    
    sort(quads_to_remove.begin(), quads_to_remove.end());
	quads_to_remove.erase(unique(quads_to_remove.begin(), quads_to_remove.end()), quads_to_remove.end());
 
    for(int quad = 0; quad < quads_to_remove.size(); quad ++) {
    	quad_cells.erase(quad_cells.begin() + quads_to_remove[quad] - quad); //-quad because after deletion the indices shift
    }  

    for(int quad = 0; quad < quads_to_add.size(); quad ++) {
    	quad_cells.push_back(quads_to_add[quad]);
    }     

    quads_to_remove.clear();
    for(int quad1 = 0; quad1 < quad_cells.size(); quad1 ++) {
    	for(int quad2 = quad1 + 1; quad2 < quad_cells.size(); quad2 ++) {
    		int duplicate = 1;
    		for(int point = 0; point < quad_cells[quad1].size(); point ++) {
    			if((quad_cells[quad1][point].x != quad_cells[quad2][point].x) || (quad_cells[quad1][point].y != quad_cells[quad2][point].y)) {
    				duplicate = 0;
    				break;
    			}
    		}
    		if (duplicate) {quads_to_remove.push_back(quad2);}
    	}
    } 

    sort(quads_to_remove.begin(), quads_to_remove.end());
	quads_to_remove.erase(unique(quads_to_remove.begin(), quads_to_remove.end()), quads_to_remove.end());
 
    for(int quad = 0; quad < quads_to_remove.size(); quad ++) {
    	quad_cells.erase(quad_cells.begin() + quads_to_remove[quad] - quad); //-quad because after deletion the indices shift
    } 

    //One more pass to remove extra quads generated because of cross - segments

    quads_to_remove.clear();
    for(int quad1 = 0; quad1 < quad_cells.size(); quad1 ++) {
    	for(int quad2 = 0; quad2 < quad_cells.size(); quad2 ++) {
			if(quad1 != quad2 && quad_cells[quad1][0].x == quad_cells[quad2][0].x && quad_cells[quad1][1].x == quad_cells[quad2][1].x) { 
				if((quad_cells[quad1][0].y <= quad_cells[quad2][0].y) && (quad_cells[quad1][1].y <= quad_cells[quad2][1].y)
					&& (quad_cells[quad1][2].y >= quad_cells[quad2][2].y) && (quad_cells[quad1][3].y >= quad_cells[quad2][3].y)) {			
					quads_to_remove.push_back(quad2);
				}
			}	
    	}
    } 

    sort(quads_to_remove.begin(), quads_to_remove.end());
	quads_to_remove.erase(unique(quads_to_remove.begin(), quads_to_remove.end()), quads_to_remove.end());
 
    for(int quad = 0; quad < quads_to_remove.size(); quad ++) {
    	quad_cells.erase(quad_cells.begin() + quads_to_remove[quad] - quad); //-quad because after deletion the indices shift
    } 

    //Add boundary lines
	if(boundary[0].x != sorted_vertices[0].x) {
		new_quad.clear();

		new_quad.push_back(boundary[0]);

		temp_point.x = sorted_vertices[0].x;
		temp_point.y = y_limit_lower;
		new_quad.push_back(temp_point);		

		temp_point.x = sorted_vertices[0].x;
		temp_point.y = y_limit_upper;
		new_quad.push_back(temp_point);	

		new_quad.push_back(boundary[3]);

		quad_cells.push_back(new_quad);
	}

	if(boundary[1].x != sorted_vertices[sorted_vertices.size()-1].x) {
		new_quad.clear();

		temp_point.x = sorted_vertices[sorted_vertices.size()-1].x;
		temp_point.y = y_limit_lower;
		new_quad.push_back(temp_point);	

		new_quad.push_back(boundary[1]);

		new_quad.push_back(boundary[2]);

		temp_point.x = sorted_vertices[sorted_vertices.size()-1].x;
		temp_point.y = y_limit_upper;
		new_quad.push_back(temp_point);

		quad_cells.push_back(new_quad);
	}
    
    //Get the graph
    std::vector<int> same_boundary;
    std::vector<POINT> graph_vertices;
    std::vector<POINT> graph_edges;
    POINT centroid_vertex;
    POINT curr_centroid_vertex;
    POINT temp_edge_middle;
    int inside;
    int place; 
    int place1;
    int place2;
    int use; 
    int n;

    for(int cell1 = 0; cell1 < quad_cells.size(); cell1 ++) {
    	same_boundary.clear();
    	for(int cell2 = 0; cell2 < quad_cells.size(); cell2 ++) { 
			if(cell1 != cell2) {
				if((quad_cells[cell1][1].x == quad_cells[cell2][0].x) && 
				   ((quad_cells[cell1][2].y == quad_cells[cell2][0].y || quad_cells[cell1][2].y == quad_cells[cell2][3].y) ||
				    (quad_cells[cell1][1].y == quad_cells[cell2][0].y || quad_cells[cell1][1].y == quad_cells[cell2][3].y))) {
					same_boundary.push_back(cell2);
				}
			}
		}

		temp_points1.clear();
		for(int pt = 0; pt < 4; pt++) {temp_points1.push_back(quad_cells[cell1][pt]);}
		centroid_vertex = centroid(temp_points1);
		inside = 0;
		for(int vertex = 0; vertex < graph_vertices.size(); vertex++) {
			if(centroid_vertex.x == graph_vertices[vertex].x && centroid_vertex.y == graph_vertices[vertex].y) { 
				inside = 1;
				place = vertex;  
			}
		}
		if(inside == 0) {
			graph_vertices.push_back(centroid_vertex); 
			place = -1;
		}

		if(same_boundary.size() == 1) {
			temp_points1.clear();
			temp_points1.push_back(quad_cells[cell1][1]);
			temp_points1.push_back(quad_cells[cell1][2]);
			temp_edge_middle = centroid(temp_points1);
			graph_vertices.push_back(temp_edge_middle);
			n = graph_vertices.size() - 1;
			
			if(place != -1) {
				temp_point.x = place;
				temp_point.y = n;
				graph_edges.push_back(temp_point);
			}
			else {
				temp_point.x = n-1;
				temp_point.y = n;
				graph_edges.push_back(temp_point);
			}

			temp_points1.clear();
			for(int pt = 0; pt < 4; pt++) {temp_points1.push_back(quad_cells[same_boundary[0]][pt]);}
			curr_centroid_vertex = centroid(temp_points1);
			inside = 0;
			for(int vertex = 0; vertex < graph_vertices.size(); vertex++) {
				if(curr_centroid_vertex.x == graph_vertices[vertex].x && curr_centroid_vertex.y == graph_vertices[vertex].y) { 
					inside = 1;
					place2 = vertex;  
				}
			}
			if(inside == 0) { 
				place2 = -1;
			}
			if(place2 == -1) {
				graph_vertices.push_back(curr_centroid_vertex);
				temp_point.x = n;
				temp_point.y = n + 1;
				graph_edges.push_back(temp_point);
			}
			else {
				temp_point.x = n;
				temp_point.y = place2;
				graph_edges.push_back(temp_point);
			}
		}

		else if(same_boundary.size() > 1) {
			n = graph_vertices.size() - 1;
			if(place != -1) { use = place; }
			else { use = n; }
			for(int i = 0; i < same_boundary.size(); i ++){
				temp_points1.clear();
				for(int pt = 0; pt < 4; pt++) {temp_points1.push_back(quad_cells[same_boundary[i]][pt]);}
				curr_centroid_vertex = centroid(temp_points1);
				temp_points1.clear();
				temp_points1.push_back(quad_cells[same_boundary[i]][0]);
				temp_points1.push_back(quad_cells[same_boundary[i]][3]);
				temp_edge_middle = centroid(temp_points1);
				graph_vertices.push_back(temp_edge_middle);
				place1 = graph_vertices.size() - 1;
				inside = 0;
				for(int vertex = 0; vertex < graph_vertices.size(); vertex++) {
					if(curr_centroid_vertex.x == graph_vertices[vertex].x && curr_centroid_vertex.y == graph_vertices[vertex].y) { 
						inside = 1;
						place2 = vertex;
					}
				}
				if(inside == 0) {
					graph_vertices.push_back(curr_centroid_vertex);
					place2 = graph_vertices.size() - 1;
				}
				temp_point.x = use;
				temp_point.y = place1;
				graph_edges.push_back(temp_point);
				temp_point.x = place1;
				temp_point.y = place2;
				graph_edges.push_back(temp_point);
			}	
		}
	}


	//Source
	int min_ind = -1; 
	int min = 9999999;
	int dist;
	int m;

	for(int vertex = 0; vertex < graph_vertices.size(); vertex ++) {
		temp_segment.a = start_point;
		temp_segment.b = graph_vertices[vertex];
		if(check_obstruction(obstacles, temp_segment)) {
			dist = find_dist(graph_vertices[vertex], start_point);
			if(dist < min) {
				min = dist;
				min_ind = vertex;
			}	
		}
	}

	graph_vertices.push_back(start_point);
	m = graph_vertices.size()-1;
	temp_point.x = min_ind;
	temp_point.y = m;
	graph_edges.push_back(temp_point);

	min_ind = -1; 
	min = 9999999;

	for(int vertex = 0; vertex < graph_vertices.size(); vertex ++) {
		temp_segment.a = end_point;
		temp_segment.b = graph_vertices[vertex];
		if(check_obstruction(obstacles, temp_segment)) {
			dist = find_dist(graph_vertices[vertex], end_point);
			if(dist < min) {
				min = dist;
				min_ind = vertex;
			}	
		}
	}

	graph_vertices.push_back(end_point);
	m = graph_vertices.size()-1;
	temp_point.x = min_ind;
	temp_point.y = m;
	graph_edges.push_back(temp_point);


	std::vector< std::vector<int> > graph;
	std::vector<int> edges;
	
	for(int vertex = 0; vertex < graph_vertices.size(); vertex ++) {
		edges.clear();
		for(int edge = 0; edge < graph_edges.size(); edge ++) {
			if(graph_edges[edge].x == vertex) {
				edges.push_back(graph_edges[edge].y);
			}
			else if(graph_edges[edge].y == vertex){
				edges.push_back(graph_edges[edge].x);
			}
		}
		graph.push_back(edges); 
	}

	cout <<"GRAPH VERTICES: "<< endl;	
	for(int i = 0; i < graph_vertices.size(); i++){
		cout << "(" << graph_vertices[i].x << "," << graph_vertices[i].y;
		if(i == graph_vertices.size() - 1) { cout << ") "; }
		else cout << "), ";
	}
	cout << endl;
	cout << endl;

	cout <<"GRAPH: "<< endl;
	for(int j = 0; j < graph.size(); j++){
		cout << "(";
		for(int k = 0; k < graph[j].size(); k++) {	
			cout << graph[j][k];
			if(k != (graph[j].size() - 1)) {cout << ", ";}
		}
		if(j == (graph.size() - 1)) {cout << ") "; }
		else cout << "), ";
	}
	cout << endl;

    return 0;    
}
