#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <iterator>
#include <string> 

#include <cmath>
#include "motion_planning.hpp"
#include "dubins.h"
#include "inflate_objects.hpp"
#include "vertical_cell_decomposition.hpp"

//a function to ensure robots are not taking in the same time segments which cross
std::vector<std::vector<robotPos>> coordinate_motion(std::vector<std::vector<robotPos>> initial_paths) {
    //vector to store the updated paths
    std::vector<std::vector<robotPos>> final_paths = {{},{},{}};
    //vector to keep track of the path lengths
    std::vector<int> path_lengths;

    //initial path lengths
    path_lengths.push_back(initial_paths[0].size());
    path_lengths.push_back(initial_paths[1].size());
    path_lengths.push_back(initial_paths[2].size());

    //length of the longest path to control the number of iterations through the checker loop
    int max_length;

    //adding the initial points to the final paths
    final_paths[0].push_back(initial_paths[0][0]);
    final_paths[1].push_back(initial_paths[1][0]);
    final_paths[2].push_back(initial_paths[2][0]);
    
    //temp segments and points used to check for intersections
    SEGMENT temp_segment_0;
    SEGMENT temp_segment_1;
    SEGMENT temp_segment_2;    
    POINT temp_point_1;
    POINT temp_point_2;
    //store information if there are intersections between any combination of two points
    std::vector<POINT> intersection_point = {{}, {}, {}};
    //a flag for checking intersection which will be 1 only if segments are present for both paths (so different lengths don't lead to errors)
    std::vector<int> check_intersection = {1, 1, 1};
    //for traching the number of points added for each path
    std::vector<int> added = {0, 0, 0};

    //maximal path length to indicate where to stop checking
    max_length = max(path_lengths[0], max(path_lengths[1], path_lengths[2]));

    //always add the final points separately - the algorithm will consider them a collision because they are always the same (the gate)
    for(int point = 0; point < max_length - 2; point ++){
        
        //prepare segments to check their intersection
        if(point < path_lengths[0] - 2) {
            temp_point_1.x = initial_paths[0][point-added[0]].x; 
            temp_point_1.y = initial_paths[0][point-added[0]].y;
            temp_point_2.x = initial_paths[0][point+1-added[0]].x; 
            temp_point_2.y = initial_paths[0][point+1-added[0]].y;
            temp_segment_0.a = temp_point_1;
            temp_segment_0.b = temp_point_2;
        }
        else {
            //if the entire path has been checked - don't check
            check_intersection[0] = 0;
        }  
        //repeat for paths 1 and 2
        if(point < path_lengths[1] - 2) {
            temp_point_1.x = initial_paths[1][point-added[1]].x; 
            temp_point_1.y = initial_paths[1][point-added[1]].y;
            temp_point_2.x = initial_paths[1][point+1-added[1]].x; 
            temp_point_2.y = initial_paths[1][point+1-added[1]].y;
            temp_segment_1.a = temp_point_1;
            temp_segment_1.b = temp_point_2;
        }
        else {
            check_intersection[1] = 0;
        }
        if(point < path_lengths[2] - 2) {
            temp_point_1.x = initial_paths[2][point-added[2]].x; 
            temp_point_1.y = initial_paths[2][point-added[2]].y;
            temp_point_2.x = initial_paths[2][point+1-added[2]].x; 
            temp_point_2.y = initial_paths[2][point+1-added[2]].y;
            temp_segment_2.a = temp_point_1;
            temp_segment_2.b = temp_point_2;
        }
        else {
            check_intersection[2] = 0;
        }

        //check for intersection between each segment combination
        if(check_intersection[0] && check_intersection[1]) intersection_point[0] = intersection_trial(temp_segment_0, temp_segment_1); 
        if(check_intersection[1] && check_intersection[2]) intersection_point[1] = intersection_trial(temp_segment_1, temp_segment_2); 
        if(check_intersection[2] && check_intersection[0]) intersection_point[2] = intersection_trial(temp_segment_2, temp_segment_0);

        //in case of no intersection for path 0 - add next point of the path
        if((check_intersection[0] && (intersection_point[0].x == -1) && (intersection_point[2].x == -1)) ||
           (check_intersection[0] && (intersection_point[0].x == -1) && !check_intersection[2]) ||
           (check_intersection[0] && (intersection_point[2].x == -1) && !check_intersection[1]) ||
           (check_intersection[0] && !check_intersection[1] && !check_intersection[2])){
            final_paths[0].push_back(initial_paths[0][point+1-added[0]]);
        }
        //in case of no intersection for path 1 - add next point of the path
        if((check_intersection[1] && (intersection_point[0].x == -1) && (intersection_point[1].x == -1)) ||
           (check_intersection[1] && (intersection_point[0].x == -1) && !check_intersection[2]) ||
           (check_intersection[1] && (intersection_point[1].x == -1) && !check_intersection[0]) ||
           (check_intersection[1] && !check_intersection[0] && !check_intersection[2])){
            final_paths[1].push_back(initial_paths[1][point+1-added[1]]);
        }
        //in case of no intersection for path 2 - add next point of the path
        if((check_intersection[2] && (intersection_point[1].x == -1) && (intersection_point[2].x == -1)) ||
           (check_intersection[2] && (intersection_point[1].x == -1) && !check_intersection[0]) ||
           (check_intersection[2] && (intersection_point[2].x == -1) && !check_intersection[1]) ||
           (check_intersection[2] && !check_intersection[0] && !check_intersection[1])){
            final_paths[2].push_back(initial_paths[2][point+1-added[2]]);
        }   

        //in case of intersection between all the three paths
        if(check_intersection[0] && check_intersection[1] && check_intersection[2] &&
            intersection_point[0].x != -1 && intersection_point[0].x != -1 && intersection_point[0].x != -1){
            //if path 0 is the longest add next point to it and keep 1 and 2 at the same point for this move
            if(path_lengths[0] == max_length) {
                final_paths[0].push_back(initial_paths[0][point+1-added[0]]);
                final_paths[1].push_back(initial_paths[1][point-added[1]]);
                path_lengths[1] += 1;
                added[1] += 1;                
                final_paths[2].push_back(initial_paths[2][point-added[2]]);
                path_lengths[2] += 1;
                added[2] += 1;
            }
            //if path 1 is the longest add next point to it and keep 0 and 2 at the same point for this move
            else if(path_lengths[1] == max_length) {
                final_paths[1].push_back(initial_paths[1][point+1-added[1]]);
                final_paths[0].push_back(initial_paths[0][point-added[0]]);
                path_lengths[0] += 1;
                added[0] += 1;                
                final_paths[2].push_back(initial_paths[2][point-added[2]]);
                path_lengths[2] += 1;
                added[2] += 1;
            }
            //if path 2 is the longest or all paths are equally long add next point to path 2 and keep 0 and 1 at the same point for this move            
            else {
                final_paths[2].push_back(initial_paths[2][point+1-added[2]]);
                final_paths[0].push_back(initial_paths[0][point-added[0]]);
                path_lengths[0] += 1;
                added[0] += 1;                
                final_paths[1].push_back(initial_paths[1][point-added[1]]);
                path_lengths[1] += 1;
                added[1] += 1;
            }    
        }

        else {
            //if there is intersection between paths 0 and 1 add a new point to the longer and halt the shorter
            if(check_intersection[0] && check_intersection[1] && intersection_point[0].x != -1){
                if(path_lengths[0] > path_lengths[1]) {
                    final_paths[0].push_back(initial_paths[0][point+1-added[0]]);
                    final_paths[1].push_back(initial_paths[1][point-added[1]]);
                    path_lengths[1] += 1;
                    added[1] += 1;
                }
                else {
                    final_paths[1].push_back(initial_paths[1][point+1-added[1]]);
                    final_paths[0].push_back(initial_paths[0][point-added[0]]);
                    path_lengths[0] += 1;
                    added[0] += 1;
                }
            }
            //if there is intersection between paths 1 and 2 add a new point to the longer and halt the shorter
            if(check_intersection[1] && check_intersection[2] && intersection_point[1].x != -1){
                if(path_lengths[1] > path_lengths[2]) {
                    final_paths[1].push_back(initial_paths[1][point+1-added[1]]);
                    final_paths[2].push_back(initial_paths[2][point-added[2]]);
                    path_lengths[2] += 1;
                    added[2] += 1;
                }
                else {
                    final_paths[2].push_back(initial_paths[2][point+1-added[2]]);
                    final_paths[1].push_back(initial_paths[1][point-added[1]]);
                    path_lengths[1] += 1;
                    added[1] += 1;
                }
            }
            //if there is intersection between paths 0 and 2 add a new point to the longer and halt the shorter
            if(check_intersection[0] && check_intersection[2] && intersection_point[2].x != -1){
                if(path_lengths[2] > path_lengths[0]) {
                    final_paths[2].push_back(initial_paths[2][point+1-added[2]]);
                    final_paths[0].push_back(initial_paths[0][point-added[0]]);
                    path_lengths[0] += 1;
                    added[0] += 1;
                }
                else {
                    final_paths[0].push_back(initial_paths[0][point+1-added[0]]);
                    final_paths[2].push_back(initial_paths[2][point-added[2]]);
                    path_lengths[2] += 1;
                    added[2] += 1;                
                }
            }
        }
        //reset the flags for need to be checked 
        check_intersection = {1, 1, 1};
        //update the max length after the additions
        max_length = max(path_lengths[0], max(path_lengths[1], path_lengths[2]));
    }

    //if all three robots are about to finish at the same time delay 0 with 2 steps and 1 with 1 step to avoid collisions    
    if ((path_lengths[0] == path_lengths[1]) && (path_lengths[1] == path_lengths[2])){
        final_paths[0].push_back(initial_paths[0][initial_paths[0].size()-2]);
        final_paths[0].push_back(initial_paths[0][initial_paths[0].size()-2]);
        final_paths[1].push_back(initial_paths[1][initial_paths[1].size()-2]);
    }
    //if 0 is about to finish with any other robot delay it with a step
    else if ((path_lengths[0] == path_lengths[1]) || (path_lengths[0] == path_lengths[2])){
        final_paths[0].push_back(initial_paths[0][initial_paths[0].size()-2]);
    }
    //if 1 is about to finish together with 2 delay it with a step
    else if (path_lengths[1] == path_lengths[2]){
        final_paths[1].push_back(initial_paths[1][initial_paths[1].size()-2]);
    }

    //add the gate point manually (otherwise it will be detected as intersection)
    final_paths[0].push_back(initial_paths[0][initial_paths[0].size()-1]);
    final_paths[1].push_back(initial_paths[1][initial_paths[1].size()-1]);
    final_paths[2].push_back(initial_paths[2][initial_paths[2].size()-1]);

    //print the final paths
    for(int path_num = 0; path_num < final_paths.size(); path_num ++) {
        cout << "Final path for robot " << path_num << ": ";
        for(int point = 0; point < final_paths[path_num].size(); point ++) {
            cout << "{" << final_paths[path_num][point].x << ", " << final_paths[path_num][point].y << "}, ";
        }
        cout << endl;
    }

    return final_paths;
}
