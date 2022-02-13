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

    cout << "Path lengths: " << path_lengths[0] << " " << path_lengths[1] << " " << path_lengths[2] << endl;

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
    std::vector<POINT> intersection_point = {{}, {}, {}};
    std::vector<int> check_intersection = {1, 1, 1};
    std::vector<int> added = {0, 0, 0};

    max_length = max(path_lengths[0], max(path_lengths[1], path_lengths[2]));

    for(int point = 0; point < max_length - 2; point ++){
        
        if(point < path_lengths[0] - 2) {
            temp_point_1.x = initial_paths[0][point-added[0]].x; 
            temp_point_1.y = initial_paths[0][point-added[0]].y;
            temp_point_2.x = initial_paths[0][point+1-added[0]].x; 
            temp_point_2.y = initial_paths[0][point+1-added[0]].y;
            temp_segment_0.a = temp_point_1;
            temp_segment_0.b = temp_point_2;
        }
        else {
            check_intersection[0] = 0;
        }  
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

        if(check_intersection[0] && check_intersection[1]) intersection_point[0] = intersection_trial(temp_segment_0, temp_segment_1); 
        if(check_intersection[1] && check_intersection[2]) intersection_point[1] = intersection_trial(temp_segment_1, temp_segment_2); 
        if(check_intersection[2] && check_intersection[0]) intersection_point[2] = intersection_trial(temp_segment_2, temp_segment_0);

        if((check_intersection[0] && (intersection_point[0].x == -1) && (intersection_point[2].x == -1)) ||
           (check_intersection[0] && (intersection_point[0].x == -1) && !check_intersection[2]) ||
           (check_intersection[0] && (intersection_point[2].x == -1) && !check_intersection[1]) ||
           (check_intersection[0] && !check_intersection[1] && !check_intersection[2])){
            final_paths[0].push_back(initial_paths[0][point+1-added[0]]);
            cout << "Pushing from 1" << endl;
        }
        if((check_intersection[1] && (intersection_point[0].x == -1) && (intersection_point[1].x == -1)) ||
           (check_intersection[1] && (intersection_point[0].x == -1) && !check_intersection[2]) ||
           (check_intersection[1] && (intersection_point[1].x == -1) && !check_intersection[0]) ||
           (check_intersection[1] && !check_intersection[0] && !check_intersection[2])){
            final_paths[1].push_back(initial_paths[1][point+1-added[1]]);
            cout << "Pushing from 2" << endl;
        }
        if((check_intersection[2] && (intersection_point[1].x == -1) && (intersection_point[2].x == -1)) ||
           (check_intersection[2] && (intersection_point[1].x == -1) && !check_intersection[0]) ||
           (check_intersection[2] && (intersection_point[2].x == -1) && !check_intersection[1]) ||
           (check_intersection[2] && !check_intersection[0] && !check_intersection[1])){
            final_paths[2].push_back(initial_paths[2][point+1-added[2]]);
            cout << "Pushing from 3" << endl;
        }   

        cout << "Check intersections " << check_intersection[0] << " " << check_intersection[1] << " " << check_intersection[2] << endl; 
        cout << "Intersection points {" << intersection_point[0].x << ", " << intersection_point[0].y << "}, {" << intersection_point[1].x << ", " << intersection_point[1].y << "}, {"<< intersection_point[2].x << ", " << intersection_point[2].y << "}"<< endl; 

        if(check_intersection[0] && check_intersection[1] && check_intersection[2] &&
            intersection_point[0].x != -1 && intersection_point[0].x != -1 && intersection_point[0].x != -1){
            if(path_lengths[0] == max_length) {
                final_paths[0].push_back(initial_paths[0][point+1-added[0]]);
                final_paths[1].push_back(initial_paths[1][point-added[1]]);
                path_lengths[1] += 1;
                added[1] += 1;                
                final_paths[2].push_back(initial_paths[2][point-added[2]]);
                path_lengths[2] += 1;
                added[2] += 1;
            }
            else if(path_lengths[1] == max_length) {
                final_paths[1].push_back(initial_paths[1][point+1-added[1]]);
                final_paths[0].push_back(initial_paths[0][point-added[0]]);
                path_lengths[0] += 1;
                added[0] += 1;                
                final_paths[2].push_back(initial_paths[2][point-added[2]]);
                path_lengths[2] += 1;
                added[2] += 1;
            }
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
            if(check_intersection[0] && check_intersection[1] && intersection_point[0].x != -1){
                if(path_lengths[0] > path_lengths[1]) {
                    final_paths[0].push_back(initial_paths[0][point+1-added[0]]);
                    final_paths[1].push_back(initial_paths[1][point-added[1]]);
                    path_lengths[1] += 1;
                    added[1] += 1;
                    cout << "Pushing from 4" << endl;
                }
                else {
                    final_paths[1].push_back(initial_paths[1][point+1-added[1]]);
                    final_paths[0].push_back(initial_paths[0][point-added[0]]);
                    path_lengths[0] += 1;
                    added[0] += 1;
                    cout << "Pushing from 5" << endl;              
                }
            }
            if(check_intersection[1] && check_intersection[2] && intersection_point[1].x != -1){
                if(path_lengths[1] > path_lengths[2]) {
                    final_paths[1].push_back(initial_paths[1][point+1-added[1]]);
                    final_paths[2].push_back(initial_paths[2][point-added[2]]);
                    path_lengths[2] += 1;
                    added[2] += 1;
                    cout << "Pushing from 6" << endl;            
                }
                else {
                    final_paths[2].push_back(initial_paths[2][point+1-added[2]]);
                    final_paths[1].push_back(initial_paths[1][point-added[1]]);
                    path_lengths[1] += 1;
                    added[1] += 1;
                    cout << "Pushing from 7" << endl;
                }
            }
            if(check_intersection[0] && check_intersection[2] && intersection_point[2].x != -1){
                if(path_lengths[2] > path_lengths[0]) {
                    final_paths[2].push_back(initial_paths[2][point+1-added[2]]);
                    final_paths[0].push_back(initial_paths[0][point-added[0]]);
                    path_lengths[0] += 1;
                    added[0] += 1;
                    cout << "Pushing from 8" << endl;
                }
                else {
                    final_paths[0].push_back(initial_paths[0][point+1-added[0]]);
                    final_paths[2].push_back(initial_paths[2][point-added[2]]);
                    path_lengths[2] += 1;
                    added[2] += 1;                
                    cout << "Pushing from 9" << endl;
                }
            }
        }
        check_intersection = {1, 1, 1};
        max_length = max(path_lengths[0], max(path_lengths[1], path_lengths[2]));
    }
    
    if ((path_lengths[0] == path_lengths[1]) && (path_lengths[1] == path_lengths[2])){
        cout << "Pushing from 10" << endl;
        final_paths[0].push_back(initial_paths[0][initial_paths[0].size()-2]);
        final_paths[0].push_back(initial_paths[0][initial_paths[0].size()-2]);
        final_paths[1].push_back(initial_paths[1][initial_paths[1].size()-2]);
    }
    else if ((path_lengths[0] == path_lengths[1]) || (path_lengths[0] == path_lengths[2])){
        cout << "Pushing from 11" << endl;        
        final_paths[0].push_back(initial_paths[0][initial_paths[0].size()-2]);
    }
    else if (path_lengths[1] == path_lengths[2]){
        cout << "Pushing from 12" << endl;
        final_paths[1].push_back(initial_paths[1][initial_paths[1].size()-2]);
    }

    cout << "Pushing from 13" << endl;
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