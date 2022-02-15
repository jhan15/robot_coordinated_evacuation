#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <algorithm>
#include "dubins.h"
#include "vertical_cell_decomposition.hpp"
	
using namespace std;


std::vector<POINT> sort_vertices(std::vector< std::vector<POINT> > obstacles, std::vector<POINT> sorted_vertices, int vertices_num) {

    int add_to_list;

    for(int curr_vertex = vertices_num - 1; curr_vertex >= 0; curr_vertex--) {
      sorted_vertices.push_back(POINT{-1,-1,-1,-1});
    }

    for(int curr_vertex = vertices_num - 1; curr_vertex >= 0; curr_vertex--) {
      for(int obj = 0; obj < obstacles.size(); obj++) {
        for(int vertex = 0; vertex < obstacles[obj].size(); vertex++) {
          add_to_list = 0;
          if(obstacles[obj][vertex].x > sorted_vertices[curr_vertex].x) {
            if( curr_vertex == vertices_num - 1 ||
            obstacles[obj][vertex].x < sorted_vertices[curr_vertex + 1].x ||
            obstacles[obj][vertex].x == sorted_vertices[curr_vertex + 1].x)
            {add_to_list = 1;}

            for(int vert = 0; vert < sorted_vertices.size(); vert ++) {
              if (sorted_vertices[vert].x == obstacles[obj][vertex].x && sorted_vertices[vert].y == obstacles[obj][vertex].y) {
                add_to_list = 0;
              }
            }
          }
          if(add_to_list == 1) {
            sorted_vertices[curr_vertex].x = obstacles[obj][vertex].x;
            sorted_vertices[curr_vertex].y = obstacles[obj][vertex].y;
            sorted_vertices[curr_vertex].obs = obj;
          }
        }             
      } 
    }
    
    return sorted_vertices;
}


std::vector< std::vector<POINT> > close_polygons (std::vector< std::vector<POINT> > polygons) {
    for(int polygon = 0; polygon < polygons.size(); polygon++) {
      //check if it has already been added
      if(polygons[polygon][0].x != polygons[polygon].back().x || polygons[polygon][0].x != polygons[polygon].back().x ){
        polygons[polygon].push_back(polygons[polygon][0]);
      }
    }
    return polygons;
}


POINT intersection_trial(SEGMENT sigment1, SEGMENT sigment2){
    POINT intersection_pt;
    boost::geometry::model::linestring<point_xy> line1, line2;
    typedef boost::geometry::model::segment<point_xy> Segment;
    Segment AB( point_xy(sigment1.a.x,sigment1.a.y), point_xy(sigment1.b.x,sigment1.b.y) );
    Segment CD( point_xy(sigment2.a.x,sigment2.a.y), point_xy(sigment2.b.x,sigment2.b.y) );

    std::vector<point_xy> result;
    boost::geometry::intersection(AB, CD,result);

    if (result.size()>0){
        intersection_pt = {float(boost::geometry::get<0>(result[0])),float(boost::geometry::get<1>(result[0]))};
    }
    else{
        intersection_pt = {-1,-1};
    }
    return intersection_pt;
}


std::vector< SEGMENT > find_lines(std::vector<POINT> sorted_vertices, std::vector< std::vector<POINT> > obstacles, float y_limit_lower, float y_limit_upper){
    
    SEGMENT curr_segment;
    SEGMENT temp_segment;
    POINT temp_point;
    std::vector< SEGMENT > open_line_segments;

    float prev_x = -1;

    for(const POINT& pt : sorted_vertices) {
        prev_x = pt.x;
        int up = 0;
        int down = 0;
        int break_now = 0;
        POINT lower_obs_pt;
        POINT upper_obs_pt;
        POINT intersection_point;
      
        temp_point.x = pt.x;
        temp_point.y = y_limit_lower;
        temp_point.obs = pt.obs;
        curr_segment.a = temp_point;
        lower_obs_pt = temp_point;
    
        temp_point.y = y_limit_upper;
        curr_segment.b = temp_point;
        upper_obs_pt = temp_point;

        for(int obs = 0; obs < obstacles.size(); obs++) { 
            for(int vertex = 0; vertex < obstacles[obs].size()-1; vertex++) {
                temp_segment.a = obstacles[obs][vertex];
                temp_segment.b = obstacles[obs][vertex + 1];

                // FOR DEBUG
                // print the horizontal lines and the current obstacle segment being checked
                // int output7 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
                // int output8 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
                // int output9 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
                // auto color_rand = cv::Scalar(output7,output8,output9);
                // cv::line(plot, cv::Point2f(curr_segment.a.x*enlarge,curr_segment.a.y*enlarge), cv::Point2f(curr_segment.b.x*enlarge,curr_segment.b.y*enlarge), color_rand, 2);
                // cv::line(plot, cv::Point2f(temp_segment.a.x*enlarge,temp_segment.a.y*enlarge), cv::Point2f(temp_segment.b.x*enlarge,temp_segment.b.y*enlarge), color_rand, 2);
                // cv::imshow("Clipper", plot);
                // cv::waitKey(0);
                // std::cout << "current point: " << "( " << pt.x << ","<< pt.y << ")" << std::endl;

                intersection_point = intersection_trial(curr_segment, temp_segment);

                if(intersection_point.x != -1){ 
                    //FOR DEBUG
                    // printing the intersection points of the vertical lines with the obstacles and showing whats left from them
                    // cv::Point2f point_center2(pt.x*enlarge,pt.y*enlarge);
                    // std::string text = std::to_string(pt.obs);
                    // putText(plot, text, point_center2, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));
                    // cv::Point2f point_center(intersection_point.x*enlarge,intersection_point.y*enlarge);
                    // cv::circle(plot, point_center, 5,cv::Scalar( 0, 10, 125 ),2);
                    // cv::imshow("Clipper", plot);
                    // cv::waitKey(0);
                    // std::cout << "temp_segment.a (" << temp_segment.a.x << " , " << temp_segment.a.y << " )" << endl;
                    // std::cout << "temp_segment.b (" << temp_segment.b.x << " , " << temp_segment.b.y << " )"<< endl;
                    // std::cout << "curr_segment.a (" << curr_segment.a.x << " , " << curr_segment.a.y << " )"<< endl;
                    // std::cout << "curr_segment.b (" << curr_segment.b.x << " , " << curr_segment.b.y << " )"<< endl;
                    // std::cout << "intersection_point (" << intersection_point.x << " , " << intersection_point.y << " )"<< endl;
                    if(obs == pt.obs) {
                        if((intersection_point.x != pt.x) || (intersection_point.y != pt.y)) {
                            if(intersection_point.y > pt.y) {up = 1;}
                            if(intersection_point.y < pt.y) {down = 1;}
                        }
                    }
                    else {
                        if((intersection_point.x != pt.x) || (intersection_point.y != pt.y)) {
                            if((up == 0) && (intersection_point.y > pt.y) && (intersection_point.y < upper_obs_pt.y)) {
                                upper_obs_pt = intersection_point;
                            }
                            if((down == 0) && (intersection_point.y < pt.y) && (intersection_point.y > lower_obs_pt.y)) {
                                lower_obs_pt = intersection_point;
                            }
                        }
                    }
                }

                if(up && down) {
                    break_now = 1;
                    break;
                }
            }
        
            if(break_now) {break;}
        }    
        
        // FOR DEBUG
        // printing the vertical lines
        //if(down == 0){
        //    cv::line(plot, cv::Point2f(lower_obs_pt.x*enlarge,lower_obs_pt.y*enlarge), cv::Point2f(pt.x*enlarge,pt.y*enlarge), cv::Scalar(0,255,0), 1);
        //}
        //if(up == 0){
        //    cv::line(plot, cv::Point2f(upper_obs_pt.x*enlarge,upper_obs_pt.y*enlarge), cv::Point2f(pt.x*enlarge,pt.y*enlarge), cv::Scalar(0,255,0), 1);
        //}
        // cv::imshow("Clipper", plot);
        // cv::waitKey(0);

        temp_point = {-1,-1};
        temp_segment = {temp_point,temp_point};

        if(up && down) {
            //temp_segment default values of -1 remain unchanged
        }
        else if(down) {
            temp_segment.b = upper_obs_pt;
        }
        else if(up) {
            temp_segment.a = lower_obs_pt;  
        }
        else {
            temp_segment.a = lower_obs_pt;
            temp_segment.b = upper_obs_pt;  
        } 
        open_line_segments.push_back(temp_segment);
    }
        // FOR DEBUG
        // print out the open line segments
        // std::cout << "open line segments:" << std::endl;
        // for(int i = 0; i < open_line_segments.size(); i++){
        //  std::cout << "seg # "<< i << " A: x=" << open_line_segments[i].a.x << " y=" << open_line_segments[i].a.y << " B: x=" << open_line_segments[i].b.x << " y=" << open_line_segments[i].b.y << std::endl; 
        // }

    return open_line_segments;
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


std::vector< std::vector<POINT> > find_cells(std::vector<SEGMENT> open_line_segments, std::vector<POINT> sorted_vertices, std::vector< std::vector<POINT> > obstacles){
    POINT temp_point;
    SEGMENT temp_segment;
    POINT curr_vertex;
    SEGMENT curr_segment;
    SEGMENT next_segment;
    SEGMENT next_next_segment;
    POINT next_vertex;
    POINT next_next_vertex;
    std::vector<SEGMENT> lines_to_check;
    std::vector<int> group;
    std::vector< std::vector<POINT> > trapezoids;
    std::vector<POINT> temp_points1;
    std::vector<POINT> temp_points2;
    std::vector< std::vector<POINT> > cells;
    int break_now;
    std::vector<int> done;
    bool extra_search = true; // to allow for adding extra trapazoids when next two segements have same x

    for(int i = 0; i < open_line_segments.size(); i++) {
      curr_segment = open_line_segments[i];
      curr_vertex = sorted_vertices[i];
      break_now = 0;
      done = {0,0,1};

      // a is lower limit , b is upper limit

      // group 0 -> not blocked from the bottom
      // group 1 -> not blocked from the top
      // group 2 -> completely blocked

      // if lower limit is blocked -> don't look down
      if(curr_segment.a.x == -1) {done[0] = 1;}
      // if upper limit is blocked -> don't look up :)
      if(curr_segment.b.x == -1) {done[1] = 1;}
      // if upper and lower limits are blocked -> figure out something else
      if((curr_segment.a.x == -1) && (curr_segment.b.x == -1)) {done[2] = 0;}
      int counter = 0;
      for(int j = i+1; j < open_line_segments.size(); j++) {
        counter +=1;
        // std::cout << "the done status for segment " << i << " is: " << done[0] << done[1] << done[2] << " iteration:" << counter << std::endl;
        lines_to_check.clear();
        group.clear();
        trapezoids.clear();
        bool double_check = false;
        bool next_two_seg_same_x = false;
        
        next_segment = open_line_segments[j];
        next_vertex = sorted_vertices[j];
        if( j != open_line_segments.size()-1 && extra_search){
          next_next_segment = open_line_segments[j+1];
          next_next_vertex = sorted_vertices[j+1];
          next_two_seg_same_x = (next_vertex.x == next_next_vertex.x);
        }
        // check to see if the next segemnt is completely free from both sides
        double_check = next_segment.a.x != -1 && next_segment.b.x != -1;
        // if not blocked from the bottom
        if(done[0] == 0) {
          // cout << "--------cur seg is not blocked from bottom----------" << endl;
          // std::cout << "test -> done[0]  == 0 [not blocked from the bottom" << std::endl;
          // and the next vertex is free from both sides
          if(double_check) {
            // cout << "--next seg is free" << endl;         
            temp_segment.a = centroid({curr_segment.a,curr_vertex}); 
            temp_segment.b = centroid({next_segment.a,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(0);

            // FOR DEBUG
            // print debug temp points
            // std::cout << "temp_points1: (" << temp_points1[0].x << " , " << temp_points1[0].y << ") , (" << temp_points1[1].x << " , " << temp_points1[1].y << ")" << std::endl;
            // int output4 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            // int output5 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            // int output6 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            // auto color_rand = cv::Scalar(output4,output5,output6);
            // cv::Point2f point_center(temp_segment.a.x*enlarge,temp_segment.a.y*enlarge);
            // cv::circle(plot, point_center, 5,color_rand,cv::FILLED,cv::LINE_8);
            // cv::Point2f point_center11(temp_segment.b.x*enlarge,temp_segment.b.y*enlarge);
            // cv::circle(plot, point_center11, 5,color_rand,cv::FILLED,cv::LINE_8);  
            // cv::line(plot, cv::Point2f(temp_points1[0].x*enlarge,temp_points1[0].y*enlarge), cv::Point2f(temp_points1[1].x*enlarge,temp_points1[1].y*enlarge), color_rand, 2);
            // cv::line(plot, cv::Point2f(temp_points2[0].x*enlarge,temp_points2[0].y*enlarge), cv::Point2f(temp_points2[1].x*enlarge,temp_points2[1].y*enlarge), color_rand, 2);
            // cv::imshow("Clipper", plot);
            // cv::waitKey(0); 
            
            //.a remains the same        
            temp_segment.b = centroid({next_segment.b,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(0);
            trapezoids.push_back({curr_segment.a,next_segment.a,next_vertex,curr_vertex});
            trapezoids.push_back({curr_segment.a,next_vertex,next_segment.b,curr_vertex});
          }
          // if next segment is not blocked from the bottom
          else if(next_segment.a.x != -1) {
            // cout << "--next seg is not blocked from bottom" << endl;       
            temp_segment.a = centroid({curr_segment.a,curr_vertex}); 
            temp_segment.b = centroid({next_segment.a,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(0);
            // trapezoids
            temp_points1.clear();
            trapezoids.push_back({curr_segment.a,next_segment.a,next_vertex,curr_vertex});
            if(extra_search){
              if(next_two_seg_same_x && (next_next_segment.b.x != -1) && (next_next_segment.a.x == -1)){
                // cout << "--next next seg is not blocked from top" << endl;
                temp_segment.a = centroid({curr_segment.a,curr_vertex}); 
                temp_segment.b = centroid({next_next_segment.b,next_next_vertex}); 
                lines_to_check.push_back(temp_segment);
                group.push_back(0);
                temp_points1.clear();
                trapezoids.push_back({curr_segment.a,next_next_vertex,next_next_segment.b,curr_vertex});              
              }
            }
          }
          //if the next segment is not blocked from the top
          else if(next_segment.b.x != -1) {
            // cout << "--next seg is not blocked from top" << endl;       
            temp_segment.a = centroid({curr_segment.a,curr_vertex}); 
            temp_segment.b = centroid({next_segment.b,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(0);
            trapezoids.push_back({curr_segment.a,next_vertex,next_segment.b,curr_vertex});
            if(extra_search){
              if(next_two_seg_same_x && (next_next_segment.a.x != -1) && (next_next_segment.b.x == -1)){
                // cout << "--next next seg is not blocked from bottom" << endl;
                temp_segment.a = centroid({curr_segment.a,curr_vertex}); 
                temp_segment.b = centroid({next_next_segment.a,next_next_vertex}); 
                lines_to_check.push_back(temp_segment);
                group.push_back(0);
                trapezoids.push_back({curr_segment.a,next_next_segment.a,next_next_vertex,curr_vertex});              
              }
            }
          }
          else {
            temp_segment.a = centroid({curr_segment.a,curr_vertex}); 
            temp_segment.b = next_vertex;
            lines_to_check.push_back(temp_segment);
            group.push_back(0);
            trapezoids.push_back({curr_segment.a,next_vertex,curr_vertex});
          }
        }
        // not blocked from the bottom
        if(done[1] == 0) {
          // cout << "--------cur seg is not blocked from top----------" << endl;
          // if next segment is free from both sides
          if(double_check) {
            // cout << "--next seg is free" << endl;        
            temp_segment.a = centroid({curr_segment.b,curr_vertex}); 
            temp_segment.b = centroid({next_segment.a,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(1);         
            temp_segment.b = centroid({next_segment.b,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(1);
            trapezoids.push_back({curr_vertex,next_segment.a,next_vertex,curr_segment.b});
            trapezoids.push_back({curr_vertex,next_vertex,next_segment.b,curr_segment.b});
          }
          // if next segement not blocked from the bottom
          else if(next_segment.a.x != -1) {
            // cout << "--next seg is not blocked from bottom" << endl;       
            temp_segment.a = centroid({curr_segment.b,curr_vertex}); 
            temp_segment.b = centroid({next_segment.a,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(1);
            trapezoids.push_back({curr_vertex,next_segment.a,next_vertex,curr_segment.b});
            if(extra_search){
              if(next_two_seg_same_x && (next_next_segment.b.x != -1)  && (next_next_segment.a.x == -1)){
                // cout << "--next next seg is not blocked from top" << endl;
                temp_segment.a = centroid({curr_segment.b,curr_vertex}); 
                temp_segment.b = centroid({next_next_segment.b,next_next_vertex});
                lines_to_check.push_back(temp_segment);
                group.push_back(1);
                trapezoids.push_back({curr_vertex,next_next_vertex,next_next_segment.b,curr_segment.b});              
              }
            }
          }
          // if next segment is not blocked from the top
          else if(next_segment.b.x != -1) {
            // cout << "--next seg is not blocked from top" << endl;         
            temp_segment.a = centroid({curr_segment.b,curr_vertex}); 
            temp_segment.b = centroid({next_segment.b,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(1);
            trapezoids.push_back({curr_vertex,next_vertex,next_segment.b,curr_segment.b});

            if(extra_search){
              if(next_two_seg_same_x && (next_next_segment.a.x != -1)  && (next_next_segment.b.x == -1)){
                // cout << "--next next seg is not blocked from bottom" << endl;
                temp_segment.a = centroid({curr_segment.b,curr_vertex}); 
                temp_segment.b = centroid({next_next_segment.a,next_next_vertex});
                lines_to_check.push_back(temp_segment);
                group.push_back(1);
                trapezoids.push_back({curr_vertex,next_next_segment.a,next_next_vertex,curr_segment.b});              
              }
            }
          }
          else {
            temp_segment.a = centroid({curr_segment.b,curr_vertex}); 
            temp_segment.b = next_vertex;
            lines_to_check.push_back(temp_segment);
            group.push_back(1);
            trapezoids.push_back({curr_vertex,next_vertex,curr_segment.b});
          }
        }

        //blocked from both
        if(done[2] == 0) {
          // if next segement free from both sides
          if(double_check) {       
            temp_segment.a = curr_vertex; 
            temp_segment.b = centroid({next_segment.a,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(2);         
            temp_segment.b = centroid({next_segment.b,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(2);
            trapezoids.push_back({curr_vertex,next_segment.a,next_vertex});
            trapezoids.push_back({curr_vertex,next_vertex,next_segment.b});
          }
          else if(next_segment.a.x != -1) {        
            temp_segment.a = curr_vertex; 
            temp_segment.b = centroid({next_segment.a,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(2);
            trapezoids.push_back({curr_vertex,next_segment.a,next_vertex});
          }
          else if(next_segment.b.x != -1) {       
            temp_segment.a = curr_vertex; 
            temp_segment.b = centroid({next_segment.b,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(2);

            temp_points1.clear();
            trapezoids.push_back({curr_vertex,next_vertex,next_segment.b});
          }
          else {
            temp_segment.a = curr_vertex; 
            temp_segment.b = next_vertex;
            lines_to_check.push_back({temp_segment});
            group.push_back(2);
            trapezoids.push_back({curr_vertex,next_vertex});
          }
        }
        // FOR DEBUG
        // print lines_to_check
        // for(unsigned j=0; j<lines_to_check.size(); j++){
        //   int output4 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
        //   int output5 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
        //   int output6 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
        //   auto color_rand = cv::Scalar(output4,output5,output6);
        //   cv::line(plot, cv::Point2f(lines_to_check[j].a.x*enlarge,lines_to_check[j].a.y*enlarge), cv::Point2f(lines_to_check[j].b.x*enlarge,lines_to_check[j].b.y*enlarge), color_rand, 1);           
        //   cv::imshow("Clipper", plot);
        //   cv::waitKey(0); 
        // } 
        // print for trapezoids
        // for (unsigned j=0; j<trapezoids.size(); j++) {
        //     // std::cout << "test test" << std::endl;
        //     cv::line(plot, cv::Point2f(trapezoids[j][0].x*enlarge,trapezoids[j][0].y*enlarge), cv::Point2f(trapezoids[j][1].x*enlarge,trapezoids[j][1].y*enlarge), cv::Scalar(60,110,23), 2);
        //     cv::line(plot, cv::Point2f(trapezoids[j][1].x*enlarge,trapezoids[j][1].y*enlarge), cv::Point2f(trapezoids[j][2].x*enlarge,trapezoids[j][2].y*enlarge), cv::Scalar(60,110,23), 2);
        //     cv::line(plot, cv::Point2f(trapezoids[j][2].x*enlarge,trapezoids[j][2].y*enlarge), cv::Point2f(trapezoids[j][0].x*enlarge,trapezoids[j][0].y*enlarge), cv::Scalar(60,110,23), 2);
        // }
        // cv::imshow("Clipper", plot);
        // cv::waitKey(0);

        std::vector<int> temp_to_remove;
        for(int line = 0; line < lines_to_check.size(); line++) {  
          // for debugging
          // reset_obs_plot(plot,obstacles);
          int no_intersection[3] = {1, 1, 1};           
          for(int obs = 0; obs < obstacles.size(); obs++) {              
            for(int vertex = 0; vertex < obstacles[obs].size()-1; vertex++) {
              temp_segment.a = obstacles[obs][vertex];
              temp_segment.b = obstacles[obs][vertex+1];          
              temp_point = intersection_trial(lines_to_check[line], temp_segment);

              // FOR DEBUG
              // print the intersection prosedure
              // std::cout << "-- (before recall) intersection_point: " << "(" << temp_point.x*enlarge << " , " << temp_point.y*enlarge << ")" << std::endl;
              // temp_point = segment_intersection(lines_to_check[line], temp_segment,true);
              // std::cout << "-- intersect data after retest --" << std::endl;
              // bool _check = intersect(temp_segment.a,temp_segment.b,lines_to_check[line].a,lines_to_check[line].b,true);
              // std::cout << "-- intersect double check: " << _check << std::endl;
              // std::cout << "-- intersection_point: " << "(" << temp_point.x*enlarge << " , " << temp_point.y*enlarge << ")" << std::endl;
              // std::cout << "-- obstacle segment-p1: " << "(" << temp_segment.a.x*enlarge << " , " << temp_segment.a.y*enlarge << ")" << std::endl;
              // std::cout << "-- obstacle segment-p2: " << "(" << temp_segment.b.x*enlarge << " , " << temp_segment.b.y*enlarge << ")" << std::endl;
              // std::cout << "-- line segment-p1: " << "(" << lines_to_check[line].a.x*enlarge << " , " << lines_to_check[line].a.y*enlarge << ")" << std::endl;
              // std::cout << "-- line segment-p2: " << "(" << lines_to_check[line].b.x*enlarge << " , " << lines_to_check[line].b.y*enlarge << ")" << std::endl;
              // std::cout<< "-------------------------" << std::endl;
              // int output4 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
              // int output5 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
              // int output6 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
              // auto color_rand = cv::Scalar(output4,output5,output6);
              // cv::Point2f point_center(temp_point.x*enlarge,temp_point.y*enlarge);
              // cv::circle(plot, point_center, 5,color_rand,2);//cv::Scalar( 0, 10, 125 ),2);
              // cv::line(plot, cv::Point2f(lines_to_check[line].a.x*enlarge,lines_to_check[line].a.y*enlarge), cv::Point2f(lines_to_check[line].b.x*enlarge,lines_to_check[line].b.y*enlarge), color_rand, 2);           
              // cv::line(plot, cv::Point2f(temp_segment.a.x*enlarge,temp_segment.a.y*enlarge), cv::Point2f(temp_segment.b.x*enlarge,temp_segment.b.y*enlarge), color_rand, 2);           
              // cv::imshow("Clipper", plot);
              // cv::waitKey(0);
              
              if(temp_point.x != -1) {
                no_intersection[group[line]] = 0;
                int found = 0;
                for(int idx = 0; idx < temp_to_remove.size(); idx++) {
                  if(line == temp_to_remove[idx]) {
                    found = 1;
                    break;
                  }
                }
                if(found == 0) {
                  temp_to_remove.push_back(line);
                } 
              }
            // std::cout << "-- intersection_point: " << "(" << temp_point.x << " , " << temp_point.y << ")" << std::endl;
            }      
          }
          
          // std::cout << "-- no_intersection stat: " << no_intersection[0] << " , "  << no_intersection[1] << " , " << no_intersection[2] << std::endl;
          if(no_intersection[group[line]] == 1) {done[group[line]] = 1;}
        }

        for(int line = 0; line < lines_to_check.size(); line++) {
          int found = 0;
          for(int idx = 0; idx < temp_to_remove.size(); idx++) {
            if(line == temp_to_remove[idx]) {
              found = 1;
              break;
            }
          }
          if(found == 0) {
            cells.push_back(trapezoids[line]);

            // FOR DEBUG
            // print the cells one by one
            // int output4 = 0;
            // int output5 = 0;
            // int output6 = 0;
            // output4 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            // output5 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            // output6 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            // auto color_rand = cv::Scalar(output4,output5,output6);
            // for(unsigned j=1; j<cells[cells.size()-1].size(); j++){
            //   cv::Point2f point_center(cells[cells.size()-1][j-1].x*enlarge,cells[cells.size()-1][j-1].y*enlarge);
            //   cv::circle(plot, point_center, 5,color_rand,cv::FILLED,cv::LINE_8); 
            //   cv::line(plot, cv::Point2f(cells[cells.size()-1][j-1].x*enlarge,cells[cells.size()-1][j-1].y*enlarge), cv::Point2f(cells[cells.size()-1][j].x*enlarge,cells[cells.size()-1][j].y*enlarge), color_rand, 2);
            //   if (j == cells[cells.size()-1].size() -1){
            //     cv::line(plot, cv::Point2f(cells[cells.size()-1][j].x*enlarge,cells[cells.size()-1][j].y*enlarge), cv::Point2f(cells[cells.size()-1][0].x*enlarge,cells[cells.size()-1][0].y*enlarge), color_rand, 2);
            //   }
            // }
            // cv::imshow("Clipper", plot);
            // cv::waitKey(0);  
          } 
        }

        if(done[0] && done[1] && done[2]){ break;}
      }
    }
    return cells;
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


std::vector< std::vector<POINT> > merge_polygons(std::vector<std::vector<POINT>> cells) {
    
    //I don't think we ever use tri and other cells so I don't see why we need this
    //to vring it back just uncomment and change cells to quad_cells starting from the end of the commented out section
    /*
    std::vector< std::vector<POINT> > quad_cells; 
    
    std::vector< std::vector<POINT> > tri_cells;
    std::vector< std::vector<POINT> > other_cells;

    for(int cell = 0; cell < cells.size(); cell++) {
      if(cells[cell].size() > 3) {quad_cells.push_back(cells[cell]);}
      else if(cells[cell].size() == 3) {tri_cells.push_back(cells[cell]);}
      else {other_cells.push_back(cells[cell]);}
    }
    */

    POINT temp_point;
    std::vector<int> quads_to_remove;
    std::vector< std::vector<POINT> > quads_to_add;
    std::vector<POINT> temp1;
    std::vector<POINT> temp2;
    std::vector<POINT> new_quad;
    int area1, area2, area3;

    for(int cell1 = 0; cell1 < cells.size(); cell1++) {
      for(int cell2 = 0; cell2 < cells.size(); cell2++) {
        if(cell1 != cell2) {
          if(cells[cell1][0].x == cells[cell2][0].x && cells[cell1][1].x == cells[cell2][1].x) {
          
            temp1 = cells[cell1];
            // add the first point to the back
            temp1.push_back(cells[cell1][0]);
            temp2 = cells[cell2];
            temp2.push_back(cells[cell2][0]);
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

    // for (int i = 0 ; i < quads_to_remove.size();i++){
    //   std::cout << "quads to remove: " << quads_to_remove[i] << std::endl;
    // }


    quads_to_remove.erase(unique(quads_to_remove.begin(), quads_to_remove.end()), quads_to_remove.end());

    for(int quad = 0; quad < quads_to_remove.size(); quad ++) {
      cells.erase(cells.begin() + quads_to_remove[quad] - quad); //-quad because after deletion the indices shift
    }

    for(int quad = 0; quad < quads_to_add.size(); quad ++) {
      cells.push_back(quads_to_add[quad]);
    }     

    quads_to_remove.clear();
    for(int quad1 = 0; quad1 < cells.size(); quad1 ++) {
      for(int quad2 = quad1 + 1; quad2 < cells.size(); quad2 ++) {
        int duplicate = 1;
        for(int point = 0; point < cells[quad1].size(); point ++) {
          if((cells[quad1][point].x != cells[quad2][point].x) || (cells[quad1][point].y != cells[quad2][point].y)) {
            duplicate = 0;
            break;
          }
        }
        if (duplicate) {quads_to_remove.push_back(quad2);}
      }
    } 

    sort(quads_to_remove.begin(), quads_to_remove.end());
    quads_to_remove.erase(unique(quads_to_remove.begin(), quads_to_remove.end()), quads_to_remove.end());
 
    // for (int i = 0 ; i < quads_to_remove.size();i++){
    //   std::cout << "quads to remove: " << quads_to_remove[i] << std::endl;
    // }

    for(int quad = 0; quad < quads_to_remove.size(); quad ++) {
      cells.erase(cells.begin() + quads_to_remove[quad] - quad); //-quad because after deletion the indices shift
    } 

    //One more pass to remove extra quads generated because of cross - segments

    quads_to_remove.clear();
    for(int quad1 = 0; quad1 < cells.size(); quad1 ++) {
      for(int quad2 = 0; quad2 < cells.size(); quad2 ++) {
        if(quad1 != quad2 && cells[quad1][0].x == cells[quad2][0].x && cells[quad1][1].x == cells[quad2][1].x) { 
          if((cells[quad1][0].y <= cells[quad2][0].y) && (cells[quad1][1].y <= cells[quad2][1].y)
              && (cells[quad1][2].y >= cells[quad2][2].y) && (cells[quad1][3].y >= cells[quad2][3].y)) {      
              quads_to_remove.push_back(quad2);
          }
        } 
      }
    } 

    sort(quads_to_remove.begin(), quads_to_remove.end());
    quads_to_remove.erase(unique(quads_to_remove.begin(), quads_to_remove.end()), quads_to_remove.end());
 
    for(int quad = 0; quad < quads_to_remove.size(); quad ++) {
      cells.erase(cells.begin() + quads_to_remove[quad] - quad); //-quad because after deletion the indices shift
    }

    return cells;
}


std::vector<std::vector<POINT>> boundary_cells(std::vector<POINT> boundary, std::vector<std::vector<POINT>> cells, std::vector<POINT> sorted_vertices, float y_limit_lower, float y_limit_upper) {
    
    POINT temp_point;
    std::vector<POINT> new_quad;

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

      cells.push_back(new_quad);
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

      cells.push_back(new_quad);
    }
    return cells;
}


tuple <std::vector<POINT>, std::vector<POINT>> get_graph(std::vector< std::vector<POINT> > cells) {
    std::vector<int> same_boundary;
    std::vector<POINT> graph_vertices;
    std::vector<POINT> graph_edges;
    POINT centroid_vertex;
    POINT curr_centroid_vertex;
    POINT temp_edge_middle;
    POINT temp_point;
    std::vector<POINT> temp_points1;
    std::vector<POINT> temp_points2;
    int inside;
    int place; 
    int place1;
    int place2;
    int use; 
    int n;

    // for each quad cell find the cells that have the same boundary --> find neigbour cells
    for(int cell1 = 0; cell1 < cells.size(); cell1 ++) {
      same_boundary.clear();
      //compare to the rest of the cells if it is not the same cell
      for(int cell2 = 0; cell2 < cells.size(); cell2 ++) { 
        if(cell1 != cell2) {
          if((cells[cell1][1].x == cells[cell2][0].x) && 
            ((cells[cell1][2].y == cells[cell2][0].y || cells[cell1][2].y == cells[cell2][3].y) ||
            (cells[cell1][1].y == cells[cell2][0].y || cells[cell1][1].y == cells[cell2][3].y))) {
            same_boundary.push_back(cell2);
          }
        }
      }

      temp_points1.clear();
      for(int pt = 0; pt < 4; pt++) {temp_points1.push_back(cells[cell1][pt]);}
      centroid_vertex = centroid(temp_points1);
      inside = 0;
      for(int vertex = 0; vertex < graph_vertices.size(); vertex++) {
        if(centroid_vertex.x == graph_vertices[vertex].x && centroid_vertex.y == graph_vertices[vertex].y) { 
          inside = 1;
          place = vertex;
          break;
        }
      }
      if(inside == 0) {
        graph_vertices.push_back(centroid_vertex); 
        place = -1;
      }

      if(same_boundary.size() == 1) {
        temp_points1.clear();
        temp_points1.push_back(cells[cell1][1]);
        temp_points1.push_back(cells[cell1][2]);
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
        for(int pt = 0; pt < 4; pt++) {temp_points1.push_back(cells[same_boundary[0]][pt]);}
        curr_centroid_vertex = centroid(temp_points1);
        inside = 0;
        for(int vertex = 0; vertex < graph_vertices.size(); vertex++) {
          if(curr_centroid_vertex.x == graph_vertices[vertex].x && curr_centroid_vertex.y == graph_vertices[vertex].y) { 
            inside = 1;
            place2 = vertex;
            break;
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
          for(int pt = 0; pt < 4; pt++) {temp_points1.push_back(cells[same_boundary[i]][pt]);}
          curr_centroid_vertex = centroid(temp_points1);
          temp_points1.clear();
          temp_points1.push_back(cells[same_boundary[i]][0]);
          temp_points1.push_back(cells[same_boundary[i]][3]);
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
    return make_tuple(graph_edges, graph_vertices);
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
            if(intersection_trial(segment,obs_side).x != -1){
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


tuple <std::vector<POINT>, std::vector<POINT>> add_start_end(std::vector<POINT> graph_vertices, std::vector<POINT> graph_edges, POINT start_point, vector<POINT> end_point, std::vector<std::vector<POINT>> obstacles){
    //Source
    int min_ind = -1; 
    float min = INFINITY;
    float dist;
    int m;

    POINT temp_point;
    SEGMENT temp_segment;

    for(int vertex = 0; vertex < graph_vertices.size(); vertex ++) {
      temp_segment.a = start_point;
      temp_segment.b = graph_vertices[vertex];

      // FOR DEBUG
      // printing the points that are measured to the start point
      // int output10 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
      // int output11 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
      // int output12 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
      // auto color_rand = cv::Scalar(output10,output11,output12);
      // cv::Point2f point_center22(temp_segment.a.x*enlarge,temp_segment.a.y*enlarge);
      // cv::circle(plot, point_center22, 2,color_rand,cv::FILLED,cv::LINE_8);
      // cv::Point2f point_center222(temp_segment.b.x*enlarge,temp_segment.b.y*enlarge);
      // cv::circle(plot, point_center222, 2,color_rand,cv::FILLED,cv::LINE_8);
      // cv::imshow("Clipper", plot);
      // cv::waitKey(0);

      // find the closest vertex in the graph to the start point
      if(check_obstruction(obstacles, temp_segment)) {
        dist = find_dist(graph_vertices[vertex], start_point);
        // std::cout << "current distance: " << dist << "minimum distance: " << min << std::endl;
        if(dist < min) {
          min = dist;
          min_ind = vertex;
          // std::cout << "distance: " << dist << " closest vertex: " << vertex << std::endl;
        } 
      }
    }

    if(min_ind == -1){
      throw std::logic_error( "THE START POINT IS UNREACHABLE" );
      //std::cout << "the start point is unreachable" << std::endl;
      //return false;
    }

    graph_vertices.push_back(start_point);
    m = graph_vertices.size()-1;
    temp_point.x = min_ind;
    temp_point.y = m;
    graph_edges.push_back(temp_point);



    // destination
    min_ind = -1; 
    min = INFINITY;

    for(int vertex = 0; vertex < graph_vertices.size(); vertex ++) {
      temp_segment.a = end_point[0]; //TODO: change for more than one robot
      temp_segment.b = graph_vertices[vertex];
      if(check_obstruction(obstacles, temp_segment)) {
        dist = find_dist(graph_vertices[vertex], end_point[0]); //TODO: change for more than one robot
        if(dist < min) {
          min = dist;
          min_ind = vertex;
        } 
      }
    }

    if(min_ind == -1){
      throw std::logic_error( "THE END POINT IS UNREACHABLE" );
      //std::cout << "the end point is unreachable" << std::endl;
      //return false;
    }

    graph_vertices.push_back(end_point[0]); //TODO: change for more than one robot
    m = graph_vertices.size()-1;
    temp_point.x = min_ind;
    temp_point.y = m;
    graph_edges.push_back(temp_point);

    return make_tuple(graph_edges, graph_vertices);
}


std::vector< std::vector<int> > graph_construction(std::vector<POINT> graph_vertices, std::vector<POINT> graph_edges) {
    std::vector< std::vector<int> > graph;
    for(int vertex = 0; vertex < graph_vertices.size(); vertex ++) {
      std::vector<int> empty;
      graph.push_back(empty);
      for (POINT &edge : graph_edges){
        if(edge.x == vertex){
          graph[vertex].push_back(edge.y);
        }
        else if(edge.y == vertex){
          graph[vertex].push_back(edge.x);
        }
      }
    }
    return graph;
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


std::vector<std::vector<int>>  optimize_graph(std::vector<int> my_path, std::vector<POINT> new_graph_vertices, std::vector<std::vector<POINT>> obstacles){
    SEGMENT temp_path;
    SEGMENT temp_obs;
    std::vector< std::vector<int> >  optimized_graph;
    POINT inter_result;
    for(int vertex = 0; vertex < new_graph_vertices.size();vertex++){
      std::vector<int> empty;
      optimized_graph.push_back(empty);
      for(int vertex_2 = 0 ;vertex_2<new_graph_vertices.size();vertex_2++){
        bool break_off = false;
        if(vertex_2!=vertex ){
          temp_path.a = {new_graph_vertices[vertex].x,new_graph_vertices[vertex].y};
          temp_path.b = {new_graph_vertices[vertex_2].x,new_graph_vertices[vertex_2].y};
          for(int obs = 0 ; obs< obstacles.size();obs++){
            for(int pt = 0 ; pt< obstacles[obs].size()-1;pt++){
              temp_obs.a = {obstacles[obs][pt].x,obstacles[obs][pt].y};
              temp_obs.b = {obstacles[obs][pt+1].x,obstacles[obs][pt+1].y};
              inter_result = intersection_trial(temp_obs,temp_path);
              if(inter_result.x !=-1){
                break_off = true;
                break;
              }
            }
            if(break_off){
              break;
            }
          }
          if(break_off){
            continue;
          }
          // FOR DEBUG
          // std::cout << "vertex1: " << vertex << " vertex2: " << vertex_2 << endl;
          // std::cout << "temp obs: ( " << temp_obs.a.x << " , " << temp_obs.a.y << " ) ( " << temp_obs.b.x <<" ," << temp_obs.b.y << " )"<< endl;
          // std::cout << "temp obs: ( " << temp_path.a.x << " , " << temp_path.a.y << " ) ( " << temp_path.b.x <<" ," << temp_path.b.y << " )"<< endl;

          optimized_graph[vertex].insert(optimized_graph[vertex].begin(),vertex_2);
        }
      }
      // std::cout << "-------------" << endl;
    }
    // cout << "size of optimized graph: " << optimized_graph.size();


    return optimized_graph;
}

std::vector<int> look_ahead_optimize(std::vector<int> my_path, std::vector<POINT> graph_vertices, std::vector<std::vector<POINT>> obstacles, int look_ahead, float gamma){
  SEGMENT temp_path;
  SEGMENT temp_obs;
  float distance= 0.0 ;
  float best_distance;
  int best_point;
  int next_point;
  bool break_loop;
  int cap = 0;
  float gamma_i;
  float angle;
  POINT inter_result;
  const double pi = boost::math::constants::pi<double>();

  // to guarentee that look ahead won't exceed the path length
  if(look_ahead>my_path.size()-1){look_ahead=my_path.size()-1;}

  std::vector<int> optimized_path;
  optimized_path.push_back(my_path[0]);
  for(int i = 0;i<my_path.size()-1;i++){
    gamma_i = 1;
    // std::cout << "i at beginning: " << i << std::endl;
    best_distance = INFINITY;
    next_point = i+1;
    best_point = my_path[next_point];
    if(i<=my_path.size()-look_ahead-1){cap = look_ahead;}else{cap= my_path.size()-1-i;}
    // std::cout << "cap is ::: " << cap << std::endl;
    for(int j=1;j<=cap;j++){
      // std::cout << j+i << " , " << my_path.size() << std::endl;
      break_loop= false;
      // std::cout << "point " << i <<  " " << graph_vertices[my_path[i]].x << " , " << graph_vertices[my_path[i]].y << "point " << j+i <<  " " << graph_vertices[my_path[i+j]].x << " , " << graph_vertices[my_path[i+j]].y << std::endl;
      distance = sqrt(pow(graph_vertices[my_path[i]].x - graph_vertices[my_path[i+j]].x,2) + pow(graph_vertices[my_path[i]].y - graph_vertices[my_path[i+j]].y,2));
      distance = gamma_i * distance ;
      angle = atan2(graph_vertices[my_path[i]].y-graph_vertices[my_path[i+j]].y,graph_vertices[my_path[i]].x-graph_vertices[my_path[i+j]].x)*180/pi;
      // std::cout << "angle" << angle<< std::endl;
      // std::cout << "distance: " << distance << " best dissstance : " << best_distance  << " best pnt("<< next_point << std::endl;
      if(distance <= best_distance){
        // std::cout << "less distance detected -- checking collision" << std::endl;
        temp_path.a = {graph_vertices[my_path[i]].x, graph_vertices[my_path[i]].y};
        temp_path.b = {graph_vertices[my_path[i+j]].x, graph_vertices[my_path[i+j]].y};        
        for(int obs = 0 ; obs< obstacles.size();obs++){
          for(int pt = 0 ; pt< obstacles[obs].size()-1;pt++){
            temp_obs.a = {obstacles[obs][pt].x,obstacles[obs][pt].y};
            temp_obs.b = {obstacles[obs][pt+1].x,obstacles[obs][pt+1].y};
            inter_result = intersection_trial(temp_obs,temp_path);
            if(inter_result.x !=-1){
              break_loop = true;
              // std::cout << "COLLISION DETECTED" <<std::endl;
              break;
            }
          }
          if(break_loop){
            break;
          }
        }
        if(break_loop){
          continue;
        }
        else{
          // std::cout << " NO COLLISION DETECTED" << std::endl;
          best_distance = distance;
          best_point = my_path[j+i];
          next_point = i+j;
        }      
      }
      gamma_i*=gamma;
    }
    
    i = next_point-1;
    
    optimized_path.push_back(my_path[next_point]);
    // cv::Point2f centerCircle(opt_points[opt_points.size()-1].x*enlarge,opt_points[opt_points.size()-1].y*enlarge);
    // std::string text = std::to_string(next_po);
    // putText(plot, text, centerCircle, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));
    // std::cout << "adding pt #: (" << next_point << ")" << graph_vertices[my_path[my_path.size()-1]].x << " , " << graph_vertices[my_path[my_path.size()-1]].y << std::endl;
    // std::cout << "i at end: " << next_point <<  "========================" << std::endl;
  }
  std::cout<< "Optimized path with look ahead points: total points "<< optimized_path.size() << std::endl;
  for(int i = 0 ; i<optimized_path.size();i++){
    std::cout << "Point # " << i << " ( " << graph_vertices[optimized_path[i]].x << " , " << graph_vertices[optimized_path[i]].y << " )" << endl;
  }
  return optimized_path;
}

std::vector<robotPos> index_to_coordinates(std::vector<int> index_path, std::vector<POINT> coordinates) {
    std::vector<robotPos> path_points;
    robotPos temp_pt;
    for (unsigned i=0; i<index_path.size(); i++) {
        temp_pt = {coordinates[index_path[i]].x, coordinates[index_path[i]].y,-1};
        // cout << "optimized point: (x: " << temp_pt.x << " , y: "<<  temp_pt.y << " , theta: " << temp_pt.th << " ) " << endl;
        path_points.push_back(temp_pt);
    }
    return path_points;
}


void print_data(std::vector<POINT> boundary, std::vector<POINT> start_point, std::vector<POINT> end_point, std::vector< std::vector<POINT> > obstacles, std::vector<POINT> graph_vertices, std::vector< std::vector<int> > graph, std::vector<POINT> new_graph_vertices, std::vector< std::vector<int> > optimized_graph, std::vector<int> path, std::vector<int> optimized_path, std::vector<robotPos> path_points) {    
    
    // FOR DEBUG
    // std::cout<<"\n>>>> Border postion:"<<std::endl;
    // for(int i = 0; i < boundary.size(); i++){
    //  cout<< "x=" << boundary[i].x*enlarge << " y=" << boundary[i].y*enlarge << endl;
    // }

    //FOR DEBUG
    // std::cout<<"\n>>>> Starting points:"<<std::endl;
    // for(int i = 0; i < start_point.size(); i++){
    //  cout<< "x=" << start_point[i].x << " y=" << start_point[i].y << " theta="<< start_point[i].theta << endl;
    // } 

    //FOR DEBUG
    //std::cout<<"\n>>>> End points:"<<std::endl;
    //for(int i = 0; i < end_point.size(); i++){
    //  cout<< "x=" << end_point[i].x << " y=" << end_point[i].y << endl;
    //}

    //FOR DEBUG
    // std::cout<<"\n>>>> Obstacles:"<<std::endl;
    // for(int i = 0; i < obstacles.size(); i++){
    //  std::cout<<"\nObstacle "<< i << endl;
    //  for(int j = 0; j < obstacles[i].size(); j++){
    //    cout<< "x=" << obstacles[i][j].x << " y=" << obstacles[i][j].y << endl;
    //  }
    // }    
    cout << endl;
    cout <<"GRAPH VERTICES: "<< endl; 
    for(int i = 0; i < graph_vertices.size(); i++){
      cout << "(" << graph_vertices[i].x << "," << graph_vertices[i].y;
      if(i == graph_vertices.size() - 1) { cout << ") "; }
      else cout << "), ";
    }
    cout << endl;
    cout << endl;

    cout <<"NEW GRAPH VERTICES: "<< endl; 
    for(int i = 0; i < new_graph_vertices.size(); i++){
      cout << "(" << new_graph_vertices[i].x << "," << new_graph_vertices[i].y;
      if(i == new_graph_vertices.size() - 1) { cout << ") "; }
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
    cout << endl;

    cout <<"OPTIMIZED GRAPH: "<< endl;
    for(int j = 0; j < optimized_graph.size(); j++){
      cout << "(";
      for(int k = 0; k < optimized_graph[j].size(); k++) {  
        cout << optimized_graph[j][k];
        if(k != (optimized_graph[j].size() - 1)) {cout << ", ";}
      }
      if(j == (optimized_graph.size() - 1)) {cout << ") "; }
      else cout << "), ";
    }
    cout << endl;
    cout << endl;

    cout <<"PATH: "<< endl; 
    for(int i = 0; i < path.size(); i++){
      cout << path[i];
      if(i != path.size() - 1) { cout << ", "; }
    }
    cout << endl;
    cout << endl;

    cout <<"OPTIMIZED PATH: "<< endl; 
    for(int i = 0; i < optimized_path.size(); i++){
      cout << optimized_path[i];
      if(i != optimized_path.size() - 1) { cout << ", "; }
    }
    cout << endl;
    cout << endl;

    cout <<"OPTIMIZED PATH WITH COORDINATES: "<< endl; 
    for(int i = 0; i < path_points.size(); i++){
      cout << "x=" << path_points[i].x << " y=" << path_points[i].y << " theta=" << path_points[i].th << endl;
    }
    cout << endl;
    cout << endl;   
}


// ----------------- GRAVE YARD ----------------------


// bool points_successive (POINT a, POINT b, std::vector<POINT>  obstacle){
//         for(int i= 0 ;i< obstacle.size();i++){
//             if(a.x == obstacle[i].x && a.y == obstacle[i].y){
//                 std::cout << " a found" << endl;
//                 if(i==obstacle.size()-1){
//                     return false;
//                 }
//                 std::cout << " next point: (" << obstacle[i+1].x << " , " << obstacle[i+1].y 
//                 << ") point b : ( " << b.x << " , " << b.y << endl;
//                 return (obstacle[i+1].x == b.x && obstacle[i+1].y == b.y);
//             }
//             if(b.x == obstacle[i].x && b.y == obstacle[i].y){
//                 std::cout << " b found" << endl;
//                 if(i==obstacle.size()-1){
//                     return false;
//                 }
//                 std::cout << " next point: (" << obstacle[i+1].x << " , " << obstacle[i+1].y 
//                 << ") point a : ( " << a.x << " , " << a.y << endl;
//                 return (obstacle[i+1].x == a.x && obstacle[i+1].y == a.y);
//             }
//         }
//     return false;
// }

// int points_from_same_obs (POINT a, POINT b, std::vector<POINT> obstacles){
//     int obs1_num = -1;
//     int obs2_num = -1;
//     for(POINT curr_point : obstacles){
//         std::cout << "current point: ( " << curr_point.x << " , " << curr_point.y << ") point a: (" << a.x << " , " << a.y << ") " << endl;
//         if(curr_point.x == a.x && curr_point.y == a.y){
//             obs1_num = curr_point.obs;
//         }
//         if(curr_point.x == b.x && curr_point.y == b.y){
//             obs2_num = curr_point.obs;
//         }
//         if(obs1_num !=-1 && obs2_num != -1){
//             break;
//         }
//     }
//     std::cout << "obs1: " << obs1_num << " obs2: " << obs2_num << endl;
//     if(obs1_num == obs2_num){
//         return obs1_num;
//     }
//     return -1;
// }


//Function to determine the intersection of two segments; source https://flassari.is/2008/11/line-line-intersection-in-cplusplus/
// POINT intersection(SEGMENT segment1, SEGMENT segment2) {   
//     POINT intersection_point;

//     float d = (segment1.a.x - segment1.b.x) * (segment2.a.y - segment2.b.y) - (segment1.a.y - segment1.b.y) * (segment2.a.x - segment2.b.x);

//     if (d == 0) {
// 		intersection_point.x = (-1);
//     	intersection_point.y = (-1);
//         return intersection_point;
//     }

//     // Get the x and y
//     float pre = (segment1.a.x*segment1.b.y - segment1.a.y*segment1.b.x); 
//     float post = (segment2.a.x*segment2.b.y - segment2.a.y*segment2.b.x);
//     float x = ( pre * (segment2.a.x - segment2.b.x) - (segment1.a.x - segment1.b.x) * post ) / d;
//     float y = ( pre * (segment2.a.y - segment2.b.y) - (segment1.a.y - segment1.b.y) * post ) / d;

//     if ( int(1000000*x) < int(1000000*min(segment1.a.x, segment1.b.x)) || int(1000000*x) > int(1000000*max(segment1.a.x, segment1.b.x)) || int(1000000*x) < int(1000000*min(segment2.a.x, segment2.b.x)) || int(1000000*x) > int(1000000*max(segment2.a.x, segment2.b.x)) ) {
// 		intersection_point.x = (-1);
//     	intersection_point.y = (-1);
//         return intersection_point;
//     }
//     if ( int(1000000*y) < int(1000000*min(segment1.a.y, segment1.b.y)) || int(1000000*y) > int(1000000*max(segment1.a.y, segment1.b.y)) || int(1000000*y) < int(1000000*min(segment2.a.y, segment2.b.y)) || int(1000000*y) > int(1000000*max(segment2.a.y, segment2.b.y)) ) {
// 		intersection_point.x = (-1);
//     	intersection_point.y = (-1);
//         return intersection_point;
//     }

//     intersection_point.x = x; //(int(x+0.5));
//     intersection_point.y = y; //(int(y+0.5));


//     return intersection_point;
// }


// function that returns the determinant of a 2x2 matrix
// float determinant( POINT a , POINT b){
//     return float ( (a.x * b.y) - (a.y * b.x) );
// }

// bool counter_clockwise(POINT A,POINT B,POINT C){
//     return double ((C.y-A.y) * (B.x-A.x)) > double ((B.y-A.y) * (C.x-A.x));
// }

// // function to test if two lines intersect. returns true if they are
// bool intersect(POINT A,POINT B,POINT C,POINT D,bool print = false){
//     //Check if any three points are co-linear

//     float t1 = double(A.x * (B.y - C.y)) + double(B.x * (C.y - A.y)) + double(C.x * (A.y - B.y));
//     float t2 = double(A.x * (B.y - D.y)) + double(B.x * (D.y - A.y)) + double(D.x * (A.y - B.y));
//     float t3 = double(A.x * (C.y - D.y)) + double(C.x * (D.y - A.y)) + double(D.x * (A.y - C.y));
//     float t4 = double(B.x * (C.y - D.y)) + double(C.x * (D.y - B.y)) + double(D.x * (B.y - C.y));

//     if (print){
//         std::cout << "-t1: " <<  t1 << " -> " << (t1 == 0)  << std::endl;
//         std::cout << "-t2: " <<  t2 << " -> " << (t2 == 0)  << std::endl;
//         std::cout << "-t3: " <<  t3 << " -> " << (t3 == 0)  << std::endl;
//         std::cout << "-t4: " <<  t4 << " -> " << (t4 == 0)  << std::endl;
//         std::cout << "-t5: " << (counter_clockwise(A,C,D) != counter_clockwise(B,C,D) && counter_clockwise(A,B,C) != counter_clockwise(A,B,D)) << std::endl;
//     }
//     // if statements to check if any 3 points are co-linear
//     // if( t1 == 0 || t2 == 0 || t3 == 0 || t4 ==0){
//     //     return true;
//     // }

//     return counter_clockwise(A,C,D) != counter_clockwise(B,C,D) && counter_clockwise(A,B,C) != counter_clockwise(A,B,D);;
// }

// // calculates the point of intersection between two lines
// // must be only triggered if the two lines are infact intersecting
// POINT line_intersection(POINT A, POINT B, POINT C, POINT D) { 
//     POINT inter_p = {-1,-1};

//     POINT x_diff = {A.x - B.x , C.x - D.x};
//     POINT y_diff = {A.y - B.y , C.y - D.y};
//     float div = determinant(x_diff, y_diff);

//     if (div == 0){
//         return inter_p;
//     }
//     float h1 = determinant(A,B);
//     float h2 = determinant(C,D);
//     POINT d = {h1,h2};
//     float x = determinant(d,x_diff) / div;
//     float y = determinant(d, y_diff) / div;

//     inter_p = {x,y};
//     // std::cout << "intersection point: " << inter_p.x << inter_p.y << std::endl;
//     return inter_p;
//     }

// // function to test if two segments of line are intersecting
// // if they are not, it returns a point of -1. if they are, it calcuates the intersection point
// POINT segment_intersection(SEGMENT sigment1, SEGMENT sigment2,bool print = false){
//     POINT intersection_p = {-1,-1};
//     POINT a = sigment1.a;
//     POINT b = sigment1.b;
//     POINT c = sigment2.a;
//     POINT d = sigment2.b;
//     int enlarge = 600;

//     if (print){
//         std::cout << "-- a: (" << a.x*enlarge << "," << a.y*enlarge << ") b: (" << b.x*enlarge << "," << b.y*enlarge << ")" << std::endl;
//         std::cout << "-- c: (" << c.x*enlarge << "," << c.y*enlarge << ") d: (" << d.x*enlarge << "," << d.y*enlarge << ")" << std::endl;
//         std::cout << "-- the intersect call func result: " << (intersect(a, b, c, d)) << std::endl;
//     }
    
//     if( intersect(a, b, c, d,print)){
//         return line_intersection(a, b, c, d);
//     }
//     return intersection_p;  
// }