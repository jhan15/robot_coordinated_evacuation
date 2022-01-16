#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <iterator>
#include <string> 

#include <cmath>
#include "dubins.h"
#include "inflate_objects.hpp"
#include "vertical_cell_decomposition.hpp"

int enlarge = 600;

namespace student {

 void loadImage(cv::Mat& img_out, const std::string& config_folder){  
   throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED" );
 }

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED" );
  }

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED" );   
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){

    throw std::logic_error( "STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED" );  

  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, 
                        const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, 
                        const std::vector<cv::Point2f>& dest_image_points_plane, 
                        cv::Mat& plane_transf, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED" );  
  }


  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
            const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED" );   
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED" );   
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED" );    
  }

  void reset_obs_plot(cv::Mat plot,std::vector< std::vector<POINT> > obstacles){
      for (unsigned i=0; i<obstacles.size(); i++) {
          for(unsigned j=1; j<obstacles[i].size();j++){
              cv::line(plot, cv::Point2f(obstacles[i][j-1].x*enlarge,obstacles[i][j-1].y*enlarge), cv::Point2f(obstacles[i][j].x*enlarge,obstacles[i][j].y*enlarge), cv::Scalar(255,255,255), 2);
              cv::line(plot, cv::Point2f(obstacles[i][j-1].x*enlarge,obstacles[i][j-1].y*enlarge), cv::Point2f(obstacles[i][j].x*enlarge,obstacles[i][j].y*enlarge), cv::Scalar(210,210,210), 1);
              if (j == obstacles[i].size() -1){
                  cv::line(plot, cv::Point2f(obstacles[i][j].x*enlarge,obstacles[i][j].y*enlarge), cv::Point2f(obstacles[i][0].x*enlarge,obstacles[i][0].y*enlarge), cv::Scalar(255,255,255), 2);
                  cv::line(plot, cv::Point2f(obstacles[i][j].x*enlarge,obstacles[i][j].y*enlarge), cv::Point2f(obstacles[i][0].x*enlarge,obstacles[i][0].y*enlarge), cv::Scalar(210,210,210), 1);        
              }
          }
      }     
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list,
                const std::vector<Polygon>& gate_list,
                const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta,
                std::vector<Path>& path, const std::string& config_folder){


    // drawing the solution
    int l = 1000;        
    cv::Mat plot(l - 300,l, CV_8UC3, cv::Scalar(255,255,255));

    // inflating the obsticales and borders of the arena
    int inflate_value = 0;
    
    std::vector<Polygon> inflated_obstacle_list = inflate_obstacles(obstacle_list,inflate_value,plot);

    const Polygon inflated_borders = inflate_borders(borders,-inflate_value,plot);



    // TO DO: implement a function to merge the obstacles that are over lapping
    // TO DO: delete all the vertcices outside of the borders


    // You can test the roadmap here --------------

    POINT temp_point;

    std::vector<POINT> boundary;

    for (const auto &position : borders) {
      temp_point.x = position.x;
      temp_point.y = position.y;
      boundary.push_back(temp_point);
    }

    //std::cout<<"\n>>>> Border postion:"<<std::endl;
    //for(int i = 0; i < boundary.size(); i++){
    //  cout<< "x=" << boundary[i].x << " y=" << boundary[i].y << endl;
    //}
    

    std::vector<POINT> start_point;

    for (int i = 0; i < x.size(); i++) {
      temp_point.x = x[i];
      temp_point.y = y[i];
      temp_point.theta = theta[i];
      start_point.push_back(temp_point);
    }

    
    // std::cout<<"\n>>>> Starting points:"<<std::endl;
    // for(int i = 0; i < start_point.size(); i++){
    //  cout<< "x=" << start_point[i].x << " y=" << start_point[i].y << " theta="<< start_point[i].theta << endl;
    // }
    
    
    std::vector<POINT> end_point;
    std::vector<POINT> gate;

    for (int i = 0; i < gate_list.size(); i++) {
      //cout << "\n>>>> Gate " << i << endl;
      for (const auto &position : gate_list[i]) {
        temp_point.x = position.x;
        temp_point.y = position.y;
        gate.push_back(temp_point);
        //cout<< "x=" << temp_point.x << " y=" << temp_point.y << endl;
      }
      end_point.push_back(centroid(gate));
      gate.clear();
    }

    
    //std::cout<<"\n>>>> End points:"<<std::endl;
    //for(int i = 0; i < end_point.size(); i++){
    //  cout<< "x=" << end_point[i].x << " y=" << end_point[i].y << endl;
    //}
    

    std::vector< std::vector<POINT> > obstacles;
    std::vector<POINT> obstacle;
    int vertices_num = 0;

    for (int i = 0; i < obstacle_list.size(); i++) { // change to inflated_obstacle_list later
      for (const auto &position : obstacle_list[i]) { // change to inflated_obstacle_list later
        temp_point.x = position.x;
        temp_point.y = position.y;
        temp_point.obs = i;
        obstacle.push_back(temp_point);
        vertices_num += 1;
      }
      obstacles.push_back(obstacle);
      obstacle.clear();
    }

    
    //std::cout<<"\n>>>> Obstacles:"<<std::endl;
    //for(int i = 0; i < obstacles.size(); i++){
    //  std::cout<<"\nObstacle "<< i << endl;
    //  for(int j = 0; j < obstacles[i].size(); j++){
    //    cout<< "x=" << obstacles[i][j].x << " y=" << obstacles[i][j].y << endl;
    //  }
    //}
  
    //sorting the vertices by their x value in increasing order
    std::vector<POINT> sorted_vertices;

    //filling the vector with the required number of placeholder points 
    for(int curr_vertex = vertices_num - 1; curr_vertex >= 0; curr_vertex--) {
      temp_point.x = -1;
      temp_point.y = -1;
      temp_point.obs = -1;
      sorted_vertices.push_back(temp_point);
    }
       
    int add_to_list;

    for(int curr_vertex = vertices_num - 1; curr_vertex >= 0; curr_vertex--) {
      for(int obj = 0; obj < obstacles.size(); obj++) {
        for(int vertex = 0; vertex < obstacles[obj].size(); vertex++) {
          add_to_list = 0;
          if(obstacles[obj][vertex].x > sorted_vertices[curr_vertex].x) { //the x of the vertex is bigger than the current one in this position
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

    std::cout<< "vertices number: " << vertices_num << std::endl;
    // std::cout<<"\n>>>> Sorted vertices:"<<std::endl;

    //print out the vertices of the obstacles
    for(int i = 0; i < sorted_vertices.size(); i++){
      //  cout<< "x=" << sorted_vertices[i].x << " y=" << sorted_vertices[i].y << endl;
       cv::Point2f centerCircle(sorted_vertices[i].x*enlarge,sorted_vertices[i].y*enlarge);
       cv::circle(plot, centerCircle, 2,cv::Scalar( 40, 30, 125 ),cv::FILLED,cv::LINE_8);
      //  std::string text = std::to_string(sorted_vertices[i].obs);
      //  putText(plot, text, centerCircle, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));
    }
    cv::imshow("Clipper", plot);
    cv::waitKey(0);  

    //Determining the vertical lines
    float y_limit_lower = min(min(boundary[0].y, boundary[1].y), min(boundary[2].y, boundary[3].y));
    float y_limit_upper = max(max(boundary[0].y, boundary[1].y), max(boundary[2].y, boundary[3].y));

    //cout << "\nlimits " << y_limit_lower << " " << y_limit_upper << endl;

    std::vector< SEGMENT > open_line_segments;
    SEGMENT curr_segment;

    //add the first point of the obstacle to the end to close the polygon
    for(int obs = 0; obs < obstacles.size(); obs++) {
      obstacles[obs].push_back(obstacles[obs][0]);
    }
    float prev_x = -1;

    for(const POINT& pt : sorted_vertices){
      // to ignore this pt if the previous pt was not that far away
      // std::cout << "previous x:" << prev_x << " current x: " << pt.x << " difference: " << (pt.x-prev_x) << std::endl;
      // if (prev_x != -1 && prev_x != pt.x && (pt.x-prev_x < 0.0003) ){
      //   continue;
      // }

      prev_x = pt.x;
      int up = 0;
      int down = 0;
      int break_now = 0;
      POINT lower_obs_pt;
      POINT upper_obs_pt;
      POINT intersection_point;
      SEGMENT temp_segment;
      
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

          // check if the current vertex of the obsticale is equal to any of the segment points
          // this is to stop counting the current vertex as an intersection
          if((curr_segment.a.x == pt.x && curr_segment.a.y == pt.y) ||
          (curr_segment.b.x == pt.x && curr_segment.a.y == pt.y) || 
          (temp_segment.a.x == pt.x && temp_segment.a.y == pt.y) ||
          (temp_segment.b.x == pt.x && temp_segment.b.y == pt.y)){
            intersection_point = {-1,-1};
          }
          else{
            intersection_point = segment_intersection(curr_segment, temp_segment,false);
          }

          if(intersection_point.x != -1){ 
            // printing the intersection points of the vertical lines with the obstacles and showing whats left from them
            // cv::Point2f point_center2(pt.x*enlarge,pt.y*enlarge);
            // std::string text = std::to_string(pt.obs);
            // putText(plot, text, point_center2, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));
            // cv::Point2f point_center(intersection_point.x*enlarge,intersection_point.y*enlarge);
            // cv::circle(plot, point_center, 5,cv::Scalar( 0, 10, 125 ),2);
            // cv::imshow("Clipper", plot);
            // cv::waitKey(0);
            // std::cout << "temp_segment.a " << temp_segment.a.x << " " << temp_segment.a.y << endl;
            // std::cout << "temp_segment.b " << temp_segment.b.x << " " << temp_segment.b.y << endl;
            // std::cout << "curr_segment.a " << curr_segment.a.x << " " << curr_segment.a.y << endl;
            // std::cout << "curr_segment.b " << curr_segment.b.x << " " << curr_segment.b.y << endl;
            // std::cout << "intersection_point " << intersection_point.x << " " << intersection_point.y << endl;
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

      // printing the vertical lines for debugging
      // std::cout << "=========================" << std::endl;
      if(down == 0){
          cv::line(plot, cv::Point2f(lower_obs_pt.x*enlarge,lower_obs_pt.y*enlarge), cv::Point2f(pt.x*enlarge,pt.y*enlarge), cv::Scalar(0,255,0), 1);
      }
      if(up == 0){
        cv::line(plot, cv::Point2f(upper_obs_pt.x*enlarge,upper_obs_pt.y*enlarge), cv::Point2f(pt.x*enlarge,pt.y*enlarge), cv::Scalar(0,255,0), 1);
      }
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

    // print out the open line segments
    // std::cout << "open line segments:" << std::endl;
    // for(int i = 0; i < open_line_segments.size(); i++){
    //  std::cout << "seg # "<< i << " A: x=" << open_line_segments[i].a.x << " y=" << open_line_segments[i].a.y << " B: x=" << open_line_segments[i].b.x << " y=" << open_line_segments[i].b.y << std::endl; 
    // }

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
    std::vector<int> done;

    for(int i = 0; i < open_line_segments.size(); i++) {
      curr_segment = open_line_segments[i];
      curr_vertex = sorted_vertices[i];
      break_now = 0;
      done = {0,0,1};

      // a is lower limit , b is upper limit
      // if lower limit is blocked -> don't look down
      if(curr_segment.a.x == -1) {done[0] = 1;}
      // if upper lmit is blocked -> don't look up :)
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

        next_segment = open_line_segments[j];
        next_vertex = sorted_vertices[j];
        // check to see if the next segemnt is completely blocked on both its points
        double_check = next_segment.a.x != -1 && next_segment.b.x != -1;

        // group 0 -> not blocked from the bottom
        // group 1 -> not blocked from the top
        // group 2 -> completely blocked

        // if not blocked from the bottom
        if(done[0] == 0) {
          // std::cout << "test -> done[0]  == 0 [not blocked from the bottom" << std::endl;
          // and the next vertical line is not completely blocked on both of its points
          if(double_check) {
            // make a segment out of the lower limit of the current vertical line, 
            // and the current point of the obstacle
            temp_points1.push_back(curr_segment.a);
            temp_points1.push_back(curr_vertex);
            //make a segment out of the lower limit of the next vertical lines,
            // and the next point of the next higher x value.
            temp_points2.push_back(next_segment.a);
            temp_points2.push_back(next_vertex);
            // find the center of both these segments          
            temp_segment.a = centroid(temp_points1); 
            temp_segment.b = centroid(temp_points2); 
            lines_to_check.push_back(temp_segment);
            group.push_back(0);

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
            temp_points2.clear();
            temp_points2.push_back(next_segment.b);
            temp_points2.push_back(next_vertex);          
            temp_segment.b = centroid(temp_points2); 
            lines_to_check.push_back(temp_segment);
            group.push_back(0);
            //trapezoids
            temp_points1.clear();
            // the lower point from the current vertical line, the lower point from the next vertical line
            // the next point on the obstacle, and the current point of the obsticale
            temp_points1.push_back(curr_segment.a);
            temp_points1.push_back(next_segment.a);
            temp_points1.push_back(next_vertex);
            temp_points1.push_back(curr_vertex);
            trapezoids.push_back(temp_points1);

            // print debug for temp points
            // int output4 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            // int output5 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            // int output6 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            // auto color_rand = cv::Scalar(output4,output5,output6);
            // cv::line(plot, cv::Point2f(temp_points1[0].x*enlarge,temp_points1[0].y*enlarge), cv::Point2f(temp_points1[1].x*enlarge,temp_points1[1].y*enlarge), color_rand, 2);
            // cv::line(plot, cv::Point2f(temp_points1[1].x*enlarge,temp_points1[1].y*enlarge), cv::Point2f(temp_points1[2].x*enlarge,temp_points1[2].y*enlarge), color_rand, 2);
            // cv::line(plot, cv::Point2f(temp_points1[2].x*enlarge,temp_points1[2].y*enlarge), cv::Point2f(temp_points1[3].x*enlarge,temp_points1[3].y*enlarge), color_rand, 2);
            // cv::line(plot, cv::Point2f(temp_points1[3].x*enlarge,temp_points1[3].y*enlarge), cv::Point2f(temp_points1[0].x*enlarge,temp_points1[0].y*enlarge), color_rand, 2); 
            // cv::imshow("Clipper", plot);
            // cv::waitKey(0); 

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
            // trapezoids
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
        temp_points1.clear();
        temp_points2.clear();
        if(done[1] == 0) {
          if(double_check) {
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


            // change from the repo trapezoids.append([ curr_vertex, next_segment[0], next_vertex , point(curr_segment[1].x, curr_segment[1].y,curr_segment[1].obstacle, 34)]);
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
        temp_points1.clear();
        temp_points2.clear();
        if(done[2] == 0) {
          if(double_check) {
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
            // trapezoids
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
            // trapezoids
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
            // trapezoids
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


        // print debug for lines_to_check
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
          for(int obs = 0; obs < obstacles.size(); obs++) {              //for index3,obs in enumerate(new_obstacles): //obs.append( obs[0] ); <-already done
            for(int vertex = 0; vertex < obstacles[obs].size()-1; vertex++) { //for index4 in range(len(obs)-1):
              temp_segment.a = obstacles[obs][vertex];
              temp_segment.b = obstacles[obs][vertex+1];
              temp_point = segment_intersection(lines_to_check[line], temp_segment,false);
              if (temp_point.x != -1){
                bool _check = intersect(temp_segment.a,temp_segment.b,lines_to_check[line].a,lines_to_check[line].b,false);
                if (_check == 0){
                  temp_point = {-1,-1};
                }
              }
              // print the intersection prosedure
              // if ((i == 34 || i == 35) && temp_point.x != -1){
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
              // }
              
              if(temp_point.x != -1) {                      //if (segment_intersection( q[0], q[1],  obs[index4],  obs[index4+1]) != -1):
                no_intersection[group[line]] = 0;                 //ok[q[2]] = False;
                int found = 0;
                for(int idx = 0; idx < temp_to_remove.size(); idx++) {
                  if(line == temp_to_remove[idx]) {
                    found = 1;
                    break;
                  }
                }
                if(found == 0) {                        //if(index5 not in temp_to_remove):
                  temp_to_remove.push_back(line);                 //temp_to_remove.append(index5);
                } 
              }
            // std::cout << "-- intersection_point: " << "(" << temp_point.x << " , " << temp_point.y << ")" << std::endl;
            }      
          }
          
          // std::cout << "-- no_intersection stat: " << no_intersection[0] << " , "  << no_intersection[1] << " , " << no_intersection[2] << std::endl;
          if(no_intersection[group[line]] == 1) {done[group[line]] = 1;}      //if (  ok[q[2]] is True ):   //done[q[2]] = True;

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

            // debug for printing the cells one by one
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

    // int output4 = 0;
    // int output5 = 0;
    // int output6 = 0;
    // int enlarge = 600;
    // std::cout << "cell size " << cells.size() << std::endl;
    // for (unsigned i=0; i<cells.size(); i++) {
    //   output4 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
    //   output5 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
    //   output6 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
    //   auto color_rand = cv::Scalar(output4,output5,output6);
    //   for(unsigned j=1; j<cells[i].size(); j++){
    //     cv::Point2f point_center(cells[i][j-1].x*enlarge,cells[i][j-1].y*enlarge);
    //     // std::cout << ABC[i][j].x << ' ' << ABC[i][j].y << std::endl;
    //     cv::circle(plot, point_center, 2,cv::Scalar( 40, 30, 125 ),cv::FILLED,cv::LINE_8);
    //     cv::line(plot, cv::Point2f(cells[i][j-1].x*enlarge,cells[i][j-1].y*enlarge), cv::Point2f(cells[i][j].x*enlarge,cells[i][j].y*enlarge), color_rand, 1);
    //   }
    //   cv::line(plot, cv::Point2f(cells[i][cells.size()-1].x*enlarge,cells[i][cells.size()-1].y*enlarge), cv::Point2f(cells[i][0].x,cells[i][0].y), color_rand, 1);
    // }


    //Merge overlapping polygons
    std::vector< std::vector<POINT> > quad_cells; 
    std::vector< std::vector<POINT> > tri_cells;
    std::vector< std::vector<POINT> > other_cells;

    for(int cell = 0; cell < cells.size(); cell++) {
      if(cells[cell].size() > 3) {quad_cells.push_back(cells[cell]);}
      else if(cells[cell].size() == 3) {tri_cells.push_back(cells[cell]);}
      else {other_cells.push_back(cells[cell]);}
    }

    // print out debug
    // int output4 = 0;
    // int output5 = 0;
    // int output6 = 0;
    // int enlarge = 600;
    // std::cout << "quad cell size" << quad_cells.size() << std::endl;
    // for (unsigned i=0; i<quad_cells.size(); i++) {
    //   output4 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
    //   output5 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
    //   output6 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
    //   auto color_rand = cv::Scalar(output4,output5,output6);
    //   for(unsigned j=1; j<quad_cells[i].size(); j++){
    //     cv::Point2f point_center(quad_cells[i][j-1].x*enlarge,quad_cells[i][j-1].y*enlarge);
    //     // std::cout << ABC[i][j].x << ' ' << ABC[i][j].y << std::endl;
    //     cv::circle(plot, point_center, 2,cv::Scalar( 40, 30, 125 ),cv::FILLED,cv::LINE_8);
    //     cv::line(plot, cv::Point2f(quad_cells[i][j-1].x*enlarge,quad_cells[i][j-1].y*enlarge), cv::Point2f(quad_cells[i][j].x*enlarge,quad_cells[i][j].y*enlarge), color_rand, 1);
    //   }
    // }

    std::vector<int> quads_to_remove;
    std::vector< std::vector<POINT> > quads_to_add;
    std::vector<POINT> temp1;
    std::vector<POINT> temp2;
    std::vector<POINT> new_quad;
    int area1, area2, area3;

    for(int cell1 = 0; cell1 < quad_cells.size(); cell1++) {
      for(int cell2 = 0; cell2 < quad_cells.size(); cell2++) {
        if(cell1 != cell2) {
          if(quad_cells[cell1][0].x == quad_cells[cell2][0].x && quad_cells[cell1][1].x == quad_cells[cell2][1].x) {
          
            temp1 = quad_cells[cell1];
            // add the first point to the back
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

    // for (int i = 0 ; i < quads_to_remove.size();i++){
    //   std::cout << "quads to remove: " << quads_to_remove[i] << std::endl;
    // }


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
 
    // for (int i = 0 ; i < quads_to_remove.size();i++){
    //   std::cout << "quads to remove: " << quads_to_remove[i] << std::endl;
    // }

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
          break;
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
    float min = 9999999;
    float dist;
    int m;

    for(int vertex = 0; vertex < graph_vertices.size(); vertex ++) {
      temp_segment.a = start_point[0]; //TODO: change for more than one robot
      temp_segment.b = graph_vertices[vertex];

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
        dist = find_dist(graph_vertices[vertex], start_point[0]); //TODO: change for more than one robot
        // std::cout << "current distance: " << dist << "minimum distance: " << min << std::endl;
        if(dist < min) {
          min = dist;
          min_ind = vertex;
          // std::cout << "distance: " << dist << " closest vertex: " << vertex << std::endl;
        } 
      }
    }
      cv::imshow("Clipper", plot);
      cv::waitKey(0);
    graph_vertices.push_back(start_point[0]); //TODO: change for more than one robot
    m = graph_vertices.size()-1;
    temp_point.x = min_ind;
    temp_point.y = m;
    std::cout << "the start point: (" << (temp_point.x*enlarge) << " , " << temp_point.y*enlarge << ")" << std::endl;
    cv::Point2f point_center222(temp_point.x*enlarge,temp_point.y*enlarge);
    cv::circle(plot, point_center222, 10,cv::Scalar( 40, 30, 125 ) ,cv::FILLED,cv::LINE_8);
    cv::imshow("Clipper", plot);
    cv::waitKey(0);

    graph_edges.push_back(temp_point);

    // destination
    min_ind = -1; 
    min = 9999999;

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

    graph_vertices.push_back(end_point[0]); //TODO: change for more than one robot
    m = graph_vertices.size()-1;
    temp_point.x = min_ind;
    temp_point.y = m;
    graph_edges.push_back(temp_point);

    std::vector< std::vector<int> > graph;
    std::vector<int> edges;
  
    // for(int vertex = 0; vertex < graph_vertices.size(); vertex ++) {
    //   edges.clear();
    //   for(int edge = 0; edge < graph_edges.size(); edge ++) {
    //     if(graph_edges[edge].x == vertex) {
    //       edges.push_back(graph_edges[edge].y);
    //     }
    //     else if(graph_edges[edge].y == vertex){
    //       edges.push_back(graph_edges[edge].x);
    //     }
    //   }
    //   graph.push_back(edges); 
    // }

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

    std::vector<int> my_path;
    my_path = bfs(graph, graph_vertices.size()-2, graph_vertices.size()-1);
    
    cout << endl;
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
    cout << endl;

    cout <<"PATH: "<< endl; 
    for(int i = 0; i < my_path.size(); i++){
      cout << my_path[i];
      if(i != my_path.size() - 1) { cout << ", "; }
    }
    cout << endl;
    

    for (int node = 0; node < my_path.size(); node ++){
      path[0].points.emplace_back(0, graph_vertices[my_path[node]].x, graph_vertices[my_path[node]].y, 0, 0);
    }


    //drawing the cell decomposition
    std::vector< std::vector<POINT> > AB;
    AB.reserve( quad_cells.size() + tri_cells.size() ); // preallocate memory
    AB.insert( AB.end(), quad_cells.begin(), quad_cells.end() );
    AB.insert( AB.end(), tri_cells.begin(), tri_cells.end() );
    std::vector< std::vector<POINT> > ABC;
    if(true){
    ABC.reserve( quad_cells.size() );
    ABC.insert( ABC.end(), quad_cells.begin(), quad_cells.end() );
    }
    else{
    ABC.reserve( AB.size() + other_cells.size() ); // preallocate memory
    ABC.insert( ABC.end(), AB.begin(), AB.end() );
    ABC.insert( ABC.end(), other_cells.begin(), other_cells.end() );
    }
    int output1 = 0;
    int output2 = 0;
    int output3 = 0;
    // printing the cells
    for (unsigned i=0; i<ABC.size(); i++) {
      output1 = 0 + (rand() % static_cast<int>(205 - 0 + 1));
      output2 = 0 + (rand() % static_cast<int>(205 - 0 + 1));
      output3 = 0 + (rand() % static_cast<int>(205 - 0 + 1));
      auto color_rand = cv::Scalar(output1,output2,output3);
      for(unsigned j=1; j<ABC[i].size(); j++){
        cv::Point2f point_center(ABC[i][j-1].x*enlarge,ABC[i][j-1].y*enlarge);
        // std::cout << ABC[i][j].x << ' ' << ABC[i][j].y << std::endl;
        cv::circle(plot, point_center, 1,cv::Scalar( 40, 30, 125 ),cv::FILLED,cv::LINE_8);
        cv::line(plot, cv::Point2f(ABC[i][j-1].x*enlarge,ABC[i][j-1].y*enlarge), cv::Point2f(ABC[i][j].x*enlarge,ABC[i][j].y*enlarge), color_rand, 2);
        if (j == cells[cells.size()-1].size() -1){
          cv::line(plot, cv::Point2f(ABC[i][j].x*enlarge,ABC[i][j].y*enlarge), cv::Point2f(ABC[i][0].x*enlarge,ABC[i][0].y*enlarge), color_rand, 2);
        }
      }
      
    }
    cv::imshow("Clipper", plot);
    cv::waitKey(0);  
    //drawing the points
    // for (unsigned i=0; i<graph_vertices.size(); i++) {
    //   int output7 = 0 + (rand() % static_cast<int>(100 - 0 + 1));
    //   int output8 = 0 + (rand() % static_cast<int>(100 - 0 + 1));
    //   int output9 = 0 + (rand() % static_cast<int>(100 - 0 + 1));
    //   auto color_rand = cv::Scalar(output7,output8,output9);
    //   cv::Point2f centerCircle(graph_vertices[i].x*enlarge,graph_vertices[i].y*enlarge);
    //   cv::circle(plot, centerCircle, 2,cv::Scalar( 0, 0, 255 ),cv::FILLED,cv::LINE_8);
    //   std::string text = std::to_string(i);
    //   putText(plot, text, centerCircle, cv::FONT_HERSHEY_PLAIN, 1,  color_rand, 2);
    // }

    //draw start and end point
    cv::Point2f centerCircle11(start_point[0].x*enlarge,start_point[0].y*enlarge);
    cv::circle(plot, centerCircle11, 2,cv::Scalar( 0, 0, 0 ),cv::FILLED,cv::LINE_8);
    std::string text1 = "start_point";
    putText(plot, text1, centerCircle11, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));
    cv::Point2f centerCircle111(end_point[0].x*enlarge,end_point[0].y*enlarge);
    cv::circle(plot, centerCircle111, 2,cv::Scalar( 0, 0, 0 ),cv::FILLED,cv::LINE_8);
    std::string text2 = "end_point";
    putText(plot, text2, centerCircle111, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));

    //drawing the map_lines
    for (unsigned i=0; i<graph.size(); i++) {
      for(unsigned j=0; j<graph[i].size(); j++){
        cv::line(plot, cv::Point2f(graph_vertices[i].x*enlarge,graph_vertices[i].y*enlarge), cv::Point2f(graph_vertices[graph[i][j]].x*enlarge,graph_vertices[graph[i][j]].y*enlarge), cv::Scalar(255,0,0), 1);
      }
    }
    cv::imshow("Clipper", plot);
    cv::waitKey(0);  
    //drawing the path_lines
    for (unsigned i=1; i<my_path.size(); i++) {
      cv::line(plot, cv::Point2f(graph_vertices[my_path[i-1]].x*enlarge,graph_vertices[my_path[i-1]].y*enlarge), cv::Point2f(graph_vertices[my_path[i]].x*enlarge,graph_vertices[my_path[i]].y*enlarge), cv::Scalar(50,255,0), 2);
    }

    //print the path and inflated obsticles

    // cv::flip(plot, plot, 1);
    cv::imshow("Clipper", plot);
    cv::waitKey(0);    
    // --------------------------------------------


    // Test dubins --------------------------------
    float Kmax = 5.0;

    // A fake path from roadmap
    robotPos pos0 = {x[0], y[0], theta[0]};
    robotPos pos1 = {0.8, 0.6, -M_PI/6};
    robotPos pos2 = {1.3, 0.96, 0};

    std::vector<robotPos> rmPos = {pos0, pos1, pos2};

    // Compute the dubins path between two adjacent points
    for (auto it0 = rmPos.begin(), it1 = std::next(rmPos.begin());
         it0 != std::prev(rmPos.end()) && it1 != rmPos.end(); ++it0, ++it1)
    {
      shortestDubinsResult result = dubinsShortestPath(*it0, *it1, Kmax);

      if (result.pidx > -1){
        for (auto it = result.dubinsWPList.begin(); it != result.dubinsWPList.end(); ++it){
          path[0].points.emplace_back((*it).s, (*it).pos.x, (*it).pos.y, (*it).pos.th, (*it).k);
        }
      }
    }

    // --------------------------------------------

    return true;
    //throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );
  }
}

