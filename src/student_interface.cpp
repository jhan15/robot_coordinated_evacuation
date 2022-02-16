#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <iterator>
#include <string> 

#include <cmath>
#include "plot.hpp"
#include "dubins.h"
#include "collision.hpp"
#include "inflate_objects.hpp"
#include "vertical_cell_decomposition.hpp"
#include "motion_planning.hpp"

int enlarge = 600; // IF YOU CHANGE THIS CHANGE IT ALSO IN PLOT.CPP

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

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED" );  
  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, 
                          const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, 
                          const std::vector<cv::Point2f>& dest_image_points_plane, 
                          cv::Mat& plane_transf, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED" );  
  }


  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, const std::string& config_folder){
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

    cout << "printing points from path " << endl;
    for (int i = 0 ; i < path[0].size();i++){
      cout << path[0].points[i].x << " , " << path[0].points[i].x << endl;
    }
    
    //initialising the plot
    int l = 1000;        
    cv::Mat plot(l - 300,l, CV_8UC3, cv::Scalar(255,255,255));


    //OBSTACLES PREPROCESSING
    
    // inflating the obsticales and borders of the arena
    float inflate_value = 35;
    bool simplify = true;
    std::vector<Polygon> inflated_obstacle_list = inflate_obstacles(obstacle_list,inflate_value,simplify,plot);
    const Polygon inflated_borders = inflate_borders(borders,-inflate_value,plot);

    inflated_obstacle_list =  trim_obstacles(inflated_obstacle_list,inflated_borders, plot);
    inflated_obstacle_list =  merge_obstacles (inflated_obstacle_list,simplify, plot);

    // TO DO: implement a function to merge the obstacles that are over lapping
    // TO DO: delete all the vertcices outside of the borders


    //ADJUSTING THE FORMAT OF THE INPUT DATA
    
    //convert input boundary data into the data format we use 
    std::vector<POINT> boundary;
    for (const auto &position : inflated_borders) {
      boundary.push_back({position.x,position.y});
    }
    
    //number of robots spawned
    int robots_number = 0;

    //convert input start points data into the data format we use 
    std::vector<POINT> start_point;
    for (int i = 0; i < x.size(); i++) {
      start_point.push_back(POINT{x[i],y[i],theta[i]});
      //we assume that if the starting position is (0,0,0) the robot is not spawned
      //doesn't really relevant for the examined case where the robots are always 3
      if (x[i] + y[i] + theta[i] != 0) robots_number += 1;
    }

    //convert input gate position data into the data format we use
    std::vector<POINT> gate;
    //end point is the center of the gate 
    std::vector<POINT> end_point;
    
    for (int i = 0; i < gate_list.size(); i++) {
      for (const auto &position : gate_list[i]) {
        gate.push_back({position.x,position.y});
      }
      end_point.push_back(centroid(gate));
      gate.clear();
    }

    //the total number of vertices of all obstacles together
    int vertices_num = 0;

    //convert input obstacles data into the data format we use    
    std::vector< std::vector<POINT> > obstacles;
    std::vector<POINT> obstacle;
    for (int i = 0; i < inflated_obstacle_list.size(); i++) {
      for (const auto &position : inflated_obstacle_list[i]) {
        obstacle.push_back(POINT{position.x,position.y,-1,i});
        vertices_num += 1;
      }
      obstacles.push_back(obstacle);
      obstacle.clear();
    }


    //ROAD-MAP CONSTRUCTION

    //sorting obstacle vertices by their x value in increasing order
    std::vector<POINT> sorted_vertices;    
    sorted_vertices = sort_vertices(obstacles, sorted_vertices, vertices_num); 

    //adding the first point of the obstacle to the end to close the polygon
    obstacles = close_polygons(obstacles);

    //determining the limits of the vertical lines
    float y_limit_lower = min(min(boundary[0].y, boundary[1].y), min(boundary[2].y, boundary[3].y));
    float y_limit_upper = max(max(boundary[0].y, boundary[1].y), max(boundary[2].y, boundary[3].y));

    std::vector<SEGMENT> boundary_lines = get_boundary_lines(boundary);
    std::vector<POINT> end_points = offset_end_points (boundary_lines,robots_number, end_point);

    //finding the vertical lines
    std::vector< SEGMENT > open_line_segments;
    open_line_segments = find_lines(sorted_vertices, obstacles, y_limit_lower, y_limit_upper);

    //finding basic cells
    std::vector< std::vector<POINT> > cells;
    cells = find_cells(open_line_segments, sorted_vertices, obstacles);

    //merging overlaping polygons
    cells = merge_polygons(cells);

    //adding cells from boundary lines to closest obstacle
    cells = boundary_cells(boundary, cells, sorted_vertices, y_limit_lower, y_limit_upper);

    //getting the graph edges & vertices
    //these are for the common map and not robot specific
    std::vector<POINT> graph_edges_map;
    std::vector<POINT> graph_vertices_map;
    tie(graph_edges_map, graph_vertices_map) = get_graph(cells);
 
    //vectors to keep the paths for all robots
    std::vector<std::vector<int>> my_path;
    std::vector<std::vector<int>> optimized_path;
    std::vector<std::vector<int>> optimized_path_look_ahead;
    std::vector<std::vector<robotPos>> path_points;
    std::vector<POINT> graph_edges;
    std::vector<POINT> graph_vertices;
    std::vector<POINT> new_graph_vertices;
    //initializing them empty
    my_path = {{}, {}, {}};
    optimized_path = {{}, {}, {}};
    optimized_path_look_ahead = {{}, {}, {}}; 
    path_points = {{}, {}, {}};
    std::vector<std::vector<int>> *path_option;
    std::vector<POINT> *graph_option = &graph_vertices;

    // parameters for the optimize look ahead
    // for optimal path -> look_ahead = INFINITIY , gamma = 0.01;
    float look_ahead = INFINITY; // how many points ahead current point is allowed to look
    float gamma = 0.01;  // cost decrease on distance the further ahead you're looking 

    // option #1: my_path
    // option #2: optimized_path
    // option #3: optimized_path_look_ahead
    int path_choice = 3;

    switch(path_choice){
      case 1:
        path_option = &my_path;
        graph_option = &graph_vertices;
        break;
      case 2:
        path_option = &optimized_path;
        graph_option = &new_graph_vertices;
        break;
      case 3:
        path_option = &optimized_path_look_ahead;
        graph_option = &graph_vertices;
        break; 
    }
    //the graph is adjusted for each point by adding its starting point
    //then a path is calculated for each robot
    for(int robot = 0; robot < robots_number; robot ++) {
      //adding the start and end point for each robot into the graph
      tie(graph_edges, graph_vertices) = add_start_end(graph_vertices_map, graph_edges_map, start_point[robot], end_points[robot], obstacles);

      //constructing the graph
      std::vector< std::vector<int> > graph;
      graph = graph_construction(graph_vertices, graph_edges);
    
      //finding a path using breadth first search
      my_path[robot] = bfs(graph, graph_vertices.size()-2, graph_vertices.size()-1);

      //separating only the graph vertices which belong to the path for optimization purposes
      new_graph_vertices.clear();
      for(int i = 0 ; i < my_path[robot].size(); i++){
        new_graph_vertices.push_back({graph_vertices[my_path[robot][i]].x,graph_vertices[my_path[robot][i]].y});
      }

      //optimizing the graph
      std::vector< std::vector<int> >  optimized_graph;
      optimized_graph = optimize_graph(my_path[robot], new_graph_vertices, obstacles);

      //calculating the optimized path using breadth first search
      optimized_path[robot] = bfs(optimized_graph, 0, new_graph_vertices.size()-1);
      optimized_path_look_ahead[robot] = look_ahead_optimize(my_path[robot],graph_vertices,obstacles, look_ahead,gamma);
      //changing the path index to actual points for dubins
      path_points[robot] = index_to_coordinates((*path_option)[robot], *graph_option);     
      //printing and plotting the results
      cout << "RESULTS FOR ROBOT " << robot << endl;
      print_data(boundary, start_point, end_points[robot], obstacles, graph_vertices, graph, new_graph_vertices,
                 optimized_graph, my_path[robot], optimized_path[robot], path_points[robot]);
      plot_map(plot, robot+1,sorted_vertices, cells, start_point[robot], end_points[robot], graph, graph_vertices,my_path[robot],*graph_option,
               (*path_option)[robot]); 
    }    

    //Remove from here down after testing the coordinat_motion function
//     std::vector<std::vector<robotPos>> test_points = {{}, {}, {}};
//     robotPos temp_point;
//     temp_point.x = 1;
//     temp_point.y = 5;
//     test_points[0].push_back(temp_point);
//     temp_point.x = 8;
//     temp_point.y = 5;
//     test_points[0].push_back(temp_point);
// /*    temp_point.x = 5;
//     temp_point.y = 1;
//     test_points[0].push_back(temp_point);
//     temp_point.x = 9;
//     temp_point.y = 8;
//     test_points[0].push_back(temp_point);
// */
//     temp_point.x = 1;
//     temp_point.y = 3;
//     test_points[1].push_back(temp_point);
//     temp_point.x = 3;
//     temp_point.y = 3;
//     test_points[1].push_back(temp_point);
//     temp_point.x = 5;
//     temp_point.y = 1;
//     test_points[1].push_back(temp_point);
//     temp_point.x = 9;
//     temp_point.y = 5;
//     test_points[1].push_back(temp_point);
// /*    temp_point.x = 9;
//     temp_point.y = 8;
//     test_points[1].push_back(temp_point);
// */
//     temp_point.x = 1;
//     temp_point.y = 1;
//     test_points[2].push_back(temp_point);
//     temp_point.x = 3;
//     temp_point.y = 2;
//     test_points[2].push_back(temp_point);
//     temp_point.x = 5;
//     temp_point.y = 2;
//     test_points[2].push_back(temp_point);
//     temp_point.x = 7;
//     temp_point.y = 3;
//     test_points[2].push_back(temp_point);
//     temp_point.x = 9;
//     temp_point.y = 4;
//     test_points[2].push_back(temp_point);

//     for(int i = 0; i < 3; i++){
//       for(int j = 0; j < test_points[i].size(); j++){
//         cout << "{" << test_points[i][j].x << ", " << test_points[i][j].y << "}, ";
//       }
//       cout << endl;
//     }

//     //end of removal; change test_points to path_points below

//     //adjust the paths for collision free motion
//     test_points = coordinate_motion(test_points);

    // DUBINS
    // Compute the orientations of path_points
    for(int robot = 0; robot < robots_number; robot ++) {
      path_points[robot][0].th = mod2Pi(theta[robot]);
      if (path_points[robot].size() > 2)
      {
        for (int id=1; id<path_points[robot].size(); id++)
        {
          if (id == path_points[robot].size()-1) path_points[robot][id].th = path_points[robot][id-1].th;
          else path_points[robot][id].th = mod2Pi(atan2(path_points[robot][id+1].y-path_points[robot][id].y,
                                                        path_points[robot][id+1].x-path_points[robot][id].x));
        }
      }
      else path_points[robot][1].th = 0;
    }

    // Concatenate obstacles and boundary as one vector for collision detection
    boundary.push_back(boundary.front());
    std::vector<std::vector<pt>> obs;
    std::vector<pt> ob;
    for (int i=0; i<obstacles.size(); i++){
      for (int j=0; j<obstacles[i].size(); j++){
        ob.push_back(pt{obstacles[i][j].x, obstacles[i][j].y});
      }
      obs.push_back(ob);
      ob.clear();
    }
    for (int j=0; j<boundary.size(); j++){
      ob.push_back(pt{boundary[j].x, boundary[j].y});
    }
    obs.push_back(ob);

    float Kmax = 25.0;

    // Compute the collision-free dubins path between two points
    // Store point-pairs w/o collision-free dubins
    std::vector<std::vector<pt>> remove;
    std::vector<pt> temp;
    std::vector<std::vector<dubinsCurve>> curves;
    std::vector<dubinsCurve> tc;
    for(int robot = 0; robot < robots_number; robot ++) {
      float count = 0;
      std::cout<<"Total points for robot "<<robot<<": "<<path_points[robot].size()<<std::endl;
      for (auto it0 = path_points[robot].begin(), it1 = std::next(path_points[robot].begin());
          it0 != std::prev(path_points[robot].end()) && it1 != path_points[robot].end(); ++it0, ++it1)
      {
        std::cout<<"---- points"<<count<<" ("<<(*it0).x<<", "<<(*it0).y<<", "<<(*it0).th<<") and ";
        std::cout<<count+1<<" ("<<(*it1).x<<", "<<(*it1).y<<", "<<(*it1).th<<")"<<std::endl;
        std::cout<<"     collision-free dubins path: ";
        shortestDubinsResult sd = dubinsPath(*it0, *it1, Kmax, obs);
        if (sd.find_dubins){
          std::cout<<"yes"<<std::endl;
          tc.push_back(sd.curve);
          for (auto it = sd.dubinsWPList.begin(); it != sd.dubinsWPList.end(); ++it){
            path[robot].points.emplace_back((*it).s, (*it).pos.x, (*it).pos.y, (*it).pos.th, (*it).k);
          }
        }
        else {
          temp.push_back(pt{count,count+1});
          std::cout<<"no"<<std::endl;
        }
        count = count + 1;
      }
      remove.push_back(temp);
      temp.clear();
      curves.push_back(tc);
      tc.clear();
    }
    // bool aac = checkTwoDubins(curves[0][2], curves[1][1], Kmax);
    // std::cout<<"dubins curves collision: "<<aac<<std::endl;
    plot_dubins(plot,path,robots_number);

    // END OF DUBBINS PATH



    //close the plot on key press
    // cv::waitKey(0);    
    // cv::destroyAllWindows();

    return true;
  }
}