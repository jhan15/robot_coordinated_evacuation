#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <iterator>
#include <string> 

#include <cmath>
#include "plot.hpp"
#include "dubins.h"
#include "inflate_objects.hpp"
#include "vertical_cell_decomposition.hpp"

void plot_map (cv::Mat plot,int robot_num, std::vector<POINT> sorted_vertices, std::vector<std::vector<POINT>> cells, POINT start_point, std::vector<POINT> end_point, std::vector<std::vector<int>> graph, std::vector<POINT> graph_vertices,std::vector<int> my_path, std::vector<POINT> graph_chosen, std::vector<int> path){

    int enlarge = 600;
    float robot = 0.8 * robot_num;

    if(robot_num==1){
      //FOR DEBUGGING
      // std::cout<< "vertices number: " << vertices_num << std::endl;
      // std::cout<<"\n>>>> Sorted vertices:"<<std::endl;
      //print out the vertices of the obstacles
      for(int i = 0; i < sorted_vertices.size(); i++){
          // cout<< "x=" << sorted_vertices[i].x << " y=" << sorted_vertices[i].y << endl;
          cv::Point2f centerCircle(sorted_vertices[i].x*enlarge,sorted_vertices[i].y*enlarge);
          cv::circle(plot, centerCircle, 2,cv::Scalar( 40, 30, 125 ),cv::FILLED,cv::LINE_8);
          // std::string text = std::to_string(sorted_vertices[i].obs);
          // putText(plot, text, centerCircle, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));
      }
      //cv::imshow("Clipper", plot);
      //cv::waitKey(0); 

      // FOR DEBUG
      // std::cout << "the start point: (" << (temp_point.x*enlarge) << " , " << temp_point.y*enlarge << ")" << std::endl;
      // cv::Point2f point_center222(temp_point.x*enlarge,temp_point.y*enlarge);
      // cv::circle(plot, point_center222, 10,cv::Scalar( 40, 30, 125 ) ,cv::FILLED,cv::LINE_8);
      // cv::imshow("Clipper", plot);
      // cv::waitKey(0);

          // FOR DEBUG
      // drawing the cell decomposition
      std::vector< std::vector<POINT> > AB;
      AB.reserve( cells.size()/* + tri_cells.size() */); // preallocate memory
      AB.insert( AB.end(), cells.begin(), cells.end() );
      //AB.insert( AB.end(), tri_cells.begin(), tri_cells.end() );
      std::vector< std::vector<POINT> > ABC;
      if(true){
      ABC.reserve( cells.size() );
      ABC.insert( ABC.end(), cells.begin(), cells.end() );
      }/*
      else{
      ABC.reserve( AB.size() + other_cells.size() ); // preallocate memory
      ABC.insert( ABC.end(), AB.begin(), AB.end() );
      ABC.insert( ABC.end(), other_cells.begin(), other_cells.end() );
      }*/
      // printing the cells
      for (unsigned i=0; i<ABC.size(); i++) {
        int output1 = 0 + (rand() % static_cast<int>(205 - 0 + 1));
        int output2 = 0 + (rand() % static_cast<int>(205 - 0 + 1));
        int output3 = 0 + (rand() % static_cast<int>(205 - 0 + 1));
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
      for (unsigned i=0; i<graph_vertices.size(); i++) {
        int output7 = 0 + (rand() % static_cast<int>(100 - 0 + 1));
        int output8 = 0 + (rand() % static_cast<int>(100 - 0 + 1));
        int output9 = 0 + (rand() % static_cast<int>(100 - 0 + 1));
        auto color_rand = cv::Scalar(output7,output8,output9);
        cv::Point2f centerCircle(graph_vertices[i].x*enlarge,graph_vertices[i].y*enlarge);
        cv::circle(plot, centerCircle, 2,cv::Scalar( 0, 0, 255 ),cv::FILLED,cv::LINE_8);
        // std::string text = std::to_string(i);
        // putText(plot, text, centerCircle, cv::FONT_HERSHEY_PLAIN, 1,  color_rand, 2);
      }
    }
        // printing graph edges
    // for (unsigned i=0; i<graph_edges.size(); i++) {
    //   int output7 = 0 + (rand() % static_cast<int>(100 - 0 + 1));
    //   int output8 = 0 + (rand() % static_cast<int>(100 - 0 + 1));
    //   int output9 = 0 + (rand() % static_cast<int>(100 - 0 + 1));
    //   std::cout << "graph edge: #" << i << " ( " << (graph_edges[i].x*enlarge) << " , " << (graph_edges[i].y*enlarge) << " )" <<endl;
    //   auto color_rand = cv::Scalar(output7,output8,output9);
    //   cv::Point2f centerCircle(graph_edges[i].x*enlarge,graph_edges[i].y*enlarge);
    //   cv::circle(plot, centerCircle, 2,cv::Scalar( 0, 0, 255 ),cv::FILLED,cv::LINE_8);
    //   std::string text = std::to_string(i);
    //   putText(plot, text, centerCircle, cv::FONT_HERSHEY_PLAIN, 1,  color_rand, 2);
    //       cv::imshow("Clipper", plot);
    // cv::waitKey(0);   
    // }

    //draw start and end point
    cv::Point2f centerCircle11(start_point.x*enlarge,start_point.y*enlarge);
    cv::circle(plot, centerCircle11, 2,cv::Scalar( 0, 0, 0 ),cv::FILLED,cv::LINE_8);
    std::string text1 = "start_point";
    putText(plot, text1, centerCircle11, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255/robot_num,255));
    cv::Point2f centerCircle111(end_point[0].x*enlarge,end_point[0].y*enlarge);
    cv::circle(plot, centerCircle111, 2,cv::Scalar( 0, 0, 0 ),cv::FILLED,cv::LINE_8);
    std::string text2 = "end_point";
    putText(plot, text2, centerCircle111, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));

    //drawing the map_lines [graph]
    for (unsigned i=0; i<graph.size(); i++) {
      for(unsigned j=0; j<graph[i].size(); j++){
        cv::line(plot, cv::Point2f(graph_vertices[i].x*enlarge,graph_vertices[i].y*enlarge), cv::Point2f(graph_vertices[graph[i][j]].x*enlarge,graph_vertices[graph[i][j]].y*enlarge), cv::Scalar(255,0,0), 1);
    // cv::imshow("Clipper", plot);
    // cv::waitKey(0); 
      }
    }
    cv::imshow("Clipper", plot);
    cv::waitKey(0);  
    //drawing the original path_lines


    for (unsigned i=1; i<my_path.size(); i++) {
      cv::line(plot, cv::Point2f(graph_vertices[my_path[i-1]].x*enlarge,graph_vertices[my_path[i-1]].y*enlarge), cv::Point2f(graph_vertices[my_path[i]].x*enlarge,graph_vertices[my_path[i]].y*enlarge), cv::Scalar(0,20,255), 2);
    }

    for (unsigned i=1; i<path.size(); i++) {
      cv::line(plot, cv::Point2f(graph_chosen[path[i-1]].x*enlarge,graph_chosen[path[i-1]].y*enlarge), cv::Point2f(graph_chosen[path[i]].x*enlarge,graph_chosen[path[i]].y*enlarge), cv::Scalar(50,255,0), 2);
    }
    //drawing the optimzed_graph
    // for(int i = 0; i < optimized_graph.size(); i++){
    //   cout << "i: " << i << endl;
    //   for(int j = 0; j < optimized_graph[i].size(); j++) {  
    //     cout << "j: " << j << endl;
    //     cout << "1.x: " << new_graph_vertices[i].x;
    //     cout << " 1.y: " << new_graph_vertices[i].x<<endl;
    //     cout << "2.x: " << new_graph_vertices[optimized_graph[i][j]].x;
    //     cout << " 2.y: " << new_graph_vertices[optimized_graph[i][j]].y<<endl;
    //     cv::line(plot, cv::Point2f(new_graph_vertices[i].x*enlarge,new_graph_vertices[i].y*enlarge), cv::Point2f(new_graph_vertices[optimized_graph[i][j]].x*enlarge,new_graph_vertices[optimized_graph[i][j]].y*enlarge), cv::Scalar(255,0,0), 1);

    //   }
    // }



    //drawing the optimized path_lines
    // for (unsigned i=1; i<optimized_path.size(); i++) {
    //   cv::line(plot, cv::Point2f(new_graph_vertices[optimized_path[i-1]].x*enlarge,new_graph_vertices[optimized_path[i-1]].y*enlarge), cv::Point2f(new_graph_vertices[optimized_path[i]].x*enlarge,new_graph_vertices[optimized_path[i]].y*enlarge), cv::Scalar(0,20,255), 2);
    // }
    cv::imshow("Clipper", plot);

}