#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <cmath>
#include "dubins/dubins.h"

  
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

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list,
                const std::vector<Polygon>& gate_list,
                const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta,
                std::vector<Path>& path, const std::string& config_folder){

    //throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );
    
    // Here are the positions of border, gate, obstacles, and robot 
    std::cout<<"\n>>>> Border postion:"<<std::endl;
    for (const auto &position : borders)
    {
      std::cout<<"x="<<position.x<<" y="<<position.y<<std::endl;
    }

    std::cout<<"\n>>>> Obstacle postion:"<<std::endl;
    for (auto &obstacle : obstacle_list)
    {
      std::cout<<"Obstacle:"<<std::endl;
      for (const auto &position : obstacle)
      {
        std::cout<<"x="<<position.x<<" y="<<position.y<<std::endl;
      }
    }

    std::cout<<"\n>>>> Gate postion:"<<std::endl;
    for (auto &gate : gate_list)
    {
      std::cout<<"Gate:"<<std::endl;
      for (const auto &position : gate)
      {
        std::cout<<"x="<<position.x<<" y="<<position.y<<std::endl;
      }
    }

    std::cout<<"\n>>>> Robot initial postion:"<<std::endl;
    for (int i=0; i<x.size(); i++)
    {
      std::cout<<"x="<<x[i]<<" y="<<y[i]<<" theta="<<theta[i]<<std::endl;
    }


    // You can test the roadmap here --------------



    // --------------------------------------------
    


    // Test dubins --------------------------------

    // A fake path from roadmap
    

    robotPos pos0, pos1;
    pos0.x = 0.0;
    pos0.y = 0.0;
    pos0.th = 0;
    pos1.x = 4.0;
    pos1.y = 0.0;
    pos1.th = 0;
    float Kmax = 1.0;

    shortestDubinsResult result = dubinsShortestPath(pos0, pos1, Kmax, true);


    // --------------------------------------------
    

    // A fake path
    float ds = 0.05;

    // fake path for 0 and 1
    for (float l=0, s=0; l<3; l++, s+=ds) {
      path[0].points.emplace_back(s, x[0]+ds*l, y[0], 0.0, 0.0);
    }
    for (float l=0, s=0; l<10; l++, s+=ds) {
      path[1].points.emplace_back(s, x[1]+ds*l, y[1], 0.0, 0.0);
    }
    // no path for 2

    return true; 
  }
}

