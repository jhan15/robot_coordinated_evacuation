#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
  
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
    std::cout<<"x="<<x<<" y="<<y<<" theta="<<theta<<std::endl;


    // You can test the roadmap here --------------



    // --------------------------------------------
    

    // A fake path
    float xc = 0, yc = 1.5, r = 1.4;
    float ds = 0.05;
    for (float theta = -M_PI/2, s = 0; theta<(-M_PI/2 + 1.2); theta+=ds/r, s+=ds) {
      path.points.emplace_back(s, xc+r*std::cos(theta), yc+r*std::sin(theta), theta+M_PI/2, 1./r);    
    }

    return true; 
  }
}

