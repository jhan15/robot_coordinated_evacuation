#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include "clipper.hpp"

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

  bool inflateObstacles(const std::vector<Polygon>& obstacle_list){
    using namespace ClipperLib;
    int l = 600;
    cv::Mat plot(l ,l, CV_8UC3, cv::Scalar(255,255,255));
    int px, py;

    ClipperLib::Path subj;
    Paths solution;

    subj << 
      IntPoint(348,257) << IntPoint(364,148) << IntPoint(362,148) << 
      IntPoint(326,241) << IntPoint(295,219) << IntPoint(258,88) << IntPoint(440,129) << IntPoint(370,196) << IntPoint(372,275) << IntPoint(348,257);

    // ClipperOffset co;
    // co.AddPath(subj, jtRound , etClosedPolygon); // etClosedPolygon, etClosedLine // jtSquare , jtRound
    // co.Execute(solution, 10.0);
    
    // printf("subject size = %d\n",(int)subj.size());
    // // print results
    // printf("solution size = %d\n",(int)solution.size());

    // draw solution
    // for (unsigned j=1; j<subj.size(); j++) {
    //     cv::line(plot, cv::Point2f(subj.at(j-1).X,subj.at(j-1).Y), cv::Point2f(subj.at(j).X,subj.at(j).Y), cv::Scalar(255,0,0), 1);
    // }
    // for (unsigned i=0; i<solution.size(); i++) {
    //     ClipperLib::Path path = solution.at(i);
    //     for (unsigned j=1; j<path.size(); j++) {
    //         cv::line(plot, cv::Point2f(path.at(j-1).X,path.at(j-1).Y), cv::Point2f(path.at(j).X,path.at(j).Y), cv::Scalar(255,255,0), 2);
    //     }
    // }
    // cv::flip(plot, plot, 0);
    // cv::imshow("Clipper", plot);
    // cv::waitKey(0);
    return true;
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path, const std::string& config_folder){
    const bool res1 = inflateObstacles(obstacle_list);
    float xc = 0, yc = 1.5, r = 1.4;
    float ds = 0.05;
    for (float theta = -M_PI/2, s = 0; theta<(-M_PI/2 + 1.2); theta+=ds/r, s+=ds) {
      path.points.emplace_back(s, xc+r*std::cos(theta), yc+r*std::sin(theta), theta+M_PI/2, 1./r);    
    }

    return true; 
  }
}

