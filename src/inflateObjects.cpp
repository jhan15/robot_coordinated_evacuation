#include <iostream>
#include <string>
#include <vector>
#include "utils.hpp"
#include "clipper/clipper.hpp"

std::vector<Polygon> inflateObstacles(const std::vector<Polygon> &obstacles);
Polygon inflateBorders(const Polygon &borders);

std::vector<Polygon> inflateObstacles(const std::vector<Polygon>& obstacle_list){
    std::vector<Polygon> inflatedObsticales;
    int l = 600;
    cv::Mat plot(l ,l, CV_8UC3, cv::Scalar(255,255,255));
    int px, py;

    ClipperLib::Path subj;
    ClipperLib::Paths solution;

    subj << 
      ClipperLib::IntPoint(348,257) << ClipperLib::IntPoint(364,148) << ClipperLib::IntPoint(362,148) << 
      ClipperLib::IntPoint(326,241) << ClipperLib::IntPoint(295,219) << ClipperLib::IntPoint(258,88) << 
      ClipperLib::IntPoint(440,129) << ClipperLib::IntPoint(370,196) << ClipperLib::IntPoint(372,275) << ClipperLib::IntPoint(348,257);

    ClipperLib::ClipperOffset co;
    co.AddPath(subj, ClipperLib::JoinType::jtRound , ClipperLib::EndType::etClosedPolygon); // etClosedPolygon, etClosedLine // jtSquare , jtRound
    co.Execute(solution, 10.0);
    
    printf("subject size = %d\n",(int)subj.size());
    // print results
    printf("solution size = %d\n",(int)solution.size());

    // draw solution
    for (unsigned j=1; j<subj.size(); j++) {
        cv::line(plot, cv::Point2f(subj.at(j-1).X,subj.at(j-1).Y), cv::Point2f(subj.at(j).X,subj.at(j).Y), cv::Scalar(255,0,0), 1);
    }
    for (unsigned i=0; i<solution.size(); i++) {
        ClipperLib::Path path = solution.at(i);
        for (unsigned j=1; j<path.size(); j++) {
            cv::line(plot, cv::Point2f(path.at(j-1).X,path.at(j-1).Y), cv::Point2f(path.at(j).X,path.at(j).Y), cv::Scalar(255,255,0), 2);
        }
    }
    cv::flip(plot, plot, 0);
    cv::imshow("Clipper", plot);
    cv::waitKey(0);
    return inflatedObsticales;
  }