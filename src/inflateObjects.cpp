#include "inflateObjects.hpp"

std::vector<Polygon> inflateObstacles(const std::vector<Polygon>& obstacle_list){
    std::vector<Polygon> inflatedObsticales;
    const double INT_ROUND = 1000.;
    const int inflate = 10;
    int l = 1200;
    cv::Mat plot(l ,l, CV_8UC3, cv::Scalar(255,255,255));
    int px, py;

    // ClipperLib::Path subj;
    // ClipperLib::Paths solution;

    for (auto &obstacle : obstacle_list) {
        ClipperLib::Path ClibObsticale;
        ClipperLib::Paths ClibMergedObs;

        // extract obstacle to a clipper object
        for (const auto &point : obstacle) {
            std::cout << "Obsticale points x: " << point.x << ", y: " << point.y << std::endl;
            ClibObsticale << ClipperLib::IntPoint(point.x * INT_ROUND, point.y * INT_ROUND);
        }

        // applying the offset
        ClipperLib::ClipperOffset co;
        co.AddPath(ClibObsticale, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        co.Execute(ClibMergedObs, inflate);
        printf("subject size = %d\n",(int)ClibObsticale.size());
        // print results
        printf("solution size = %d\n",(int)ClibMergedObs.size());

        // draw solution
        for (unsigned j=1; j<ClibObsticale.size(); j++) {
            cv::line(plot, cv::Point2f(ClibObsticale.at(j-1).X,ClibObsticale.at(j-1).Y), cv::Point2f(ClibObsticale.at(j).X,ClibObsticale.at(j).Y), cv::Scalar(255,0,0), 1);
        }
        cv::line(plot, cv::Point2f(ClibObsticale.at(ClibObsticale.size()-1).X,ClibObsticale.at(ClibObsticale.size()-1).Y), cv::Point2f(ClibObsticale.at(0).X,ClibObsticale.at(0).Y), cv::Scalar(255,0,0), 1);

        for (unsigned i=0; i<ClibMergedObs.size(); i++) {
            ClipperLib::Path path = ClibMergedObs.at(i);
            for (unsigned j=1; j<path.size(); j++) {
                cv::line(plot, cv::Point2f(path.at(j-1).X,path.at(j-1).Y), cv::Point2f(path.at(j).X,path.at(j).Y), cv::Scalar(255,255,0), 2);
            }
            cv::line(plot, cv::Point2f(path.at(path.size()-1).X,path.at(path.size()-1).Y), cv::Point2f(path.at(0).X,path.at(0).Y), cv::Scalar(255,255,0), 2);

        }

    //     // add the inflated obstacle to a path
    //     coo.AddPaths(ClibMergedObs, ClipperLib::ptSubject, true);

    }
    cv::flip(plot, plot, 0);
    cv::imshow("Clipper", plot);
    cv::waitKey(0);    

    return inflatedObsticales;
  }