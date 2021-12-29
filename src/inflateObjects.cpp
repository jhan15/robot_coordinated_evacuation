#include "inflateObjects.hpp"

const double INT_ROUND = 600.;
/*
takes the obsticales in the arena and inflates them to account for the size of the robot
outputs the inflated obsticales
*/
std::vector<Polygon> inflateObstacles(const std::vector<Polygon>& obstacle_list, const Polygon &borders){
    std::vector<Polygon> inflatedObsticales;
    const int inflate = 10;
    int l = 1000;
    cv::Mat plot(l ,l, CV_8UC3, cv::Scalar(255,255,255));
    int px, py;

    // ClipperLib::Path subj;
    // ClipperLib::Paths solution;

    for (auto &obstacle : obstacle_list) {
        ClipperLib::Path ClibObsticale;
        ClipperLib::Paths ClibMergedObs;

        // extract obstacle to a clipper object
        for (const auto &position : obstacle) {
            std::cout << "Obsticale points x: " << position.x << ", y: " << position.y << std::endl;
            ClibObsticale << ClipperLib::IntPoint(position.x * INT_ROUND, position.y * INT_ROUND);
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
    // cv::imshow("Clipper", plot);
    // cv::waitKey(0); 
    const Polygon inflatedBorders = inflateBorders(borders, plot);


    return inflatedObsticales;
}


/*
takes the boarders of the arena and inflates them inward to account for the size of the robot
outputs the inflated boarders
*/
Polygon inflateBorders(const Polygon &borders, cv::Mat &plot){
    Polygon inflatedBorders;
    const int inflate = -10;
    // int l = 1200;
    // cv::Mat plot(l ,l, CV_8UC3, cv::Scalar(255,255,255));
    // int px, py;
    

    ClipperLib::Path ClibBorder;
    ClipperLib::Paths ClibInflatedBorder;

    // extract borders to a clipper object
    for (const auto &position : borders) {
        ClibBorder << ClipperLib::IntPoint(position.x * INT_ROUND, position.y * INT_ROUND);
    }

    // applying offset to borders 
    ClipperLib::ClipperOffset co;
    co.AddPath(ClibBorder, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
    co.Execute(ClibInflatedBorder, inflate);


    // draw solution
    for (unsigned j=1; j<ClibBorder.size(); j++) {
        cv::line(plot, cv::Point2f(ClibBorder.at(j-1).X,ClibBorder.at(j-1).Y), cv::Point2f(ClibBorder.at(j).X,ClibBorder.at(j).Y), cv::Scalar(255,0,0), 1);
    }
    cv::line(plot, cv::Point2f(ClibBorder.at(ClibBorder.size()-1).X,ClibBorder.at(ClibBorder.size()-1).Y), cv::Point2f(ClibBorder.at(0).X,ClibBorder.at(0).Y), cv::Scalar(255,0,0), 1);

    for (unsigned i=0; i<ClibInflatedBorder.size(); i++) {
        ClipperLib::Path path = ClibInflatedBorder.at(i);
        for (unsigned j=1; j<path.size(); j++) {
            cv::line(plot, cv::Point2f(path.at(j-1).X,path.at(j-1).Y), cv::Point2f(path.at(j).X,path.at(j).Y), cv::Scalar(255,255,0), 2);
        }
        cv::line(plot, cv::Point2f(path.at(path.size()-1).X,path.at(path.size()-1).Y), cv::Point2f(path.at(0).X,path.at(0).Y), cv::Scalar(255,255,0), 2);

    }
    cv::flip(plot, plot, 1);
    cv::imshow("Clipper", plot);
    cv::waitKey(0);    

    return inflatedBorders;
}