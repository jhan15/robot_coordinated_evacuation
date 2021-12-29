#include "inflate_objects.hpp"

const double INT_ROUND = 600.;
// for ploting the solution
int l = 1000;
cv::Mat plot(l ,l, CV_8UC3, cv::Scalar(255,255,255));

/*
takes the obsticales in the arena and inflates them to account for the size of the robot
outputs the inflated obsticales
*/
std::vector<Polygon> inflate_obstacles(const std::vector<Polygon>& obstacle_list){
    std::vector<Polygon> inflated_obsticale_list;
    const int inflate = 10;
    int px, py;

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

        // change the clipper object to a polygon vector format
        for(const ClipperLib::Path &path : ClibMergedObs){
            Polygon inflated_poly;
            for(const ClipperLib::IntPoint &point: path){
                double x = point.X / INT_ROUND;
                double y = point.Y / INT_ROUND;
                inflated_poly.emplace_back(x, y);
            }
            inflated_obsticale_list.emplace_back(inflated_poly);
        }

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
    }

    return inflated_obsticale_list;
}


/*
takes the boarders of the arena and inflates them inward to account for the size of the robot
outputs the inflated boarders
*/
Polygon inflate_borders(const Polygon &borders){
    Polygon inflated_borders;
    const int inflate = -10;    

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

    for(const ClipperLib::Path &path : ClibInflatedBorder){
        for(const ClipperLib::IntPoint &point: path){
            double x = point.X / INT_ROUND;
            double y = point.Y / INT_ROUND;
            inflated_borders.emplace_back(x, y);
        }
    }

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

    return inflated_borders;
}