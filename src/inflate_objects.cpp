#include "inflate_objects.hpp"

const double enlarge = 600.;
// for ploting the solution
int l = 1000;
cv::Mat plot(l ,l, CV_8UC3, cv::Scalar(255,255,255));

/*
takes the obsticales in the arena and inflates them to account for the size of the robot
outputs the inflated obsticales
*/
std::vector<Polygon> inflate_obstacles(const std::vector<Polygon>& obstacle_list, int inflate_value){
    std::vector<Polygon> inflated_obsticale_list;
    int px, py;

    for (auto &obstacle : obstacle_list) {
        ClipperLib::Path clib_obsticale;
        ClipperLib::Paths clib_merged_obs;

        // extract obstacle to a clipper object
        for (const auto &position : obstacle) {
            // std::cout << "Obsticale points x: " << position.x << ", y: " << position.y << std::endl;
            clib_obsticale << ClipperLib::IntPoint(position.x * enlarge, position.y * enlarge);
        }

        // applying the offset
        ClipperLib::ClipperOffset co;
        co.AddPath(clib_obsticale, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        co.Execute(clib_merged_obs, inflate_value);
        // printf("subject size = %d\n",(int)clib_obsticale.size());
        // print results
        // printf("solution size = %d\n",(int)clib_merged_obs.size());

        // change the clipper object to a polygon vector format
        for(const ClipperLib::Path &path : clib_merged_obs){
            Polygon inflated_poly;
            for(const ClipperLib::IntPoint &point: path){
                double x = point.X / enlarge;
                double y = point.Y / enlarge;
                inflated_poly.emplace_back(x, y);
            }
            inflated_obsticale_list.emplace_back(inflated_poly);
        }

        // draw solution
        for (unsigned j=1; j<clib_obsticale.size(); j++) {
            cv::line(plot, cv::Point2f(clib_obsticale.at(j-1).X,clib_obsticale.at(j-1).Y), cv::Point2f(clib_obsticale.at(j).X,clib_obsticale.at(j).Y), cv::Scalar(255,0,0), 1);
        }
        cv::line(plot, cv::Point2f(clib_obsticale.at(clib_obsticale.size()-1).X,clib_obsticale.at(clib_obsticale.size()-1).Y), cv::Point2f(clib_obsticale.at(0).X,clib_obsticale.at(0).Y), cv::Scalar(255,0,0), 1);

        for (unsigned i=0; i<clib_merged_obs.size(); i++) {
            ClipperLib::Path path = clib_merged_obs.at(i);
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
Polygon inflate_borders(const Polygon &borders, int inflate_value){
    Polygon inflated_borders;
    const int inflate = -10;    

    ClipperLib::Path clib_border;
    ClipperLib::Paths clib_inflated_border;

    // extract borders to a clipper object
    for (const auto &position : borders) {
        clib_border << ClipperLib::IntPoint(position.x * enlarge, position.y * enlarge);
    }

    // applying offset to borders 
    ClipperLib::ClipperOffset co;
    co.AddPath(clib_border, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
    co.Execute(clib_inflated_border, inflate_value);

    for(const ClipperLib::Path &path : clib_inflated_border){
        for(const ClipperLib::IntPoint &point: path){
            double x = point.X / enlarge;
            double y = point.Y / enlarge;
            inflated_borders.emplace_back(x, y);
        }
    }

    // draw solution
    for (unsigned j=1; j<clib_border.size(); j++) {
        cv::line(plot, cv::Point2f(clib_border.at(j-1).X,clib_border.at(j-1).Y), cv::Point2f(clib_border.at(j).X,clib_border.at(j).Y), cv::Scalar(255,0,0), 1);
    }
    cv::line(plot, cv::Point2f(clib_border.at(clib_border.size()-1).X,clib_border.at(clib_border.size()-1).Y), cv::Point2f(clib_border.at(0).X,clib_border.at(0).Y), cv::Scalar(255,0,0), 1);

    for (unsigned i=0; i<clib_inflated_border.size(); i++) {
        ClipperLib::Path path = clib_inflated_border.at(i);
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

    // for (unsigned i = 0; i< obstacle_list.size(); i++) {
    //   std::cout << "Obsticale #" << i << std::endl;
    //   Polygon obstacle = obstacle_list[i];
    //   Polygon obstacle_after = inflated_obstacle_list[i];
    //   for (unsigned j = 0; j< obstacle.size(); j++) {
    //       std::cout << "Obsticale points x: " << obstacle[j].x << ", y: " << obstacle[j].y << std::endl;
    //       std::cout << "Obsticale points_after x: " << obstacle_after[j].x << ", y: " << obstacle_after[j].y << std::endl;
    //   }
    // }