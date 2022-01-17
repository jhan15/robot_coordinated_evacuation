#include "inflate_objects.hpp"
#include "vertical_cell_decomposition.hpp"

const double enlarge = 600.;
// for ploting the solution
// int l = 1000;
// cv::Mat plot(l ,l, CV_8UC3, cv::Scalar(255,255,255));

/*
takes the obsticales in the arena and inflates them to account for the size of the robot
outputs the inflated obsticales
*/
std::vector<Polygon> inflate_obstacles(const std::vector<Polygon>& obstacle_list, float inflate_value, cv::Mat plot){
    std::vector<Polygon> new_obsticale_list = obstacle_list;
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
        int counter = 0;
        for(const ClipperLib::Path &path : clib_merged_obs){
            Polygon inflated_poly;
            for(const ClipperLib::IntPoint &point: path){
                double x = point.X / enlarge;
                double y = point.Y / enlarge;
                // std::cout << "--polygon point: " << counter << " ( " << x << " , " << y << ")" << std::endl;
                inflated_poly.emplace_back(x, y);
                counter ++;
            }
            inflated_obsticale_list.emplace_back(inflated_poly);
        }

        for(int i=0; i<new_obsticale_list.size();i++){
            if(new_obsticale_list[i][0].x != new_obsticale_list[i].back().x || new_obsticale_list[i][0].y != new_obsticale_list[i].back().y){
                new_obsticale_list[i].push_back(new_obsticale_list[i][0]);
            }
            for(int j=1 ; j< new_obsticale_list[i].size();j++){
                cv::line(plot, cv::Point2f(new_obsticale_list[i][j-1].x*enlarge,new_obsticale_list[i][j-1].y*enlarge), cv::Point2f(new_obsticale_list[i][j].x*enlarge,new_obsticale_list[i][j].y*enlarge), cv::Scalar(255,0,0), 1);
            }
        }
        for(int i=0; i<inflated_obsticale_list.size();i++){
            if(inflated_obsticale_list[i][0].x != inflated_obsticale_list[i].back().x || inflated_obsticale_list[i][0].y != inflated_obsticale_list[i].back().y){
                inflated_obsticale_list[i].push_back(inflated_obsticale_list[i][0]);
            }
            for(int j=1 ; j< inflated_obsticale_list[i].size();j++){
                cv::line(plot, cv::Point2f(inflated_obsticale_list[i][j-1].x*enlarge,inflated_obsticale_list[i][j-1].y*enlarge), cv::Point2f(inflated_obsticale_list[i][j].x*enlarge,inflated_obsticale_list[i][j].y*enlarge), cv::Scalar(255,255,0), 1);
            
            }
        }
        // draw solution
        // for (unsigned j=1; j<clib_obsticale.size(); j++) {
        //     cv::line(plot, cv::Point2f(clib_obsticale.at(j-1).X,clib_obsticale.at(j-1).Y), cv::Point2f(clib_obsticale.at(j).X,clib_obsticale.at(j).Y), cv::Scalar(255,0,0), 1);
        // }
        // cv::line(plot, cv::Point2f(clib_obsticale.at(clib_obsticale.size()-1).X,clib_obsticale.at(clib_obsticale.size()-1).Y), cv::Point2f(clib_obsticale.at(0).X,clib_obsticale.at(0).Y), cv::Scalar(255,0,0), 1);

        // for (unsigned i=0; i<clib_merged_obs.size(); i++) {
        //     ClipperLib::Path path = clib_merged_obs.at(i);
        //     for (unsigned j=1; j<path.size(); j++) {
        //         cv::line(plot, cv::Point2f(path.at(j-1).X,path.at(j-1).Y), cv::Point2f(path.at(j).X,path.at(j).Y), cv::Scalar(255,255,0), 2);
        //     }

        //     cv::line(plot, cv::Point2f(path.at(path.size()-1).X,path.at(path.size()-1).Y), cv::Point2f(path.at(0).X,path.at(0).Y), cv::Scalar(255,255,0), 2);

        // }
    }

    return inflated_obsticale_list;
}


/*
takes the boarders of the arena and inflates them inward to account for the size of the robot
outputs the inflated boarders
*/
Polygon inflate_borders(const Polygon &borders, float inflate_value, cv::Mat plot){
    Polygon inflated_borders;
    Polygon inflated_borders_sorted(4);
    const float inflate = -10;    

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

    float biggest_x = -1;
    float biggest_y = -1;
    float smallest_x = 9999999;
    float smallest_y = 9999999;
    for (Point pt: inflated_borders){
        if (pt.y > biggest_y){biggest_y = pt.y;}
        if (pt.y < smallest_y){smallest_y = pt.y;}
        if (pt.x > biggest_x){biggest_x = pt.x;}
        if (pt.x < smallest_x){smallest_x = pt.x;}
    }
    for (Point pt: inflated_borders){
        if(pt.x == smallest_x && pt.y == smallest_y){inflated_borders_sorted[0]=pt;}
        if(pt.x == biggest_x && pt.y == smallest_y){inflated_borders_sorted[1]=pt;}
        if(pt.x == biggest_x && pt.y == biggest_y){inflated_borders_sorted[2]=pt;}
        if(pt.x == smallest_x && pt.y == biggest_y){inflated_borders_sorted[3]=pt;}
    }


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
    // cv::flip(plot, plot, 1);
    // cv::imshow("Clipper", plot);
    // cv::waitKey(0);    

    return inflated_borders_sorted;
}

std::vector<Polygon> merge_obstacles(const std::vector<Polygon>& obstacle_list,const Polygon &borders, cv::Mat plot){
    std::vector<Polygon> new_obstacles;
    Polygon new_borders = borders;
    Polygon new_obstacle;
    SEGMENT obs_segment;
    SEGMENT border_segment;
    POINT intersection_pt;
    bool out_of_border = false;
    bool inter_check = false;
    int tracker = 0;
    int compensator = 0;
    std::vector<int> to_delete;

    float y_limit_lower = min(min(borders[0].y, borders[1].y), min(borders[2].y, borders[3].y));
    float y_limit_upper = max(max(borders[0].y, borders[1].y), max(borders[2].y, borders[3].y));
    float x_limit_lower = min(min(borders[0].x, borders[1].x), min(borders[2].x, borders[3].x));
    float x_limit_upper = max(max(borders[0].x, borders[1].x), max(borders[2].x, borders[3].x));

    if(new_borders[0].x != new_borders.back().x || new_borders[0].y != new_borders.back().y){
        new_borders.push_back(new_borders[0]);
    }
    for (int i = 0 ; i < new_borders.size(); i ++){
        std::cout << "border # " << i << " ( " << new_borders[i].x*enlarge << " , " << new_borders[i].y*enlarge << " ) " << std::endl;
    }
    // remove the parts of the obstacles that fall outside of the borders
    for(Polygon obstacle : obstacle_list){
        if(obstacle[0].x != obstacle.back().x || obstacle[0].y != obstacle.back().y){
            obstacle.push_back(obstacle[0]);
        }
        new_obstacle = obstacle;
        compensator = 0;
        
        for(int obs_pt = 0 ; obs_pt < obstacle.size()-1;obs_pt++){
            inter_check = false;
            out_of_border = obstacle[obs_pt].x > x_limit_upper || obstacle[obs_pt].x < x_limit_lower
            || obstacle[obs_pt].y > y_limit_upper || obstacle[obs_pt].y < y_limit_lower;
            if(out_of_border){
                obs_segment.a = {obstacle[obs_pt].x,obstacle[obs_pt].y};
                obs_segment.b = {obstacle[obs_pt+1].x,obstacle[obs_pt+1].y};
                for(int border_pt = 0 ; border_pt < new_borders.size()-1 ; border_pt++){
                    border_segment.a = {new_borders[border_pt].x,new_borders[border_pt].y};
                    border_segment.b = {new_borders[border_pt+1].x,new_borders[border_pt+1].y};
                    intersection_pt = segment_intersection(obs_segment,border_segment,true);
                    int output7 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
                    int output8 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
                    int output9 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
                    auto color_rand = cv::Scalar(output7,output8,output9);
                    cv::Point2f centerCircle(obstacle[obs_pt].x*enlarge,obstacle[obs_pt].y*enlarge);
                    cv::circle(plot, centerCircle, 2,cv::Scalar( 0, 0, 0 ),cv::FILLED,cv::LINE_8);
                    cv::line(plot, cv::Point2f(border_segment.a.x*enlarge,border_segment.a.y*enlarge), cv::Point2f(border_segment.b.x*enlarge,border_segment.b.y*enlarge), color_rand, 2);
                    cv::line(plot, cv::Point2f(obs_segment.a.x*enlarge,obs_segment.a.y*enlarge), cv::Point2f(obs_segment.b.x*enlarge,obs_segment.b.y*enlarge), color_rand, 2);
                    cv::imshow("Clipper", plot);
                    cv::waitKey(0);                    
                    if(intersection_pt.x == -1){
                        tracker = obs_pt;
                        if(tracker == 0){
                            tracker = obstacle.size()-1;
                        }
                        std::cout << "-- no intersection -- trying the other way" << std::endl;
                        obs_segment.b = {obstacle[tracker-1].x,obstacle[tracker-1].y};
                        intersection_pt = segment_intersection(obs_segment,border_segment,true);
                        if(intersection_pt.x == -1){
                            std::cout << "-- still no intersection -- moving on to next border" << std::endl;
                            // continue;
                        }
                    }
                    if(intersection_pt.x != -1){
                        std::cout << "moving point : " << obstacle[obs_pt].x*enlarge << " , " << obstacle[obs_pt].y * enlarge<< " ) to ( " << intersection_pt.x * enlarge<< " , " << intersection_pt.y * enlarge << " )" << std::endl;
                        obstacle[obs_pt] = {intersection_pt.x,intersection_pt.y};
                        new_obstacle[obs_pt-compensator] = obstacle[obs_pt];
                        inter_check = true;
                        // new_obstacle.push_back(obstacle[obs_pt]);
                    }

                }
            }
            if (out_of_border && !inter_check){

                std::cout << "erasing pt: (" << obstacle[obs_pt].x*enlarge << " , " << obstacle[obs_pt].y*enlarge << " ) and compensator is: " << compensator << std::endl;
                cv::Point2f centerCircle(obstacle[obs_pt].x*enlarge,obstacle[obs_pt].y*enlarge);
                cv::circle(plot, centerCircle, 3,cv::Scalar( 0, 0, 255 ),cv::FILLED,cv::LINE_8);
                new_obstacle.erase(new_obstacle.begin()+obs_pt-compensator);
                compensator++;                
            }
            else{
                new_obstacle.push_back(obstacle[obs_pt]);
                std::cout << " --------next obstacle --------- " << std::endl;
            }
        }
        new_obstacles.push_back(new_obstacle);
    }
    return new_obstacles;
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