#include "inflate_objects.hpp"
#include "vertical_cell_decomposition.hpp"
#include <set>


const double enlarge = 600.;
// for ploting the solution
// int l = 1000;
// cv::Mat plot(l ,l, CV_8UC3, cv::Scalar(255,255,255));


void writeSvg(std::vector<polygon> const& g, std::string fname) {
    std::ofstream svg(fname);
    boost::geometry::svg_mapper<point_xy> mapper(svg, 400, 400);
    for (auto& p: g) {
        mapper.add(p);
        mapper.map(p, "fill-opacity:0.5;fill:rgb(153,0,0);stroke:rgb(200,0,0);stroke-width:2");
    }
}

void writeSvg_single(polygon const& g, std::string fname) {
    std::ofstream svg(fname);
    boost::geometry::svg_mapper<point_xy> mapper(svg, 400, 400);
    mapper.add(g);
    mapper.map(g, "fill-opacity:0.5;fill:rgb(153,0,0);stroke:rgb(200,0,0);stroke-width:2");
}


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
            // if(new_obsticale_list[i][0].x != new_obsticale_list[i].back().x || new_obsticale_list[i][0].y != new_obsticale_list[i].back().y){
            //     new_obsticale_list[i].push_back(new_obsticale_list[i][0]);
            // }
            for(int j=0 ; j< new_obsticale_list[i].size();j++){
                int nxt_ind = (j+1) % new_obsticale_list[i].size();
                cv::line(plot, cv::Point2f(new_obsticale_list[i][j].x*enlarge,new_obsticale_list[i][j].y*enlarge), cv::Point2f(new_obsticale_list[i][nxt_ind].x*enlarge,new_obsticale_list[i][nxt_ind].y*enlarge), cv::Scalar(255,0,0), 1);
            }
        }
        for(int i=0; i<inflated_obsticale_list.size();i++){
            // if(inflated_obsticale_list[i][0].x != inflated_obsticale_list[i].back().x || inflated_obsticale_list[i][0].y != inflated_obsticale_list[i].back().y){
            //     inflated_obsticale_list[i].push_back(inflated_obsticale_list[i][0]);
            // }
            for(int j=0 ; j< inflated_obsticale_list[i].size();j++){
                int nxt_ind = (j+1) % inflated_obsticale_list[i].size();
                cv::line(plot, cv::Point2f(inflated_obsticale_list[i][j].x*enlarge,inflated_obsticale_list[i][j].y*enlarge), cv::Point2f(inflated_obsticale_list[i][nxt_ind].x*enlarge,inflated_obsticale_list[i][nxt_ind].y*enlarge), cv::Scalar(255,255,0), 1);
            }
        }
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

    float biggest_x = -INFINITY;
    float biggest_y = -INFINITY;
    float smallest_x = INFINITY;
    float smallest_y = INFINITY;
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


std::vector<Polygon> trim_obstacles(const std::vector<Polygon>& obstacle_list,const Polygon &borders, cv::Mat plot){
    std::vector<Polygon> trimmed_obstacles;
    std::vector<point_xy> temp_points;
    Polygon temp_obj;
    polygon obs_boost;
    polygon border_boost;
    std::vector<polygon> output;

    // converting borders to a boost object
    for(Point curr_point: borders){
        temp_points+= point_xy(curr_point.x,curr_point.y);
    }
    boost::geometry::assign_points(border_boost, temp_points);
    correct(border_boost);
    int count = 0;
    // converting obstacles to a boost object
    for(Polygon curr_obs : obstacle_list){
        count++;
        temp_points.clear();
        for(Point curr_point : curr_obs){ 
            temp_points+= point_xy(curr_point.x,curr_point.y);
        }
        boost::geometry::assign_points(obs_boost, temp_points);
        correct(obs_boost);
        boost::geometry::intersection(obs_boost, border_boost, output);
        // if(output.size()>0){
        //     for(int i=0;i<output.size();i++){
        //         boost::geometry::difference(obs_boost, output[i], output);
        //         obs_boost = output[0];
        //     }
        // }
        //change boost object to a vector of points
        obs_boost= output[0];
        temp_obj.clear();
        for(auto it = boost::begin(boost::geometry::exterior_ring(obs_boost)); it != boost::end(boost::geometry::exterior_ring(obs_boost)); ++it){
            float x = boost::geometry::get<0>(*it);
            float y = boost::geometry::get<1>(*it);
            // std::cout << "the output from boost: (" << x << " , " << y << " )" << endl;
            temp_obj.push_back({x,y});
        }
        temp_obj.pop_back();
        // std::cout << "------------" << endl;
        trimmed_obstacles.push_back(temp_obj);
        // std::string name1 = "/home/basemprince/workspace/project/output/obstacle_" + std::to_string(count)  + ".svg";
        // std::string name2 = "/home/basemprince/workspace/project/output/border_" + std::to_string(count)  + ".svg";
        // writeSvg_single(obs_boost,name1);
        // writeSvg_single(border_boost,name2);
        // std::cout << "Obstacle" << boost::geometry::dsv(obs_boost) << " has an area of " << boost::geometry::area(obs_boost) << std::endl;
        // std::cout << "border" << boost::geometry::dsv(border_boost) << " has an area of " << boost::geometry::area(border_boost) << std::endl;
        // std::string name="/home/basemprince/workspace/project/output/diff_" + std::to_string(count)  + ".svg";
        // writeSvg(output, name);
        output.clear();
        // std::cout << "-----------------" << endl;
    }
    // std::cout << "size of new obstacle size : " << trimmed_obstacles.size() << endl;
    // for(int i=0; i<trimmed_obstacles.size();i++){
    //     if(trimmed_obstacles[i][0].x != trimmed_obstacles[i].back().x || trimmed_obstacles[i][0].y != trimmed_obstacles[i].back().y){
    //         trimmed_obstacles[i].push_back(trimmed_obstacles[i][0]);
    //     }
        // std::cout << "size of new obstacle #: " << i << " is : " << trimmed_obstacles[i].size() << endl;
        // for(int j=1 ; j< trimmed_obstacles[i].size();j++){
    //     //     cv::line(plot, cv::Point2f(trimmed_obstacles[i][j-1].x*enlarge,trimmed_obstacles[i][j-1].y*enlarge), cv::Point2f(trimmed_obstacles[i][j].x*enlarge,trimmed_obstacles[i][j].y*enlarge), cv::Scalar(0,0,0), 3);
    //     //     cv::imshow("Clipper", plot);
    //     //     cv::waitKey(0); 
        // }
    // }

    // std::cout << "size of old obstacle size : " << obstacle_list.size()<< endl;
    // for(int i=0; i<obstacle_list.size();i++){
    //     if(trimmed_obstacles[i][0].x != trimmed_obstacles[i].back().x || trimmed_obstacles[i][0].y != trimmed_obstacles[i].back().y){
    //         trimmed_obstacles[i].push_back(trimmed_obstacles[i][0]);
    //     }
        // std::cout << "size of obstacle #: " << i << " is : " << obstacle_list[i].size()<< endl;
        // for(int j=1 ; j< trimmed_obstacles[i].size();j++){
            // std::cout << "current segment in in obstiacle " << i << "is : (" << trimmed_obstacles[i][j-1].x << " , " << trimmed_obstacles[i][j-1].y << ") ( " << trimmed_obstacles[i][j].x << " , " << trimmed_obstacles[i][j].y << " )"<<std::endl;
        //     cv::line(plot, cv::Point2f(trimmed_obstacles[i][j-1].x*enlarge,trimmed_obstacles[i][j-1].y*enlarge), cv::Point2f(trimmed_obstacles[i][j].x*enlarge,trimmed_obstacles[i][j].y*enlarge), cv::Scalar(0,0,0), 3);
        //     cv::imshow("Clipper", plot);
        //     cv::waitKey(0); 
        // }
    // }


    return trimmed_obstacles;
}



// std::vector<Polygon> trim_obstacles_old(const std::vector<Polygon>& obstacle_list,const Polygon &borders, cv::Mat plot){
//     std::vector<Polygon> new_obstacles;
//     Polygon new_borders = borders;
//     Polygon new_obstacle;
//     SEGMENT obs_segment;
//     SEGMENT border_segment;
//     POINT intersection_pt;
//     POINT intersection_pt2;
//     bool out_of_border = false;
//     int tracker = 0;

//     float y_limit_lower = min(min(borders[0].y, borders[1].y), min(borders[2].y, borders[3].y));
//     float y_limit_upper = max(max(borders[0].y, borders[1].y), max(borders[2].y, borders[3].y));
//     float x_limit_lower = min(min(borders[0].x, borders[1].x), min(borders[2].x, borders[3].x));
//     float x_limit_upper = max(max(borders[0].x, borders[1].x), max(borders[2].x, borders[3].x));

//     if(new_borders[0].x != new_borders.back().x || new_borders[0].y != new_borders.back().y){
//         new_borders.push_back(new_borders[0]);
//     }
//     for (int i = 0 ; i < new_borders.size(); i ++){
//         std::cout << "border # " << i << " ( " << new_borders[i].x*enlarge << " , " << new_borders[i].y*enlarge << " ) " << std::endl;
//     }
//     // int obs_counter = 0;
//     // remove the parts of the obstacles that fall outside of the borders
//     for(Polygon obstacle : obstacle_list){
//         new_obstacle.clear();
//         if(obstacle[0].x != obstacle.back().x || obstacle[0].y != obstacle.back().y){
//             obstacle.push_back(obstacle[0]);
//         }
//         // obs_counter ++;
//         // new_obstacle = obstacle;
//         for(int obs_pt = 0 ; obs_pt < obstacle.size()-1;obs_pt++){
//             // std::cout << "-- obstacle #: "<< obs_counter << " point #: " << obs_pt << std::endl;
//             out_of_border = obstacle[obs_pt].x > x_limit_upper || obstacle[obs_pt].x < x_limit_lower
//             || obstacle[obs_pt].y > y_limit_upper || obstacle[obs_pt].y < y_limit_lower;
//             if(out_of_border){
//                 // std::cout << "out of border" << std::endl;
//                 obs_segment.a = {obstacle[obs_pt].x,obstacle[obs_pt].y};
//                 for(int border_pt = 0 ; border_pt < new_borders.size()-1 ; border_pt++){
//                     obs_segment.b = {obstacle[obs_pt+1].x,obstacle[obs_pt+1].y};
//                     border_segment.a = {new_borders[border_pt].x,new_borders[border_pt].y};
//                     border_segment.b = {new_borders[border_pt+1].x,new_borders[border_pt+1].y};
//                     intersection_pt = segment_intersection(obs_segment,border_segment,false);
//                     int output7 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
//                     int output8 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
//                     int output9 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
//                     auto color_rand = cv::Scalar(output7,output8,output9);
//                     cv::Point2f centerCircle(obstacle[obs_pt].x*enlarge,obstacle[obs_pt].y*enlarge);
//                     cv::circle(plot, centerCircle, 1,cv::Scalar( 0, 0, 0 ),cv::FILLED,cv::LINE_8);
//                     cv::line(plot, cv::Point2f(border_segment.a.x*enlarge,border_segment.a.y*enlarge), cv::Point2f(border_segment.b.x*enlarge,border_segment.b.y*enlarge), color_rand, 1);
//                     cv::line(plot, cv::Point2f(obs_segment.a.x*enlarge,obs_segment.a.y*enlarge), cv::Point2f(obs_segment.b.x*enlarge,obs_segment.b.y*enlarge), color_rand, 1);
//                     // cv::imshow("Clipper", plot);
//                     // cv::waitKey(0);                    
//                     if(intersection_pt.x == -1){
//                         tracker = obs_pt;
//                         if(tracker == 0){
//                             tracker = obstacle.size()-1;
//                         }
//                         // std::cout << "obs_pt value: " << obs_pt << " tracker value : " << tracker << std::endl;
//                         // std::cout << "-- no intersection -- trying the other way" << std::endl;
//                         obs_segment.b = {obstacle[tracker-1].x,obstacle[tracker-1].y};
//                         intersection_pt = segment_intersection(obs_segment,border_segment,false);
//                         if(intersection_pt.x == -1){
//                             // std::cout << "-- still no intersection -- moving on to next border" << std::endl;
//                             continue;
//                         }
//                         else{
//                             // std::cout << "moving point : " << obstacle[obs_pt].x*enlarge << " , " << obstacle[obs_pt].y * enlarge<< " ) to ( " << intersection_pt.x * enlarge<< " , " << intersection_pt.y * enlarge << " )" << std::endl;
//                             obstacle[obs_pt] = {intersection_pt.x,intersection_pt.y};
//                             new_obstacle.push_back(obstacle[obs_pt]);
//                             break;
//                             // new_obstacle[obs_pt-compensator] = obstacle[obs_pt];
//                         }
//                     }
//                     else if(intersection_pt.x != -1){
//                         tracker = obs_pt;
//                         if(tracker == 0){
//                             tracker = obstacle.size()-1;
//                         }
//                         // std::cout << "obs_pt value: " << obs_pt << " tracker value : " << tracker << std::endl;
//                         // std::cout << "-- found intersection -- trying the other way" << std::endl;
//                         obs_segment.b = {obstacle[tracker-1].x,obstacle[tracker-1].y};
//                         intersection_pt2 = segment_intersection(obs_segment,border_segment,true);
//                         if(intersection_pt2.x == -1){
//                             // std::cout << "moving point : " << obstacle[obs_pt].x*enlarge << " , " << obstacle[obs_pt].y * enlarge<< " ) to ( " << intersection_pt.x * enlarge<< " , " << intersection_pt.y * enlarge << " )" << std::endl;
//                             obstacle[obs_pt] = {intersection_pt.x,intersection_pt.y};
//                             new_obstacle.push_back(obstacle[obs_pt]);
//                             break;
//                             // new_obstacle[obs_pt-compensator] = obstacle[obs_pt];
//                         }
//                         else{
//                             // std::cout << "the point has two intersections" << std::endl;
//                             new_obstacle.push_back({intersection_pt.x,intersection_pt.y});
//                             new_obstacle.push_back({intersection_pt2.x,intersection_pt2.y});
//                             break;

//                         }
//                     }

//                 }
//             }
//             else{
//                 new_obstacle.push_back(obstacle[obs_pt]);
                
//                 // std::cout << " --------next obstacle point --------- " << std::endl;
//             }
//             // if (out_of_border && !inter_check){

//             //     std::cout << "erasing pt: (" << obstacle[obs_pt].x*enlarge << " , " << obstacle[obs_pt].y*enlarge << " ) and compensator is: " << compensator << std::endl;
//             //     cv::Point2f centerCircle(obstacle[obs_pt].x*enlarge,obstacle[obs_pt].y*enlarge);
//             //     cv::circle(plot, centerCircle, 1,cv::Scalar( 0, 0, 255 ),cv::FILLED,cv::LINE_8);
//             //     new_obstacle.erase(new_obstacle.begin()+obs_pt-compensator);
//             //     compensator++;                
//             // }

//         }
//         new_obstacles.push_back(new_obstacle);
//     }
//     //     std::cout << "old obstacle size: " << obstacle_list.size() << " new obstacle size: " << new_obstacles.size() << std::endl;
//     //     for(int i=0; i<new_obstacles.size();i++){
//     //         std::cout << "old size of obstacle # " << i << " is: " << obstacle_list[i].size() << " new size: " << new_obstacles[i].size() << std::endl;
//     //         if(new_obstacles[i][0].x != new_obstacles[i].back().x || new_obstacles[i][0].y != new_obstacles[i].back().y){

//     //             new_obstacles[i].push_back(new_obstacles[i][0]);
//     //             std::cout << "getting triggered" << std::endl;
//     //         }
//     //         for(int j=1 ; j< new_obstacles[i].size();j++){
//     //             std::cout << "-- obstacle #: "<< i << " section #: " << j  << " ( " << (new_obstacles[i][j-1].x*enlarge) << " , " << (new_obstacles[i][j-1].y*enlarge)  << " ) , ( " << (new_obstacles[i][j].x*enlarge) << " , " << (new_obstacles[i][j].y*enlarge) << " )" << std::endl;
//     //             std::cout << "-- old obstacle #: "<< i << " section #: " << j  << " ( " << (obstacle_list[i][j-1].x*enlarge) << " , " << (obstacle_list[i][j-1].y*enlarge)  << " ) , ( " << (obstacle_list[i][j].x*enlarge) << " , " << (obstacle_list[i][j].y*enlarge) << " )" << std::endl;
//     //             cv::line(plot, cv::Point2f(new_obstacles[i][j-1].x*enlarge,new_obstacles[i][j-1].y*enlarge), cv::Point2f(new_obstacles[i][j].x*enlarge,new_obstacles[i][j].y*enlarge), cv::Scalar(0,0,0), 3);
//     //             cv::imshow("Clipper", plot);
//     //             cv::waitKey(0); 
//     //         }
//     // }

//     return new_obstacles;
// }

bool overlap_check(const Polygon &pol1, const Polygon &pol2){
    Polygon obs1 = pol1;
    Polygon obs2 = pol2;

    // using separated axes theorem to check for overlaping
    // check the projection of each edge of both obstacles
    for (int obs_count = 0; obs_count < 2; obs_count++){
        // switch the obstacles
        if (obs_count == 1){
            obs1 = pol2;
            obs2 = pol1;
        }
        for (int pt = 0; pt < obs1.size()-1; pt++){
            // to loop back to first point
            int next_pt = (pt + 1); // % obs1.size();
            Point axis_proj = {-(obs1[next_pt].y - obs1[pt].y), obs1[next_pt].x - obs1[pt].x};
            // std::cout << "current point -> ( " << obs1[pt].x << ", " << obs1[pt].y << ") , (" <<  obs1[next_pt].x << "," << obs1[next_pt].y << endl;
            float distance = sqrtf(axis_proj.x * axis_proj.x + axis_proj.y * axis_proj.y);
            axis_proj = {axis_proj.x / distance, axis_proj.y / distance};
            // std::cout << "distance calc: " << distance <<endl;
            // std::cout << "axis_proj: ( " << axis_proj.x << " , " << axis_proj.y << " )" << endl;
            float obs1_min = INFINITY;
            float obs1_max = -INFINITY;
            float obs2_min = INFINITY;
            float obs2_max = -INFINITY;
            // min and max points of obstacle 1 projection
            for (int i = 0; i < obs1.size(); i++){
                float j = (obs1[i].x * axis_proj.x + obs1[i].y * axis_proj.y);
                obs1_min = min(obs1_min, j);
                obs1_max = max(obs1_max, j);
            }
            // std::cout << "obs1 min: " << obs1_min << " , max " << obs1_max  << endl;
            // min and max points of obstacle 2 projection
            for (int i = 0; i < obs2.size(); i++){
                float j = (obs2[i].x * axis_proj.x + obs2[i].y * axis_proj.y);
                obs2_min = min(obs2_min, j);
                obs2_max = max(obs2_max, j);
            }
            // std::cout << "obs2 min: " << obs2_min << " , max" << obs2_max  << endl;
            // if one axis has no overlap -> obstacles are not overlaping
            if (!(obs2_max >= obs1_min && obs1_max >= obs2_min)){
                return false;
            }
        }
    }
    return true;
}

void Print_Vector(vector<int> Vec)
{
    cout << "{ ";
    for (int i = 0; i < Vec.size(); i++) {
        cout << Vec[i] << " , ";
    }
    cout << "}" << endl;
    return;
}


std::vector<Polygon> merge_obstacles (const std::vector<Polygon>& obstacle_list,cv::Mat plot){
    std::vector<Polygon> merged_obstacles;
    std::vector<std::vector<int> > obstacle_overlap_tab(obstacle_list.size());
    std::vector<point_xy> temp_points;
    Polygon temp_obj;
    polygon obs1;
    polygon obs2;
    bool overlap_result = false;

    for (int curr_obs = 0; curr_obs < obstacle_list.size(); curr_obs++){
        cv::Point2f centerCircle(obstacle_list[curr_obs][0].x*enlarge,obstacle_list[curr_obs][0].y*enlarge);
        std::string text = std::to_string(curr_obs);
        putText(plot, text, centerCircle, cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(24,30,100), 2);        
        obstacle_overlap_tab[curr_obs].push_back(curr_obs);
        for (int next_obs = curr_obs + 1; next_obs < obstacle_list.size(); next_obs++){

            // for debugging
            // int output7 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            // int output8 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            // int output9 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            // auto color_rand = cv::Scalar(output7,output8,output9);
            // for(int i= 0; i<obstacle_list[curr_obs].size();i++){
            //     int ind = (i+1)%obstacle_list[curr_obs].size();
            //     cv::line(plot, cv::Point2f(obstacle_list[curr_obs][i].x*enlarge,obstacle_list[curr_obs][i].y*enlarge), cv::Point2f(obstacle_list[curr_obs][ind].x*enlarge,obstacle_list[curr_obs][ind].y*enlarge), color_rand, 4);
            // }
            // for(int i= 0; i<obstacle_list[next_obs].size();i++){
            //     int ind = (i+1)%obstacle_list[next_obs].size();
            //     cv::line(plot, cv::Point2f(obstacle_list[next_obs][i].x*enlarge,obstacle_list[next_obs][i].y*enlarge), cv::Point2f(obstacle_list[next_obs][ind].x*enlarge,obstacle_list[next_obs][ind].y*enlarge), color_rand, 4);
            // }
            // cv::imshow("Clipper", plot);
            // cv::waitKey(0); 

            overlap_result = overlap_check(obstacle_list[curr_obs], obstacle_list[next_obs]);
            // std::cout << " obstacle # " << curr_obs << " and obstacle # " << next_obs << " are: " << overlap_result << std::endl;
            if(overlap_result){
                obstacle_overlap_tab[curr_obs].push_back(next_obs);
            }
        }				
    }
    std::vector<std::vector <int> > temp_l = obstacle_overlap_tab;

    // for (int i =0 ; i<obstacle_overlap_tab.size();i++){
    //     // std::cout << "obstacle #" << i << " overlaps with the following obs {";
    //     for(int j=0;j<obstacle_overlap_tab[i].size();j++){    
    //         std::cout << obstacle_overlap_tab[i][j] << " , ";
    //     }    
    //     std::cout << " }" << std::endl;
    // }
    int lf;
    std::vector<std::vector< int> >  merge_list;

    while (int(temp_l.size()) > 0){
        std::vector<int> first = temp_l[0];
        std::sort(first.begin(), first.end());
        std::vector<vector<int> > rest(temp_l.begin() + 1, temp_l.end());
        // set<std::vector< int> > first_s;
        // first_s.insert(first);
        lf = -1;
        while (int(first.size()) > lf){
            lf = first.size();
            std::vector<vector<int> > rest2;
            for (vector<int> r : rest){
                // std::cout << "first_data: {";
                // for(int i = 0 ; i< first.size();i++){
                //     std::cout  << first[i] << " , ";
                // }
                // cout << "}" << std::endl;
                // std::cout << "rest_data: {";
                // for(int i = 0 ; i< r.size();i++){
                //     std::cout  << r[i] << " , ";
                // }
                // cout << "}" << std::endl;

                std::sort(r.begin(), r.end());
                std::vector<int> common_data;
                set_intersection(first.begin(),first.end(),r.begin(),r.end(), std::back_inserter(common_data));
                // std::cout << "common data: {";
                // for(int i = 0 ; i< common_data.size();i++){
                //     std::cout  << common_data[i] << " , ";
                // }
                // cout << "}" << std::endl;
                if (common_data.size()>0){
                    first.insert(first.end(),r.begin(),r.end());
                    sort(first.begin(), first.end());
                    first.erase(unique(first.begin(), first.end()), first.end());
                    // set<std::vector< int> >::iterator it = first_s.begin();
                    // std::cout << "first vector: ";
                    // for(int i = 0 ; i< first.size();i++){
                    //     std::cout  << first[i] << " , ";
                    // }
                    // cout << "}" << std::endl;
                    // for (it = first_s.begin(); it != first_s.end(); ++it){
                    //     Print_Vector(*it);    
                    // }
                }
                else{
                    rest2.push_back(r);
                }
            }
            rest = rest2;
        }
        merge_list.push_back(first);
        temp_l = rest;
    }

    // std::cout << "final data: {";
    // for(int i = 0 ; i< merge_list.size();i++){

    //     Print_Vector(merge_list[i]);
    // }

    // int nxt_frm_bk = 0;
    // for(int i = 0; i< merge_list.size();i++){
    //     std::cout << "i: "<< i << endl;
    //     auto ind = merge_list[i].begin();
    //     while (ind != merge_list[i].end()){
    //         auto nxt_ind = std::next(ind, 1);
    //         std::cout << "ind: " << *ind << " nxt_ind: " << *nxt_ind << endl;
    //         std::cout << "nxt_frm_bk: " << nxt_frm_bk << endl;
    //         if (overlap_check(obstacle_list[*ind], obstacle_list[*nxt_ind])){
    //             nxt_frm_bk = 0;
    //             ++ind;
    //         }
    //         else{
    //             std::cout << "no over lap" << std::endl;
    //             std::cout << "before rotate" << std::endl;
    //             Print_Vector(merge_list[i]);
    //             std::rotate(nxt_ind,nxt_ind +1,merge_list[i].end()-nxt_frm_bk);
    //             nxt_frm_bk ++;
    //             std::cout << "after rotate" << std::endl;
    //             Print_Vector(merge_list[i]);
                
    //             // merge_list[i].erase(nxt_ind);  
    //         }           
    //     }
    // }

    //merging the obstacles
    for (int obs_indices=0; obs_indices < merge_list.size();obs_indices++){
        // std::cout << "merged obstacle # " << obs_indices << std::endl;
        temp_points.clear();
        //convert obs1 to a boost polygon object
        for(Point curr_point : obstacle_list[merge_list[obs_indices][0]]){
            temp_points+= point_xy(curr_point.x,curr_point.y);
        }
        boost::geometry::assign_points(obs1, temp_points);
        correct(obs1);
        for(int i=0;i<merge_list[obs_indices].size()-1;i++){
            int curr_ind = merge_list[obs_indices][i+1];
            // std::cout << "the big merged obstacle # " << obs_indices << " and sub obstacle #:" << curr_ind << endl;

            // convert obs2 to a boost polygon object
            temp_points.clear();
            for(Point curr_point : obstacle_list[curr_ind]){
                temp_points+= point_xy(curr_point.x,curr_point.y);
            }
            boost::geometry::assign_points(obs2, temp_points);
            // std::string name1 = "/home/basemprince/workspace/project/output/obs1_" + std::to_string(obs_indices) + std::to_string(curr_ind) + ".svg";
            // std::string name2 = "/home/basemprince/workspace/project/output/obs2_" + std::to_string(obs_indices) + std::to_string(curr_ind) + ".svg";
            correct(obs2);
            // writeSvg_single(obs1,name1);
            // writeSvg_single(obs2,name2);
            // std::cout << "Obstacle 1" << boost::geometry::dsv(obs1) << " has an area of " << boost::geometry::area(obs1) << std::endl;
            // std::cout << "Obstacle 2" << boost::geometry::dsv(obs2) << " has an area of " << boost::geometry::area(obs2) << std::endl;
            std::vector<polygon> output;
            boost::geometry::union_(obs1, obs2, output);
            obs1 = output[0];
            // std::string name="/home/basemprince/workspace/project/output/file_" + std::to_string(obs_indices) + std::to_string(curr_ind) + ".svg";
            // std::cout << name << endl;
            // writeSvg(output, name);
            // std::cout << "Obstacle union " << boost::geometry::dsv(output[0]) << " has an area of " << boost::geometry::area(output[0]) << std::endl;
            // int j = 0;
            // std::cout << "green || blue:" << std::endl;
            if(output.size()>1){
                merge_list[obs_indices].push_back(curr_ind);
            }
            // BOOST_FOREACH(polygon const& p, output){
            //     std::cout << j++ << ": " << boost::geometry::area(p) << std::endl;
            // }
            // std::cout << "-------------------" << endl;
        }
        //change boost object to a vector of points
        temp_obj.clear();
        for(auto it = boost::begin(boost::geometry::exterior_ring(obs1)); it != boost::end(boost::geometry::exterior_ring(obs1)); ++it){
            float x = boost::geometry::get<0>(*it);
            float y = boost::geometry::get<1>(*it);
            temp_obj.push_back({x,y});
        }
        temp_obj.pop_back();
        merged_obstacles.push_back(temp_obj);
    }
    
    for(int i=0; i<merged_obstacles.size();i++){
        // if(merged_obstacles[i][0].x != merged_obstacles[i].back().x || merged_obstacles[i][0].y != merged_obstacles[i].back().y){
        //     merged_obstacles[i].push_back(merged_obstacles[i][0]);
        // }
        for(int j=0 ; j< merged_obstacles[i].size();j++){
            int nxt_ind = (j+1) % merged_obstacles[i].size();
            cv::line(plot, cv::Point2f(merged_obstacles[i][j].x*enlarge,merged_obstacles[i][j].y*enlarge), cv::Point2f(merged_obstacles[i][nxt_ind].x*enlarge,merged_obstacles[i][nxt_ind].y*enlarge), cv::Scalar(0,0,0), 2);

        }
    }
    cv::imshow("Clipper", plot);
    cv::waitKey(0); 
    return merged_obstacles;
;
}
