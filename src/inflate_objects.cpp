
#include "inflate_objects.hpp"
#include <set>


/* takes a polygon and simplifies it using the boost function simplify */
Polygon simplify_poly (Polygon poly, float tol){
    Polygon temp_obj;
    std::vector<point_boost> temp_points;
    boost::geometry::model::linestring<point_boost> line;
    boost::geometry::model::linestring<point_boost> simplified;
    for(Point curr_point : poly){
            line+= point_boost(curr_point.x,curr_point.y);
        }
    boost::geometry::simplify(line, simplified, tol);
    for (point_boost point : simplified){
        float x = point.x();
        float y = point.y();
        temp_obj.push_back({x,y});
    }   
    return temp_obj;
}

/*
takes the obsticales in the arena and inflates them to account for the size of the robot
outputs the inflated obsticales
*/
std::vector<Polygon> inflate_obstacles(const std::vector<Polygon>& obstacle_list, float inflate_value,bool simplify, cv::Mat plot){
    std::vector<Polygon> new_obsticale_list = obstacle_list;
    std::vector<Polygon> inflated_obsticale_list;
    int px, py;

    for (Polygon obstacle : obstacle_list) {
        if (simplify){
            obstacle = simplify_poly(obstacle,0.02);
        }
        
        ClipperLib::Path clib_obsticale;
        ClipperLib::Paths clib_merged_obs;

        // extract obstacle to a clipper object
        for (const auto &position : obstacle) {
            clib_obsticale << ClipperLib::IntPoint(position.x * enlarge, position.y * enlarge);
        }

        // applying the offset
        ClipperLib::ClipperOffset co;
        co.MiterLimit = 10;

        co.AddPath(clib_obsticale, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
        co.Execute(clib_merged_obs, inflate_value);

        // change the clipper object to a polygon vector format
        int counter = 0;
        for(const ClipperLib::Path &path : clib_merged_obs){
            Polygon inflated_poly;
            for(const ClipperLib::IntPoint &point: path){
                double x = point.X / enlarge;
                double y = point.Y / enlarge;
                inflated_poly.emplace_back(x, y);
                counter ++;
            }
            inflated_obsticale_list.emplace_back(inflated_poly);
        }
        // for plotting the output
        for(int i=0; i<new_obsticale_list.size();i++){
            for(int j=0 ; j< new_obsticale_list[i].size();j++){
                int nxt_ind = (j+1) % new_obsticale_list[i].size();
                cv::line(plot, cv::Point2f(new_obsticale_list[i][j].x*enlarge,new_obsticale_list[i][j].y*enlarge), cv::Point2f(new_obsticale_list[i][nxt_ind].x*enlarge,new_obsticale_list[i][nxt_ind].y*enlarge), cv::Scalar(255,0,0), 1);
            }
        }
        // for plotting the output
        for(int i=0; i<inflated_obsticale_list.size();i++){

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

    // crude way of sorting the boarders in a specific clockwise manner
    // assumes that the arena is sqaure, will break if it is not
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

    return inflated_borders_sorted;
}

/* takes the inflated obstacles and cuts any parts of them that are outside of the boarders
using the booster library intersection */
std::vector<Polygon> trim_obstacles(const std::vector<Polygon>& obstacle_list,const Polygon &borders, cv::Mat plot){
    std::vector<Polygon> trimmed_obstacles;
    std::vector<point_boost> temp_points;
    Polygon temp_obj;
    polygon_boost obs_boost;
    polygon_boost border_boost;
    std::vector<polygon_boost> output;

    // converting borders to a boost object
    for(Point curr_point: borders){
        temp_points+= point_boost(curr_point.x,curr_point.y);
    }
    boost::geometry::assign_points(border_boost, temp_points);
    correct(border_boost);
    int count = 0;
    // converting obstacles to a boost object
    for(Polygon curr_obs : obstacle_list){
        count++;
        temp_points.clear();
        for(Point curr_point : curr_obs){ 
            temp_points+= point_boost(curr_point.x,curr_point.y);
        }
        boost::geometry::assign_points(obs_boost, temp_points);
        correct(obs_boost);
        boost::geometry::intersection(obs_boost, border_boost, output);
        //change boost object to a vector of points
        obs_boost= output[0];
        temp_obj.clear();
        for(auto it = boost::begin(boost::geometry::exterior_ring(obs_boost)); it != boost::end(boost::geometry::exterior_ring(obs_boost)); ++it){
            float x = boost::geometry::get<0>(*it);
            float y = boost::geometry::get<1>(*it);
            temp_obj.push_back({x,y});
        }
        temp_obj.pop_back();

        trimmed_obstacles.push_back(temp_obj);
        output.clear();
    }
    return trimmed_obstacles;
}

/* helper function to print out the recieved vector */
void Print_Vector(vector<int> Vec)
{
    cout << "{ ";
    for (int i = 0; i < Vec.size(); i++) {
        cout << Vec[i] << " , ";
    }
    cout << "}" << endl;
    return;
}

/* takes a vector of polygons and merges them in an iterative way
it first checks overlap, if true, it merges those polygons. it keeps
building up the merged obstacles as long as the [new merged obstacle] overlaps
with another obstacle until overlap check returns false for all*/
std::vector<Polygon> merge_obstacles (const std::vector<Polygon>& obstacle_list,bool simplify,cv::Mat plot){
    std::vector<Polygon> merged_obstacles;
    std::vector<std::vector<int> > obstacle_overlap_tab(obstacle_list.size());
    std::vector<point_boost> temp_points;
    Polygon temp_obj;
    polygon_boost obs1;
    polygon_boost obs2;
    bool overlap_result = false;

    for (int curr_obs = 0; curr_obs < obstacle_list.size(); curr_obs++){
        cv::Point2f centerCircle(obstacle_list[curr_obs][0].x*enlarge,obstacle_list[curr_obs][0].y*enlarge);
        std::string text = std::to_string(curr_obs);
        putText(plot, text, centerCircle, cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(24,30,100), 2);        
        obstacle_overlap_tab[curr_obs].push_back(curr_obs);
        // check overlap between obstacles
        for (int next_obs = curr_obs + 1; next_obs < obstacle_list.size(); next_obs++){
            overlap_result = overlap_check(obstacle_list[curr_obs], obstacle_list[next_obs]);
            if(overlap_result){
                obstacle_overlap_tab[curr_obs].push_back(next_obs);
            }
        }				
    }
    std::vector<std::vector <int> > temp_l = obstacle_overlap_tab;
    int lf;
    std::vector<std::vector< int> >  merge_list;

    // creates a list of which obstacles need to be merged together
    while (int(temp_l.size()) > 0){
        std::vector<int> first = temp_l[0];
        std::sort(first.begin(), first.end());
        std::vector<vector<int> > rest(temp_l.begin() + 1, temp_l.end());
        lf = -1;
        while (int(first.size()) > lf){
            lf = first.size();
            std::vector<vector<int> > rest2;
            for (vector<int> r : rest){
                std::sort(r.begin(), r.end());
                std::vector<int> common_data;
                set_intersection(first.begin(),first.end(),r.begin(),r.end(), std::back_inserter(common_data));
                if (common_data.size()>0){
                    first.insert(first.end(),r.begin(),r.end());
                    sort(first.begin(), first.end());
                    first.erase(unique(first.begin(), first.end()), first.end());
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

    //merging the obstacles based on the merge list
    for (int obs_indices=0; obs_indices < merge_list.size();obs_indices++){
        temp_points.clear();
        //convert obs1 to a boost polygon object
        for(Point curr_point : obstacle_list[merge_list[obs_indices][0]]){
            temp_points+= point_boost(curr_point.x,curr_point.y);
        }
        boost::geometry::assign_points(obs1, temp_points);
        correct(obs1);
        // iterate though the merge list and use boost union to merge them
        for(int i=0;i<merge_list[obs_indices].size()-1;i++){
            int curr_ind = merge_list[obs_indices][i+1];

            // convert obs2 to a boost polygon object
            temp_points.clear();
            for(Point curr_point : obstacle_list[curr_ind]){
                temp_points+= point_boost(curr_point.x,curr_point.y);
            }
            boost::geometry::assign_points(obs2, temp_points);
            correct(obs2);
            std::vector<polygon_boost> output;
            boost::geometry::union_(obs1, obs2, output);
            // it will always be one output because it has already
            // been checked before that these obstacles are overlaping
            obs1 = output[0];
            if(output.size()>1){
                merge_list[obs_indices].push_back(curr_ind);
            }
        }
        //change boost object to a vector of points
        temp_obj.clear();
        for(auto it = boost::begin(boost::geometry::exterior_ring(obs1)); it != boost::end(boost::geometry::exterior_ring(obs1)); ++it){
            float x = boost::geometry::get<0>(*it);
            float y = boost::geometry::get<1>(*it);
            temp_obj.push_back({x,y});
        }
        temp_obj.pop_back();
        // if simplify is true <-simplify results
        if(simplify){
            temp_obj = simplify_poly(temp_obj,0.03);
        }
        
        merged_obstacles.push_back(temp_obj);
    }
    // for plotting purposes
    for(int i=0; i<merged_obstacles.size();i++){
        for(int j=0 ; j< merged_obstacles[i].size();j++){
            int nxt_ind = (j+1) % merged_obstacles[i].size();
            cv::line(plot, cv::Point2f(merged_obstacles[i][j].x*enlarge,merged_obstacles[i][j].y*enlarge), cv::Point2f(merged_obstacles[i][nxt_ind].x*enlarge,merged_obstacles[i][nxt_ind].y*enlarge), cv::Scalar(0,0,0), 2);

        }
    }
    return merged_obstacles;
;
}
