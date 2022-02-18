#include "collision_detect.hpp"

//function that performs a cross product
double cross_prod(cv::Point2d a, cv::Point2d b){
	return a.x * b.y - a.y * b.x;
}

//function that performs a dot product
double dot_prod(cv::Point2d a, cv::Point2d b){
	return a.x * b.x + a.y * b.y;
}

//function to calculate distance between two points
double eucl_distance(cv::Point2d a, cv::Point2d b){
	return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2));
}


POINT segment_intersection(SEGMENT sigment1, SEGMENT sigment2){
    POINT intersection_pt;
    boost::geometry::model::linestring<point_boost> line1, line2;
    typedef boost::geometry::model::segment<point_boost> Segment;
    Segment AB( point_boost(sigment1.a.x,sigment1.a.y), point_boost(sigment1.b.x,sigment1.b.y) );
    Segment CD( point_boost(sigment2.a.x,sigment2.a.y), point_boost(sigment2.b.x,sigment2.b.y) );

    std::vector<point_boost> result;
    boost::geometry::intersection(AB, CD,result);

    if (result.size()>0){
        intersection_pt = {float(boost::geometry::get<0>(result[0])),float(boost::geometry::get<1>(result[0]))};
    }
    else{
        intersection_pt = {-1,-1};
    }
    return intersection_pt;
}

bool check_obstruction(std::vector< std::vector<POINT> > obstacles, SEGMENT segment) {
    int res = true;
    int break_out = false;
    int n;
    SEGMENT obs_side;

    for(std::vector<POINT> &obs : obstacles){
        // check that the obstacle starts and ends with the same point
        
        if(obs[0].x != obs.back().x || obs[0].y != obs.back().y){
            obs.push_back(obs[0]);
        }
        n = obs.size()-1;
        for (int pt = 0; pt < n; pt++ ){
            obs_side.a = obs[pt];
            obs_side.b = obs[pt+1];
            if(segment_intersection(segment,obs_side).x != -1){
                res = false;
                break_out = true;
                break;
            }
        }
        if (break_out){
            break;
        }
    }

    return res;
}

//checks if 2 line segments are intersect.
std::vector<cv::Point2d> line_line_coll(std::vector<cv::Point2d> line_a, std::vector<cv::Point2d> line_b){
    std::vector<cv::Point2d> pts;
	double t;
	std::vector<double> ts;

	double x_a1 = line_a[0].x;
	double y_a1 = line_a[0].y;

	double x_a2 = line_a[1].x;
	double y_a2 = line_a[1].y;

	double x_b1 = line_b[0].x;
	double y_b1 = line_b[0].y;

	double x_b2 = line_b[1].x;
	double y_b2 = line_b[1].y;

	double minX_a = min(x_a1,x_a2);
	double minY_a = min(y_a1,y_a2);
	double maxX_a = max(x_a1,x_a2);
	double maxY_a = max(y_a1,y_a2);
	   
	double minX_b = min(x_b1,x_b2);
	double minY_b = min(y_b1,y_b2);
	double maxX_b = max(x_b1,x_b2);
	double maxY_b = max(y_b1,y_b2);


	if (maxX_b<minX_a || minX_b>maxX_a || maxY_b<minY_a || minY_b>maxY_a) {
		return pts;
	}

	cv::Point2d q = cv::Point2d(x_a1, y_a1);
	cv::Point2d s = cv::Point2d(x_a2-q.x, y_a2-q.y);

	cv::Point2d p = cv::Point2d(x_b1, y_b1);
	cv::Point2d r = cv::Point2d(x_b2-p.x, y_b2-p.y);

	cv::Point2d diffPQ = cv::Point2d(q.x-p.x, q.y-p.y);

	double crossRS = cross_prod(r, s);
	double crossDiffR = cross_prod(diffPQ,r);
   	double crossDiffS = cross_prod(diffPQ,s);

   	if(crossRS == 0 && crossDiffR == 0){
   		double dotRR = dot_prod(r,r);
       	double dotSR = dot_prod(s,r);
       	double t0 = dot_prod(diffPQ,r)/dotRR;
       	double t1 = t0+dotSR/dotRR;
       	if(dotSR<0){
       		if(t0>=0 && t0<=1){
       			ts = {max(t1,0.0), min(t0,1.0)};
       		}
       	}else{
       		if(t1>=0 && t0<=1){
       			ts = {max(t0,0.0), min(t1,1.0)};
       		}
       	}
   	}else{
   		if(crossRS == 0 && crossDiffR != 0){
   			return pts;
   		}else{
   			t = crossDiffS/crossRS;
   			double u = crossDiffR/crossRS;
   			if(t>=0 && t<=1 && u>=0 && u<=1){
   				ts = {t};
   			}
   		}
   	}

   	for(int t=0; t<ts.size(); t++){
   		cv::Point2d pt = cv::Point2d(p.x+ts[t]*r.x, p.y+ts[t]*r.y);
   		pts.emplace_back(pt);
   	}
   	
   	return pts;
}

//checks if a line segment and a circle are intersected.
std::vector<cv::Point2d> circle_line_coll(double a, double b, double r, std::vector<cv::Point2d> line){
    std::vector<cv::Point2d> pts;
	std::vector<double> t;

	double line_x1 = line[0].x;
	double line_y1 = line[0].y;

	double line_x2 = line[1].x;
	double line_y2 = line[1].y;

	double p1 = 2*line_x1*line_x2;
    double p2 = 2*line_y1*line_y2;
    double p3 = 2*a*line_x1;
    double p4 = 2*a*line_x2;
    double p5 = 2*b*line_y1;
    double p6 = 2*b*line_y2;

    double c1 = pow(line_x1,2)+pow(line_x2,2)-p1+ pow(line_y1,2)+pow(line_y2,2)-p2;
    double c2 = -2*pow(line_x2,2)+p1-p3+p4-2*pow(line_y2,2)+p2-p5+p6;
    double c3 = pow(line_x2,2)-p4+pow(a,2)+pow(line_y2,2)-p6+pow(b,2)-pow(r,2);

    double delta = pow(c2,2)-4*c1*c3;
    double t1, t2, x, y;

    if(delta<0){
    	return pts;
    }
    else{
    	if(delta>0){
    		double deltaSq = sqrt(delta);
    		t1 = (-c2+deltaSq)/(2*c1);
        	t2 = (-c2-deltaSq)/(2*c1);
    	}
    	else{
    		t1 = -c2/(2*c1);
        	t2 = t1;
    	}
    }

    if(t1>=0 && t1<=1){
    	x = line_x1*t1+line_x2*(1-t1);
    	y = line_y1*t1+line_y2*(1-t1);
    	pts.emplace_back(cv::Point2d(x,y));
    	t.emplace_back(t1);
    }

    if(t2 >=0 && t2<=1 && t2!=t1){
	    x = line_x1*t2+line_x2*(1-t2);
	    y = line_y1*t2+line_y2*(1-t2);
	    pts.emplace_back(cv::Point2d(x,y));
    	t.emplace_back(t2);
	}

    return pts;

}
//checks if a line segment and an arc of circle are intersected.
bool arc_line_coll(double a, double b, double r, double s, double e, std::vector<cv::Point2d> line)
{
	bool result = false;

	std::vector<cv::Point2d> pts = circle_line_coll(a, b, r, line);

	if (pts.size() > 0)
	{
		for (auto it = pts.begin(); it != pts.end(); it++)
		{
			float theta = atan2((*it).y-b, (*it).x-a);
			theta = mod2Pi(theta);
			if (s < e && theta >= s && theta <= e)
			{
				result = true;
			}
			if (s > e && !(theta > e && theta < s))
			{
				result = true;
			}
		}
	}

	return result;
}

//function to decide an angle within an arc (not used)
bool arc_pass_intersection(float theta, double s, double e){
	if (s < e && theta >= s && theta <= e)
	{
		return true;
	}
	if (s > e && !(theta > e && theta < s))
	{
		return true;
	}

	return false;
}

//check if two arcs are intersected (not used)
bool arc_arc_coll(double a1, double b1, double r1, double s1, double e1,
                  double a2, double b2, double r2, double s2, double e2)
{
	double d = eucl_distance(cv::Point2d(a1,b1), cv::Point2d(a2,b2));
	
	if (d <= (r1+r2) and d >= abs(r1-r2)){
		double l = (r1*r1 - r2*r2 + d*d) / (2*d);
		double h = sqrt(r1*r1 - l*l);

		if (h > 0) {
			double x1 = l*(a2-a1)/d + h*(b2-b1)/d + a1;
			double y1 = l*(b2-b1)/d - h*(a2-a1)/d + b1;
			double x2 = l*(a2-a1)/d - h*(b2-b1)/d + a1;
			double y2 = l*(b2-b1)/d + h*(a2-a1)/d + b1;

			float theta11 = mod2Pi(atan2(y1-b1,x1-a1));
			float theta12 = mod2Pi(atan2(y1-b2,x1-a2));
			float theta21 = mod2Pi(atan2(y2-b1,x2-a1));
			float theta22 = mod2Pi(atan2(y2-b2,x2-a2));

			// both pass (x1,y1) or both pass (x2,y2)
			bool re11 = arc_pass_intersection(theta11, s1, e1);
			bool re12 = arc_pass_intersection(theta12, s2, e2);
			bool re21 = arc_pass_intersection(theta21, s1, e1);
			bool re22 = arc_pass_intersection(theta22, s2, e2);

			if ((re11 && re12) || (re21 && re22)) return true;
		}
		else{
			double x = l*(a2-a1)/d + a1;
			double y = l*(b2-b1)/d + b1;

			float theta1 = mod2Pi(atan2(y-b1,x-a1));
			float theta2 = mod2Pi(atan2(y-b2,x-a2));

			bool re1 = arc_pass_intersection(theta1, s1, e1);
			bool re2 = arc_pass_intersection(theta2, s2, e2);

			if (re1 && re2) return true;
		}
	}
	
	return false;
}

/* takes two polygons and checks every line to see if they overlap in anyway */
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
            float distance = sqrtf(axis_proj.x * axis_proj.x + axis_proj.y * axis_proj.y);
            axis_proj = {axis_proj.x / distance, axis_proj.y / distance};
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
            // min and max points of obstacle 2 projection
            for (int i = 0; i < obs2.size(); i++){
                float j = (obs2[i].x * axis_proj.x + obs2[i].y * axis_proj.y);
                obs2_min = min(obs2_min, j);
                obs2_max = max(obs2_max, j);
            }
            // if one axis has no overlap -> obstacles are not overlaping
            if (!(obs2_max >= obs1_min && obs1_max >= obs2_min)){
                return false;
            }
        }
    }
    return true;
}

std::tuple<std::vector<std::vector<float> >,std::vector<std::vector<float> >,std::vector<float>,std::vector<std::vector<SEGMENT> >> calculate_distances(std::vector<std::vector<robotPos>> path){
    SEGMENT path_piece;
    float dis=0;
    float curr_dis=0;
    std::vector<std::vector<float> > cumulative_distance;
    std::vector<std::vector<float> > segment_distance;
    std::vector<float> total_path_dist;
    std::vector<std::vector<SEGMENT> > path_segments;
    for(int j = 0; j<path.size();j++){
    //   cout<< "robot # :"<< j << endl;
      segment_distance.push_back({});
      cumulative_distance.push_back({});
      path_segments.push_back({});
      for(int k=1; k<path[j].size();k++){
        path_piece.a= {path[j][k-1].x,path[j][k-1].y};
        path_piece.b= {path[j][k].x,path[j][k].y};
        curr_dis = sqrt(pow(path_piece.b.x - path_piece.a.x,2) + pow(path_piece.b.y- path_piece.a.y,2));
        dis += curr_dis;
        segment_distance[j].push_back(curr_dis);
        cumulative_distance[j].push_back(dis);
        path_segments[j].push_back(path_piece);
        // cout << "segement #:" << k-1 << "(" << path_piece.a.x << " , " << path_piece.a.y << " ),( " << path_piece.b.x << " , " << path_piece.b.y << " )" << endl;
        // cout << "segement distance " << curr_dis << " distance is: " << dis << " \n==============" <<  endl;
      }
      total_path_dist.push_back(dis);
    //   cout << "total path distance for rob#: "<< j << " is: "<< dis << endl;
      dis = 0;
    }
    return std::make_tuple(segment_distance,cumulative_distance,total_path_dist,path_segments);
}

std::vector<std::vector<int> > path_intersect_check(std::vector<std::vector<float> > segment_distance, std::vector<std::vector<float> > cumulative_distance,std::vector<float> total_path_dist,std::vector<std::vector<SEGMENT> > path_segments,cv::Mat plot,bool debug){
    // important parameters
    float slow_down_rate = 0.9; // to slow down the car at the gate
    float start_slow_down = 0.15; // at what distance left to start slow down
    float offset = 0.07; // how big the boxes around the points  
    
    bool overlap = false;
    float step_size = 0.02;
    float current_step = step_size;
    float distance_on_seg = 0;
    Point car_point;
    float dis_ratio = 0;
    std::vector<std::vector<Polygon>> car_boxes;
    float slow_down = 1;
    std::vector<std::vector<int> > intersection_lines;
    int itr;
    float at_start;
    for(int i=0;i<cumulative_distance.size();i++){ // for each robot
        car_boxes.push_back({}); // create new robot
        current_step = step_size;
        itr = 0;
        for(int s=0;s<cumulative_distance[i].size();s++){
            while (current_step < cumulative_distance[i][s]){
                if((total_path_dist[i]- current_step) < start_slow_down){
                    if(slow_down>0.1){slow_down*= slow_down_rate;}
                }
                else{slow_down=1;}
                car_boxes[i].push_back({}); //create new polygon
                // cout << "slow down"<< slow_down << " current_step" << current_step<< endl;
                at_start = cumulative_distance[i][s] - segment_distance[i][s];
                distance_on_seg = current_step - at_start;
                // cout << "i------s: "<<i<< " , " << s << " segement_distance " << segment_distance[i][s] << endl;
                // cout << "current_step " << current_step << "cumulatinve_distance" << cumulative_distance[i][s] << endl;
                // cout << "at start " << at_start << endl;
                dis_ratio = distance_on_seg / segment_distance[i][s];
                // cout << "distance_on_seg " << distance_on_seg << " dis_ratio " << dis_ratio << endl;
                car_point = {((1 - dis_ratio) * path_segments[i][s].a.x + dis_ratio * path_segments[i][s].b.x), ((1 - dis_ratio) * path_segments[i][s].a.y + dis_ratio * path_segments[i][s].b.y)};
                // cout << "car_point" << s << "(" << car_point.x << "," << car_point.y << endl;
                car_boxes[i][itr].push_back({car_point.x-offset,car_point.y-offset});
                car_boxes[i][itr].push_back({car_point.x+offset,car_point.y-offset});
                car_boxes[i][itr].push_back({car_point.x+offset,car_point.y+offset});
                car_boxes[i][itr].push_back({car_point.x-offset,car_point.y+offset});
                // cout << "----------"<< endl;
                // cout << "car_boxes " << s << " " << car_boxes[i][itr].size()-4 << " (" << car_boxes[i][itr][car_boxes[i][itr].size()-4].x << "," << car_boxes[i][itr][car_boxes[i][itr].size()-4].y << endl;
                // cout << "car_boxes " << s << " " << car_boxes[i][itr].size()-3 << " (" << car_boxes[i][itr][car_boxes[i][itr].size()-3].x << "," << car_boxes[i][itr][car_boxes[i][itr].size()-3].y << endl;
                // cout << "car_boxes " << s << " " << car_boxes[i][itr].size()-2 << " (" << car_boxes[i][itr][car_boxes[i][itr].size()-2].x << "," << car_boxes[i][itr][car_boxes[i][itr].size()-2].y << endl;
                // cout << "car_boxes " << s << " " << car_boxes[i][itr].size()-1 << " (" << car_boxes[i][itr][car_boxes[i][itr].size()-1].x << "," << car_boxes[i][itr][car_boxes[i][itr].size()-1].y << endl;
                // cout << "------------" << endl;
                itr+=1;
                current_step+=(step_size*slow_down);
            }
        }
    }
    int longest_path_ind = 0;
    float longest_path = 0;
    for(int j = 0 ; j<car_boxes.size();j++){
        if(total_path_dist[j]>longest_path){
            longest_path = total_path_dist[j];
            longest_path_ind = j;
        }
    }
    // cout << "longest path " << longest_path << "longest path index " <<longest_path_ind << endl;

    if(debug){
        for(int i= 0; i<car_boxes[longest_path_ind].size();i++){// for polygon in the step
            int output7 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            int output8 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            int output9 = 0 + (rand() % static_cast<int>(255 - 0 + 1));
            auto color_rand = cv::Scalar(output7,output8,output9);
            overlap = false;
            for(int j= 0; j<car_boxes.size();j++){ // for each robot
                // cout << "i: " << i << " j: " << j << endl;
                if(i< car_boxes[j].size()){ // make sure that we still have boxes left in the step for each robot
                    for(int s= 0; s<car_boxes[j][i].size();s++){ // for printing the polygons
                        int ind = (s+1)%car_boxes[j][i].size();
                        cv::line(plot, cv::Point2f(car_boxes[j][i][s].x*enlarge,car_boxes[j][i][s].y*enlarge), cv::Point2f(car_boxes[j][i][ind].x*enlarge,car_boxes[j][i][ind].y*enlarge), color_rand, 4);
                    }
                }
            } 
            // cv::imshow("Clipper", plot);
            // cv::waitKey(0);  
        }
    }

    bool break_out = false;
    for(int i= 0; i<car_boxes[longest_path_ind].size()&&!break_out;i++){// for polygon in the step
        overlap = false;
        for(int j= 0; j<car_boxes.size()-1&&!break_out;j++){ // for each robot
            for(int k=j+1;k<car_boxes.size()&&!break_out;k++){
                // cout << "i: " << i << " j: " << j << " k: " << k << endl; 
                if(i< car_boxes[j].size() && i< car_boxes[k].size()){ // make sure that we still have boxes left in the step for each robo
                    overlap = overlap_check(car_boxes[j][i], car_boxes[k][i]);
                    // cout << "overlap: " << overlap << endl;
                    if(overlap){
                        intersection_lines.push_back({j,k});
                        break_out = true;
                    } 
                }
            }    
        }   
    }

    if (intersection_lines.empty()){
        intersection_lines.push_back({-1,-1});
    }
    return intersection_lines;
    
}