#include "vertical_cell_decomposition.hpp"

/* function to sort the vertices of all the obsticals based on an increasing x axis
takes the obstacles, an empty sorted vertices list and the total count of the vertices
returns the sorted vertices full*/
std::vector<POINT> sort_vertices(std::vector< std::vector<POINT> > obstacles, std::vector<POINT> sorted_vertices, int vertices_num) {

    int add_to_list;

    for(int curr_vertex = vertices_num - 1; curr_vertex >= 0; curr_vertex--) {
      sorted_vertices.push_back(POINT{-1,-1,-1,-1});
    }

    for(int curr_vertex = vertices_num - 1; curr_vertex >= 0; curr_vertex--) {
      for(int obj = 0; obj < obstacles.size(); obj++) {
        for(int vertex = 0; vertex < obstacles[obj].size(); vertex++) {
          add_to_list = 0;
          if(obstacles[obj][vertex].x > sorted_vertices[curr_vertex].x) {
            if( curr_vertex == vertices_num - 1 ||
            obstacles[obj][vertex].x < sorted_vertices[curr_vertex + 1].x ||
            obstacles[obj][vertex].x == sorted_vertices[curr_vertex + 1].x)
            {add_to_list = 1;}

            for(int vert = 0; vert < sorted_vertices.size(); vert ++) {
              if (sorted_vertices[vert].x == obstacles[obj][vertex].x && sorted_vertices[vert].y == obstacles[obj][vertex].y) {
                add_to_list = 0;
              }
            }
          }
          if(add_to_list == 1) {
            sorted_vertices[curr_vertex].x = obstacles[obj][vertex].x;
            sorted_vertices[curr_vertex].y = obstacles[obj][vertex].y;
            sorted_vertices[curr_vertex].obs = obj;
          }
        }             
      } 
    }
    return sorted_vertices;
}

/* function to add the first point back to the vector of the polygon to account for closing the polygon */
std::vector< std::vector<POINT> > close_polygons (std::vector< std::vector<POINT> > polygons) {
    for(int polygon = 0; polygon < polygons.size(); polygon++) {
      //check if it has already been added
      if(polygons[polygon][0].x != polygons[polygon].back().x || polygons[polygon][0].x != polygons[polygon].back().x ){
        polygons[polygon].push_back(polygons[polygon][0]);
      }
    }
    return polygons;
}

/* function to find the map's vertical lines based on the vertices of the obsticale */
std::vector< SEGMENT > find_lines(std::vector<POINT> sorted_vertices, std::vector< std::vector<POINT> > obstacles, float y_limit_lower, float y_limit_upper){
    
    SEGMENT curr_segment;
    SEGMENT temp_segment;
    POINT temp_point;
    std::vector< SEGMENT > open_line_segments;


    for(const POINT& pt : sorted_vertices) {
        int up = 0;
        int down = 0;
        bool break_now = false;
        POINT lower_obs_pt;
        POINT upper_obs_pt;
        POINT intersection_point;
      
        temp_point.x = pt.x;
        temp_point.y = y_limit_lower;
        temp_point.obs = pt.obs;
        curr_segment.a = temp_point;
        lower_obs_pt = temp_point;
    
        temp_point.y = y_limit_upper;
        curr_segment.b = temp_point;
        upper_obs_pt = temp_point;

        // go throug all the vertices of the obstacles to check if they intersect with any
        // other obstical lines
        for(int obs = 0; obs < obstacles.size()&&!break_now; obs++) { 
          for(int vertex = 0; vertex < obstacles[obs].size()-1&&!break_now; vertex++) {
            temp_segment.a = obstacles[obs][vertex];
            temp_segment.b = obstacles[obs][vertex + 1];
            // returns -1 if not intersection
            intersection_point = segment_intersection(curr_segment, temp_segment);

            if(intersection_point.x != -1){ 
              // check if the current vertex is from the current obstacle
              if(obs == pt.obs) {
                // check if the intersection point is not the current point
                if((intersection_point.x != pt.x) || (intersection_point.y != pt.y)) {
                    if(intersection_point.y > pt.y) {up = 1;}
                    if(intersection_point.y < pt.y) {down = 1;}
                }
              }
              else {
                if((intersection_point.x != pt.x) || (intersection_point.y != pt.y)) {
                  // make the intersection point the upper point of the vertical line
                  if((up == 0) && (intersection_point.y > pt.y) && (intersection_point.y < upper_obs_pt.y)) {
                      upper_obs_pt = intersection_point;
                  }
                  // make the intersection point the lower point of the vertical line
                  if((down == 0) && (intersection_point.y < pt.y) && (intersection_point.y > lower_obs_pt.y)) {
                      lower_obs_pt = intersection_point;
                  }
                }
              }
            }
            if(up && down) {
              break_now = true;
            }
          }
        }    
        temp_point = {-1,-1};
        temp_segment = {temp_point,temp_point};

        if(up && down) {
            //temp_segment default values of -1 remain unchanged
        }
        else if(down) {
            temp_segment.b = upper_obs_pt;
        }
        else if(up) {
            temp_segment.a = lower_obs_pt;  
        }
        else {
            temp_segment.a = lower_obs_pt;
            temp_segment.b = upper_obs_pt;  
        } 
        open_line_segments.push_back(temp_segment);
    }
    return open_line_segments;
}

/* Finding the centroid of a list of vertices*/
POINT centroid(std::vector<POINT> vertices) {   
  POINT centroid_point = {-1,-1};
  int num = vertices.size();
  float sum_x = 0;
  float sum_y = 0;

  if(num == 0) {
      return centroid_point;
  }

  for(int vert_idx = 0; vert_idx < num; vert_idx++) {
      sum_x += vertices[vert_idx].x;
      sum_y += vertices[vert_idx].y;
  }
  centroid_point = {sum_x/num,sum_y/num};

  return centroid_point;
}
/* function to find the cells based on the open line segments found from the vertical lines
creates categories of vertices:
-> blocked from top
-> blocked from bottom
-> free from both sides
-> completely blocked
based on the category a 4 point cell is formed using the current and the next vertices
of the open lines segments. In addition, A list of lines and trapazoids are generated
to later check them for collision */
std::vector< std::vector<POINT> > find_cells(std::vector<SEGMENT> open_line_segments, std::vector<POINT> sorted_vertices, std::vector< std::vector<POINT> > obstacles){
    POINT temp_point;
    SEGMENT temp_segment;
    POINT curr_vertex;
    SEGMENT curr_segment;
    SEGMENT next_segment;
    SEGMENT next_next_segment;
    POINT next_vertex;
    POINT next_next_vertex;
    std::vector<SEGMENT> lines_to_check;
    std::vector<int> group;
    std::vector< std::vector<POINT> > trapezoids;
    std::vector<POINT> temp_points;
    std::vector< std::vector<POINT> > cells;
    std::vector<int> done;
    bool extra_search = true; // to allow for adding extra trapazoids when next two segements have same x

    for(int i = 0; i < open_line_segments.size(); i++) {
      curr_segment = open_line_segments[i];
      curr_vertex = sorted_vertices[i];
      done = {0,0,1};

      // a is lower limit , b is upper limit

      // group 0 -> not blocked from the bottom
      // group 1 -> not blocked from the top
      // group 2 -> completely blocked

      // if lower limit is blocked -> don't look down
      if(curr_segment.a.x == -1) {done[0] = 1;}
      // if upper limit is blocked -> don't look up :)
      if(curr_segment.b.x == -1) {done[1] = 1;}
      // if upper and lower limits are blocked -> figure out something else
      if((curr_segment.a.x == -1) && (curr_segment.b.x == -1)) {done[2] = 0;}
      int counter = 0;
      for(int j = i+1; j < open_line_segments.size(); j++) {
        counter +=1;
        lines_to_check.clear();
        group.clear();
        trapezoids.clear();
        bool double_check = false;
        bool next_two_seg_same_x = false;
        
        next_segment = open_line_segments[j];
        next_vertex = sorted_vertices[j];
        if( j != open_line_segments.size()-1 && extra_search){
          next_next_segment = open_line_segments[j+1];
          next_next_vertex = sorted_vertices[j+1];
          next_two_seg_same_x = (next_vertex.x == next_next_vertex.x);
        }
        // check to see if the next segemnt is completely free from both sides
        double_check = next_segment.a.x != -1 && next_segment.b.x != -1;
        // if not blocked from the bottom
        if(done[0] == 0) {
          // cur seg is not blocked from bottom
          // and the next vertex is free from both sides
          if(double_check) {
            //--next seg is free         
            temp_segment.a = centroid({curr_segment.a,curr_vertex}); 
            temp_segment.b = centroid({next_segment.a,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(0);            
            //.a remains the same        
            temp_segment.b = centroid({next_segment.b,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(0);
            trapezoids.push_back({curr_segment.a,next_segment.a,next_vertex,curr_vertex});
            trapezoids.push_back({curr_segment.a,next_vertex,next_segment.b,curr_vertex});
          }
          // if next segment is not blocked from the bottom
          else if(next_segment.a.x != -1) {
            //--next seg is not blocked from bottom   
            temp_segment.a = centroid({curr_segment.a,curr_vertex}); 
            temp_segment.b = centroid({next_segment.a,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(0);
            // trapezoids
            trapezoids.push_back({curr_segment.a,next_segment.a,next_vertex,curr_vertex});
            if(extra_search){
              if(next_two_seg_same_x && (next_next_segment.b.x != -1) && (next_next_segment.a.x == -1)){
                //--next next seg is not blocked from top
                temp_segment.a = centroid({curr_segment.a,curr_vertex}); 
                temp_segment.b = centroid({next_next_segment.b,next_next_vertex}); 
                lines_to_check.push_back(temp_segment);
                group.push_back(0);
                trapezoids.push_back({curr_segment.a,next_next_vertex,next_next_segment.b,curr_vertex});              
              }
            }
          }
          //if the next segment is not blocked from the top
          else if(next_segment.b.x != -1) {
            //--next seg is not blocked from top    
            temp_segment.a = centroid({curr_segment.a,curr_vertex}); 
            temp_segment.b = centroid({next_segment.b,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(0);
            trapezoids.push_back({curr_segment.a,next_vertex,next_segment.b,curr_vertex});
            if(extra_search){
              if(next_two_seg_same_x && (next_next_segment.a.x != -1) && (next_next_segment.b.x == -1)){
                // cout << "--next next seg is not blocked from bottom" << endl;
                temp_segment.a = centroid({curr_segment.a,curr_vertex}); 
                temp_segment.b = centroid({next_next_segment.a,next_next_vertex}); 
                lines_to_check.push_back(temp_segment);
                group.push_back(0);
                trapezoids.push_back({curr_segment.a,next_next_segment.a,next_next_vertex,curr_vertex});              
              }
            }
          }
          else {
            temp_segment.a = centroid({curr_segment.a,curr_vertex}); 
            temp_segment.b = next_vertex;
            lines_to_check.push_back(temp_segment);
            group.push_back(0);
            trapezoids.push_back({curr_segment.a,next_vertex,curr_vertex});
          }
        }
        // not blocked from the bottom
        if(done[1] == 0) {
          //cur seg is not blocked from top
          //if next segment is free from both sides
          if(double_check) {
            //--next seg is free        
            temp_segment.a = centroid({curr_segment.b,curr_vertex}); 
            temp_segment.b = centroid({next_segment.a,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(1);         
            temp_segment.b = centroid({next_segment.b,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(1);
            trapezoids.push_back({curr_vertex,next_segment.a,next_vertex,curr_segment.b});
            trapezoids.push_back({curr_vertex,next_vertex,next_segment.b,curr_segment.b});
          }
          // if next segement not blocked from the bottom
          else if(next_segment.a.x != -1) {
            //--next seg is not blocked from bottom      
            temp_segment.a = centroid({curr_segment.b,curr_vertex}); 
            temp_segment.b = centroid({next_segment.a,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(1);
            trapezoids.push_back({curr_vertex,next_segment.a,next_vertex,curr_segment.b});
            if(extra_search){
              if(next_two_seg_same_x && (next_next_segment.b.x != -1)  && (next_next_segment.a.x == -1)){
                //--next next seg is not blocked from top
                temp_segment.a = centroid({curr_segment.b,curr_vertex}); 
                temp_segment.b = centroid({next_next_segment.b,next_next_vertex});
                lines_to_check.push_back(temp_segment);
                group.push_back(1);
                trapezoids.push_back({curr_vertex,next_next_vertex,next_next_segment.b,curr_segment.b});              
              }
            }
          }
          // if next segment is not blocked from the top
          else if(next_segment.b.x != -1) {
            //--next seg is not blocked from top     
            temp_segment.a = centroid({curr_segment.b,curr_vertex}); 
            temp_segment.b = centroid({next_segment.b,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(1);
            trapezoids.push_back({curr_vertex,next_vertex,next_segment.b,curr_segment.b});

            if(extra_search){
              if(next_two_seg_same_x && (next_next_segment.a.x != -1)  && (next_next_segment.b.x == -1)){
                //--next next seg is not blocked from bottom
                temp_segment.a = centroid({curr_segment.b,curr_vertex}); 
                temp_segment.b = centroid({next_next_segment.a,next_next_vertex});
                lines_to_check.push_back(temp_segment);
                group.push_back(1);
                trapezoids.push_back({curr_vertex,next_next_segment.a,next_next_vertex,curr_segment.b});              
              }
            }
          }
          else {
            temp_segment.a = centroid({curr_segment.b,curr_vertex}); 
            temp_segment.b = next_vertex;
            lines_to_check.push_back(temp_segment);
            group.push_back(1);
            trapezoids.push_back({curr_vertex,next_vertex,curr_segment.b});
          }
        }

        //blocked from both
        if(done[2] == 0) {
          // if next segement free from both sides
          if(double_check) {       
            temp_segment.a = curr_vertex; 
            temp_segment.b = centroid({next_segment.a,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(2);         
            temp_segment.b = centroid({next_segment.b,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(2);
            trapezoids.push_back({curr_vertex,next_segment.a,next_vertex});
            trapezoids.push_back({curr_vertex,next_vertex,next_segment.b});
          }
          else if(next_segment.a.x != -1) {        
            temp_segment.a = curr_vertex; 
            temp_segment.b = centroid({next_segment.a,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(2);
            trapezoids.push_back({curr_vertex,next_segment.a,next_vertex});
          }
          else if(next_segment.b.x != -1) {       
            temp_segment.a = curr_vertex; 
            temp_segment.b = centroid({next_segment.b,next_vertex}); 
            lines_to_check.push_back(temp_segment);
            group.push_back(2);

            trapezoids.push_back({curr_vertex,next_vertex,next_segment.b});
          }
          else {
            temp_segment.a = curr_vertex; 
            temp_segment.b = next_vertex;
            lines_to_check.push_back({temp_segment});
            group.push_back(2);
            trapezoids.push_back({curr_vertex,next_vertex});
          }
        }
        std::vector<int> temp_to_remove;
        for(int line = 0; line < lines_to_check.size(); line++) {  
          int no_intersection[3] = {1, 1, 1};           
          for(int obs = 0; obs < obstacles.size(); obs++) {              
            for(int vertex = 0; vertex < obstacles[obs].size()-1; vertex++) {
              temp_segment.a = obstacles[obs][vertex];
              temp_segment.b = obstacles[obs][vertex+1];          
              temp_point = segment_intersection(lines_to_check[line], temp_segment);
              
              if(temp_point.x != -1) {
                no_intersection[group[line]] = 0;
                int found = 0;
                for(int idx = 0; idx < temp_to_remove.size(); idx++) {
                  if(line == temp_to_remove[idx]) {
                    found = 1;
                    break;
                  }
                }
                if(found == 0) {
                  temp_to_remove.push_back(line);
                } 
              }
            }      
          }  
          if(no_intersection[group[line]] == 1) {done[group[line]] = 1;}
        }
        for(int line = 0; line < lines_to_check.size(); line++) {
          int found = 0;
          for(int idx = 0; idx < temp_to_remove.size(); idx++) {
            if(line == temp_to_remove[idx]) {
              found = 1;
              break;
            }
          }
          if(found == 0) {
            cells.push_back(trapezoids[line]);
          } 
        }

        if(done[0] && done[1] && done[2]){ break;}
      }
    }
    return cells;
}

/* calculates the area occupied by the polygon */
float polygon_area(std::vector<POINT> vertices, int vertices_num) {
    float area = 0;

    if(vertices_num % 2 !=0 )
        vertices.push_back(vertices[0]);

    for(int i = 0; i < vertices_num; i += 2)
        area += vertices[i+1].x*(vertices[i+2].y-vertices[i].y) + vertices[i+1].y*(vertices[i].x-vertices[i+2].x);
 
    area = area/2;
    return area;
}

/* if there are any overlaping between the cells, merge them together */
std::vector< std::vector<POINT> > merge_polygons(std::vector<std::vector<POINT>> cells) {
    POINT temp_point;
    std::vector<int> quads_to_remove;
    std::vector< std::vector<POINT> > quads_to_add;
    std::vector<POINT> temp1;
    std::vector<POINT> temp2;
    std::vector<POINT> new_quad;
    int area1, area2, area3;

    for(int cell1 = 0; cell1 < cells.size(); cell1++) {
      for(int cell2 = 0; cell2 < cells.size(); cell2++) {
        if(cell1 != cell2) {
          if(cells[cell1][0].x == cells[cell2][0].x && cells[cell1][1].x == cells[cell2][1].x) {
          
            temp1 = cells[cell1];
            // add the first point to the back
            temp1.push_back(cells[cell1][0]);
            temp2 = cells[cell2];
            temp2.push_back(cells[cell2][0]);
            area1 = polygon_area(temp1, 4);
            area2 = polygon_area(temp2, 4);

            temp_point.x = temp1[0].x;
            temp_point.y = min(temp1[0].y, temp2[0].y);
            new_quad.push_back(temp_point);
            temp_point.x = temp1[1].x;
            temp_point.y = min(temp1[1].y, temp2[1].y);
            new_quad.push_back(temp_point);
            temp_point.x = temp1[1].x;
            temp_point.y = max(temp1[2].y, temp2[2].y);
            new_quad.push_back(temp_point);
            temp_point.x = temp1[0].x;
            temp_point.y = max(temp1[3].y, temp2[3].y);
            new_quad.push_back(temp_point);
            temp_point.x = temp1[0].x;
            temp_point.y = min(temp1[0].y, temp2[0].y);
            new_quad.push_back(temp_point);
            area3 = polygon_area(new_quad, 4);

            if(area1 + area2 >= area3) {
              quads_to_remove.push_back(cell1);
              quads_to_remove.push_back(cell2);
              quads_to_add.push_back(new_quad);
            }
            
            temp1.clear();
            temp2.clear();
            new_quad.clear();
          }
        }
      } 
    }
    
    sort(quads_to_remove.begin(), quads_to_remove.end());

    quads_to_remove.erase(unique(quads_to_remove.begin(), quads_to_remove.end()), quads_to_remove.end());

    for(int quad = 0; quad < quads_to_remove.size(); quad ++) {
      cells.erase(cells.begin() + quads_to_remove[quad] - quad); //-quad because after deletion the indices shift
    }

    for(int quad = 0; quad < quads_to_add.size(); quad ++) {
      cells.push_back(quads_to_add[quad]);
    }     

    quads_to_remove.clear();
    for(int quad1 = 0; quad1 < cells.size(); quad1 ++) {
      for(int quad2 = quad1 + 1; quad2 < cells.size(); quad2 ++) {
        int duplicate = 1;
        for(int point = 0; point < cells[quad1].size(); point ++) {
          if((cells[quad1][point].x != cells[quad2][point].x) || (cells[quad1][point].y != cells[quad2][point].y)) {
            duplicate = 0;
            break;
          }
        }
        if (duplicate) {quads_to_remove.push_back(quad2);}
      }
    } 

    sort(quads_to_remove.begin(), quads_to_remove.end());
    quads_to_remove.erase(unique(quads_to_remove.begin(), quads_to_remove.end()), quads_to_remove.end());
 
    for(int quad = 0; quad < quads_to_remove.size(); quad ++) {
      cells.erase(cells.begin() + quads_to_remove[quad] - quad); //-quad because after deletion the indices shift
    } 

    //One more pass to remove extra quads generated because of cross - segments
    quads_to_remove.clear();
    for(int quad1 = 0; quad1 < cells.size(); quad1 ++) {
      for(int quad2 = 0; quad2 < cells.size(); quad2 ++) {
        if(quad1 != quad2 && cells[quad1][0].x == cells[quad2][0].x && cells[quad1][1].x == cells[quad2][1].x) { 
          if((cells[quad1][0].y <= cells[quad2][0].y) && (cells[quad1][1].y <= cells[quad2][1].y)
              && (cells[quad1][2].y >= cells[quad2][2].y) && (cells[quad1][3].y >= cells[quad2][3].y)) {      
              quads_to_remove.push_back(quad2);
          }
        } 
      }
    } 

    sort(quads_to_remove.begin(), quads_to_remove.end());
    quads_to_remove.erase(unique(quads_to_remove.begin(), quads_to_remove.end()), quads_to_remove.end());
 
    for(int quad = 0; quad < quads_to_remove.size(); quad ++) {
      cells.erase(cells.begin() + quads_to_remove[quad] - quad); //-quad because after deletion the indices shift
    }

    return cells;
}

/* adding the boundaries to the outside cells */
std::vector<std::vector<POINT>> boundary_cells(std::vector<POINT> boundary, std::vector<std::vector<POINT>> cells, std::vector<POINT> sorted_vertices, float y_limit_lower, float y_limit_upper) {
    
    POINT temp_point;
    std::vector<POINT> new_quad;

    if(boundary[0].x != sorted_vertices[0].x) {
      new_quad.clear();

      new_quad.push_back(boundary[0]);

      temp_point.x = sorted_vertices[0].x;
      temp_point.y = y_limit_lower;
      new_quad.push_back(temp_point);   

      temp_point.x = sorted_vertices[0].x;
      temp_point.y = y_limit_upper;
      new_quad.push_back(temp_point); 

      new_quad.push_back(boundary[3]);

      cells.push_back(new_quad);
    }

    if(boundary[1].x != sorted_vertices[sorted_vertices.size()-1].x) {
      new_quad.clear();

      temp_point.x = sorted_vertices[sorted_vertices.size()-1].x;
      temp_point.y = y_limit_lower;
      new_quad.push_back(temp_point); 

      new_quad.push_back(boundary[1]);

      new_quad.push_back(boundary[2]);

      temp_point.x = sorted_vertices[sorted_vertices.size()-1].x;
      temp_point.y = y_limit_upper;
      new_quad.push_back(temp_point);

      cells.push_back(new_quad);
    }
    return cells;
}

/* creates vertices on the borders of the neighbouring cells and in the middle of each cell
then connects the applicable vertices together to create a graph */
tuple <std::vector<POINT>, std::vector<POINT>> get_graph(std::vector< std::vector<POINT> > cells) {
    std::vector<int> same_boundary;
    std::vector<POINT> graph_vertices;
    std::vector<POINT> graph_edges;
    POINT centroid_vertex;
    POINT curr_centroid_vertex;
    POINT temp_edge_middle;
    POINT temp_point;
    std::vector<POINT> temp_points;
    int inside;
    int place; 
    int place1;
    int place2;
    int use; 
    int n;

    // for each quad cell find the cells that have the same boundary --> find neigbour cells
    for(int cell1 = 0; cell1 < cells.size(); cell1 ++) {
      same_boundary.clear();
      //compare to the rest of the cells if it is not the same cell
      for(int cell2 = 0; cell2 < cells.size(); cell2 ++) { 
        if(cell1 != cell2) {
          if((cells[cell1][1].x == cells[cell2][0].x) && 
            ((cells[cell1][2].y == cells[cell2][0].y || cells[cell1][2].y == cells[cell2][3].y) ||
            (cells[cell1][1].y == cells[cell2][0].y || cells[cell1][1].y == cells[cell2][3].y))) {
            same_boundary.push_back(cell2);
          }
        }
      }

      temp_points.clear();
      for(int pt = 0; pt < 4; pt++) {temp_points.push_back(cells[cell1][pt]);}
      centroid_vertex = centroid(temp_points);
      inside = 0;
      for(int vertex = 0; vertex < graph_vertices.size(); vertex++) {
        if(centroid_vertex.x == graph_vertices[vertex].x && centroid_vertex.y == graph_vertices[vertex].y) { 
          inside = 1;
          place = vertex;
          break;
        }
      }
      if(inside == 0) {
        graph_vertices.push_back(centroid_vertex); 
        place = -1;
      }

      if(same_boundary.size() == 1) {
        temp_points.clear();
        temp_points.push_back(cells[cell1][1]);
        temp_points.push_back(cells[cell1][2]);
        temp_edge_middle = centroid(temp_points);
        graph_vertices.push_back(temp_edge_middle);
        n = graph_vertices.size() - 1;
      
        if(place != -1) {
          temp_point.x = place;
          temp_point.y = n;
          graph_edges.push_back(temp_point);
        }
        else {
          temp_point.x = n-1;
          temp_point.y = n;
          graph_edges.push_back(temp_point);
        }

        temp_points.clear();
        for(int pt = 0; pt < 4; pt++) {temp_points.push_back(cells[same_boundary[0]][pt]);}
        curr_centroid_vertex = centroid(temp_points);
        inside = 0;
        for(int vertex = 0; vertex < graph_vertices.size(); vertex++) {
          if(curr_centroid_vertex.x == graph_vertices[vertex].x && curr_centroid_vertex.y == graph_vertices[vertex].y) { 
            inside = 1;
            place2 = vertex;
            break;
          }
        }
        if(inside == 0) { 
          place2 = -1;
        }
        if(place2 == -1) {
          graph_vertices.push_back(curr_centroid_vertex);
          temp_point.x = n;
          temp_point.y = n + 1;
          graph_edges.push_back(temp_point);
        }
        else {
          temp_point.x = n;
          temp_point.y = place2;
          graph_edges.push_back(temp_point);
        }
      }

      else if(same_boundary.size() > 1) {
        n = graph_vertices.size() - 1;
        if(place != -1) { use = place; }
        else { use = n; }
        for(int i = 0; i < same_boundary.size(); i ++){
          temp_points.clear();
          for(int pt = 0; pt < 4; pt++) {temp_points.push_back(cells[same_boundary[i]][pt]);}
          curr_centroid_vertex = centroid(temp_points);
          temp_points.clear();
          temp_points.push_back(cells[same_boundary[i]][0]);
          temp_points.push_back(cells[same_boundary[i]][3]);
          temp_edge_middle = centroid(temp_points);
          graph_vertices.push_back(temp_edge_middle);
          place1 = graph_vertices.size() - 1;
          inside = 0;
          for(int vertex = 0; vertex < graph_vertices.size(); vertex++) {
            if(curr_centroid_vertex.x == graph_vertices[vertex].x && curr_centroid_vertex.y == graph_vertices[vertex].y) { 
              inside = 1;
              place2 = vertex;
            }
          }
          if(inside == 0) {
            graph_vertices.push_back(curr_centroid_vertex);
            place2 = graph_vertices.size() - 1;
          }
          temp_point.x = use;
          temp_point.y = place1;
          graph_edges.push_back(temp_point);
          temp_point.x = place1;
          temp_point.y = place2;
          graph_edges.push_back(temp_point);
        } 
      }
    }
    return make_tuple(graph_edges, graph_vertices);
}

/* finds the euclidean distance between two points  */
float find_distance(POINT pt1, POINT pt2) {
    return float (sqrt(pow((pt1.x - pt2.x),2) + pow(pt1.y-pt2.y,2))); 
}

/* adds the start and the target points to the graph for each robot and
sends back a specific graph for that robot */
tuple <std::vector<POINT>, std::vector<POINT>> add_start_end(std::vector<POINT> graph_vertices, std::vector<POINT> graph_edges, POINT start_point, POINT end_point, std::vector<std::vector<POINT>> obstacles){
    // start point
    int min_ind = -1; 
    float min = INFINITY;
    float dist;
    int m;

    POINT temp_point;
    SEGMENT temp_segment;

    for(int vertex = 0; vertex < graph_vertices.size(); vertex ++) {
      temp_segment.a = start_point;
      temp_segment.b = graph_vertices[vertex];
      // find the closest vertex in the graph to the start point
      if(check_obstruction(obstacles, temp_segment)) {
        dist = find_distance(graph_vertices[vertex], start_point);
        if(dist < min) {
          min = dist;
          min_ind = vertex;
        } 
      }
    }
    // there is probably an obsticale at the start point <- use another map or decrease inflation
    if(min_ind == -1){
      throw std::logic_error( "THE START POINT IS UNREACHABLE <- try reducing inflation value" );
    }

    graph_vertices.push_back(start_point);
    m = graph_vertices.size()-1;
    temp_point.x = min_ind;
    temp_point.y = m;
    graph_edges.push_back(temp_point);

    // target point
    min_ind = -1; 
    min = INFINITY;

    for(int vertex = 0; vertex < graph_vertices.size(); vertex ++) {
      temp_segment.a = end_point;
      temp_segment.b = graph_vertices[vertex];
      if(check_obstruction(obstacles, temp_segment)) {
        dist = find_distance(graph_vertices[vertex], end_point);
        if(dist < min) {
          min = dist;
          min_ind = vertex;
        } 
      }
    }
    if(min_ind == -1){
      throw std::logic_error( "THE END POINT IS UNREACHABLE <- try reducing inflation value"  );
    }
    graph_vertices.push_back(end_point);
    m = graph_vertices.size()-1;
    temp_point.x = min_ind;
    temp_point.y = m;
    graph_edges.push_back(temp_point);

    return make_tuple(graph_edges, graph_vertices);
}

/* create the graph from the vertices and edges */
std::vector< std::vector<int> > graph_construction(std::vector<POINT> graph_vertices, std::vector<POINT> graph_edges) {
    std::vector< std::vector<int> > graph;
    for(int vertex = 0; vertex < graph_vertices.size(); vertex ++) {
      std::vector<int> empty;
      graph.push_back(empty);
      for (POINT &edge : graph_edges){
        if(edge.x == vertex){
          graph[vertex].push_back(edge.y);
        }
        else if(edge.y == vertex){
          graph[vertex].push_back(edge.x);
        }
      }
    }
    return graph;
}

/* a function used by bfs to back trace all the way to the beginning of the
path once the target has been found -> returns a path that begins with the start point  */
std::vector<int> backtrace(std::vector<int> parent, int start, int end) {
    std::vector<int> path;
    path.push_back(end);

    while (path[path.size()-1] != start) {
        path.push_back(parent[path[path.size()-1]]);
    }
    std::reverse(path.begin(),path.end());

    return path;
}

/* function used by bfs to check if current point is already in the path 
takes the point and the path and returns true if not found*/
bool visit_check(int point, vector<int>& path){
  for (int i = 0; i < path.size(); i++){
    if (path[i] == point){
      return false;
    }
  }
  return true;
}

/* Breadth First Search on a graph with a given Source and Target */
std::vector<int> bfs(std::vector< std::vector<int> > graph, int source, int target) {
  std::vector<int> path;
  std::vector<int> visited;
  std::vector<int> parent;
  std::vector<int> queue;
  int current;

  for(int node = 0; node < graph.size(); node++){
    visited.push_back(0);
    parent.push_back(-1);
  }

  queue.push_back(source);
  while(queue.size() > 0){
    current = queue[0];
    queue.erase(queue.begin());
    if (current == target) {
      path = backtrace(parent, source, target);
      return path;
    }
    for (int neighbor : graph[current]){
      if(visited[neighbor] == 0){
        visited[neighbor] = 1;
        parent[neighbor] = current;
        queue.push_back(neighbor);
      }
    }
  }
  path.push_back(-1); 
  return path;
}
 
/* a bfs version that keeps searching for all the available paths
up till a defined limit */
std::vector<std::vector<int>> bfs_multiple(std::vector< std::vector<int> > graph, int source, int target, int path_count){
	queue<vector<int> > point_queue;
  // for all the paths found
  std::vector<std::vector<int>>  paths;
	vector<int> path;
  int last_added;

	path.push_back(source);
	point_queue.push(path);
	while (!point_queue.empty()) {
		path = point_queue.front();
		point_queue.pop();
		last_added = path[path.size() - 1];

		// if target found , save to the paths found list
		if (last_added == target){
			paths.push_back(path);
      if(paths.size()>path_count){
        return paths;
      }
    }
		for (int i = 0; i < graph[last_added].size(); i++) {
			if (visit_check(graph[last_added][i], path)) {
				vector<int> newpath(path);
				newpath.push_back(graph[last_added][i]);
				point_queue.push(newpath);
			}
		}
	}
  return paths;
}

/* one of the options to optimize the path: a brute force method to connect all the points
int the path with each other, and keep the ones that are not intersecting with anything */
std::vector<std::vector<int>>  optimize_graph(std::vector<int> my_path, std::vector<POINT> new_graph_vertices, std::vector<std::vector<POINT>> obstacles){
    SEGMENT temp_path;
    SEGMENT temp_obs;
    std::vector< std::vector<int> >  optimized_graph;
    POINT inter_result;
    for(int vertex = 0; vertex < new_graph_vertices.size();vertex++){
      std::vector<int> empty;
      optimized_graph.push_back(empty);
      for(int vertex_2 = 0 ;vertex_2<new_graph_vertices.size();vertex_2++){
        bool break_off = false;
        if(vertex_2!=vertex ){
          temp_path.a = {new_graph_vertices[vertex].x,new_graph_vertices[vertex].y};
          temp_path.b = {new_graph_vertices[vertex_2].x,new_graph_vertices[vertex_2].y};
          for(int obs = 0 ; obs< obstacles.size();obs++){
            for(int pt = 0 ; pt< obstacles[obs].size()-1;pt++){
              temp_obs.a = {obstacles[obs][pt].x,obstacles[obs][pt].y};
              temp_obs.b = {obstacles[obs][pt+1].x,obstacles[obs][pt+1].y};
              inter_result = segment_intersection(temp_obs,temp_path);
              if(inter_result.x !=-1){
                break_off = true;
                break;
              }
            }
            if(break_off){
              break;
            }
          }
          if(break_off){
            continue;
          }
          optimized_graph[vertex].insert(optimized_graph[vertex].begin(),vertex_2);
        }
      }
    }
    return optimized_graph;
}

/* an optimization method that looks ahead for each point [up to a defined limit].
it compares the distances between the current point and the number of points decided by the
look ahead number to find out which one has the lower distance. the function uses a cost discount [tunable]
that the further appart [in index] the points are the cheaper it is, to promote skipping points if needed*/
std::vector<int> look_ahead_optimize(std::vector<int> my_path, std::vector<POINT> graph_vertices, std::vector<std::vector<POINT>> obstacles, float look_ahead, float gamma){
  SEGMENT temp_path;
  SEGMENT temp_obs;
  float distance= 0.0 ;
  float best_distance;
  int best_point;
  int next_point;
  bool break_loop;
  int cap = 0;
  float gamma_i;
  float angle;
  POINT inter_result;
  const double pi = boost::math::constants::pi<double>();

  // to guarentee that look ahead won't exceed the path length
  if(look_ahead>my_path.size()-1){look_ahead=my_path.size()-1;}

  std::vector<int> optimized_path;
  // add start point
  optimized_path.push_back(my_path[0]);
  for(int i = 0;i<my_path.size()-1;i++){
    gamma_i = 1;
    best_distance = INFINITY;
    next_point = i+1;
    best_point = my_path[next_point];
    // to insure that the look ahead wont exceed the size of the path
    if(i<=my_path.size()-look_ahead-1){cap = look_ahead;}else{cap= my_path.size()-1-i;}
    for(int j=1;j<=cap;j++){
      break_loop= false;
      distance = sqrt(pow(graph_vertices[my_path[i]].x - graph_vertices[my_path[i+j]].x,2) + pow(graph_vertices[my_path[i]].y - graph_vertices[my_path[i+j]].y,2));
      // apply a discount to the distance the further ahead you look
      distance = gamma_i * distance ;
      angle = atan2(graph_vertices[my_path[i]].y-graph_vertices[my_path[i+j]].y,graph_vertices[my_path[i]].x-graph_vertices[my_path[i+j]].x)*180/pi;
      if(distance <= best_distance){
        temp_path.a = {graph_vertices[my_path[i]].x, graph_vertices[my_path[i]].y};
        temp_path.b = {graph_vertices[my_path[i+j]].x, graph_vertices[my_path[i+j]].y};  
        // if better distance found , check collision      
        for(int obs = 0 ; obs< obstacles.size();obs++){
          for(int pt = 0 ; pt< obstacles[obs].size()-1;pt++){
            temp_obs.a = {obstacles[obs][pt].x,obstacles[obs][pt].y};
            temp_obs.b = {obstacles[obs][pt+1].x,obstacles[obs][pt+1].y};
            inter_result = segment_intersection(temp_obs,temp_path);
            if(inter_result.x !=-1){
              break_loop = true;
              break;
            }
          }
          if(break_loop){
            break;
          }
        }
        if(break_loop){
          continue;
        }
        else{
          best_distance = distance;
          best_point = my_path[j+i];
          next_point = i+j;
        }      
      }
      gamma_i*=gamma;
    }
    
    i = next_point-1;
    optimized_path.push_back(my_path[next_point]);
  }
  return optimized_path;
}

/* converts the graph indecies to actual x and y coordinates based on the recieved graph */
std::vector<robotPos> index_to_coordinates(std::vector<int> index_path, std::vector<POINT> coordinates) {
  std::vector<robotPos> path_points;
  robotPos temp_pt;
  for (unsigned i=0; i<index_path.size(); i++) {
    temp_pt = {coordinates[index_path[i]].x, coordinates[index_path[i]].y,-1};
    path_points.push_back(temp_pt);
  }
  return path_points;
}

/* creates boundary lines from the points of the boundaries */
std::vector<SEGMENT> get_boundary_lines(std::vector<POINT> boundary){
  std::vector<SEGMENT> boundary_lines;
  SEGMENT temp_s;
  for(int j=0; j < boundary.size();j++){
    int inde = (j+1) % boundary.size();
    temp_s.a = {boundary[j].x,boundary[j].y};
    temp_s.b = {boundary[inde].x,boundary[inde].y}; 
    boundary_lines.push_back(temp_s);
  }
  return boundary_lines;
}

/* applies an offset to the endpoints based on where the gate is located [east, west, north, south]
helps get to the end point after applying inflation on the objects and borders. it also adds
space in the width dimension between the end points for each robot */
std::vector<POINT> offset_end_points (std::vector<SEGMENT> boundary_lines, int robots_number, std::vector<POINT>end_point, float offset_gate_width, float offset_away_from_gate){
  float smallest_distance= INFINITY;
  int closest_boarder=-1;
  for (int j=0 ; j<boundary_lines.size();j++){
    float A = end_point[0].x - boundary_lines[j].a.x;
    float B = end_point[0].y - boundary_lines[j].a.y;
    float C = boundary_lines[j].b.x - boundary_lines[j].a.x;
    float D = boundary_lines[j].b.y - boundary_lines[j].a.y;
    float dist = abs(A * D - C * B) / sqrt(C * C + D * D);
    if (dist < smallest_distance){
      smallest_distance = dist;
      closest_boarder = j;
    }
  }
  std::vector<POINT> end_points;
  for(int rob = 0; rob < robots_number;rob++){
    end_points.push_back(end_point[0]);
  }

  for(int rob = 0 ; rob < robots_number; rob++){
    // 0  horizontal top
    if(closest_boarder == 0){
      end_points[rob] = {end_points[rob].x+offset_gate_width,end_points[rob].y+offset_away_from_gate};
    }
    // 1  vertical right
    if(closest_boarder == 1){
      end_points[rob] = {end_points[rob].x-offset_away_from_gate,end_points[rob].y +offset_gate_width};
    }
    // 2  horizontal bottom
    if(closest_boarder == 2){
      end_points[rob] = {end_points[rob].x+offset_gate_width,end_points[rob].y-offset_away_from_gate};
    }
    // 3  vertical left
    if(closest_boarder == 3){
      end_points[rob] = {end_points[rob].x+offset_away_from_gate,end_points[rob].y +offset_gate_width};
    }
    offset_gate_width += 0.05;
  }
  
  return end_points;
}

/* for debugging purposes */
void print_data(std::vector<POINT> boundary, POINT start_point, POINT end_point, std::vector< std::vector<POINT> > obstacles, std::vector<POINT> graph_vertices, std::vector<std::vector<int>> graph, std::vector<int> path, std::vector<int> optimized_path, std::vector<robotPos> path_points) {     
  std::cout<<"\n>>>> Border postion:"<<std::endl;
  for(int i = 0; i < boundary.size(); i++){
   cout<< "x=" << boundary[i].x << " y=" << boundary[i].y << endl;
  }
  std::cout<<"\n>>>> Starting point:"<<std::endl;
   cout<< "x=" << start_point.x << " y=" << start_point.y << " theta="<< start_point.theta << endl;

  std::cout<<"\n>>>> End point:"<<std::endl;
  cout<< "x=" << end_point.x << " y=" << end_point.y << endl;
  std::cout<<"\n>>>> Obstacles:"<<std::endl;
  for(int i = 0; i < obstacles.size(); i++){
   std::cout<<"\nObstacle "<< i << endl;
   for(int j = 0; j < obstacles[i].size(); j++){
     cout<< "x=" << obstacles[i][j].x << " y=" << obstacles[i][j].y << endl;
   }
  }    
  cout << endl;
  cout <<"GRAPH VERTICES: "<< endl; 
  for(int i = 0; i < graph_vertices.size(); i++){
    cout << "(" << graph_vertices[i].x << "," << graph_vertices[i].y;
    if(i == graph_vertices.size() - 1) { cout << ") "; }
    else cout << "), ";
  }
  cout << endl;
  cout << endl;

  cout <<"GRAPH: "<< endl;
  for(int j = 0; j < graph.size(); j++){
    cout << "(";
    for(int k = 0; k < graph[j].size(); k++) {  
      cout << graph[j][k];
      if(k != (graph[j].size() - 1)) {cout << ", ";}
    }
    if(j == (graph.size() - 1)) {cout << ") "; }
    else cout << "), ";
  }
  cout << endl;
  cout << endl;

  cout <<"PATH: "<< endl; 
  for(int i = 0; i < path.size(); i++){
    cout << path[i];
    if(i != path.size() - 1) { cout << ", "; }
  }
  cout << endl;
  cout << endl;

  cout <<"OPTIMIZED PATH: "<< endl; 
  for(int i = 0; i < optimized_path.size(); i++){
    cout << optimized_path[i];
    if(i != optimized_path.size() - 1) { cout << ", "; }
  }
  cout << endl;
  cout << endl;

  cout <<"OPTIMIZED PATH WITH COORDINATES: "<< endl; 
  for(int i = 0; i < path_points.size(); i++){
    cout << "x=" << path_points[i].x << " y=" << path_points[i].y << " theta=" << path_points[i].th << endl;
  }
  cout << endl;
  cout << endl;   
}