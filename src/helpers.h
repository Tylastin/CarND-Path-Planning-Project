#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

double distance(double x1, double y1, double x2, double y2);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Cost functions 

double collision_cost_function(vector<vector<double>> sensor_fusion, double car_s, double car_d, string current_state, int lane, int previous_path_size) { 
  //Penalizes behavior that would result in collision

  double lane_width = 4.0; // 
  double cost;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    // check if there's a car in lane being evaluated      
    float d = sensor_fusion[i][6];
    if (d < (lane_width/2 + lane_width*lane+ lane_width/2) && d > ( lane_width/2 + lane_width*lane-lane_width/2)) {

      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = distance(0,0,vx,vy) ;
      double check_s = sensor_fusion[i][5] ;

      check_s += ((double)previous_path_size*0.02*check_speed); // Project s value forward  
      // if gap isn't big enough to ensure safety add huge penalty
      if((fabs(check_s-car_s) < 30)) {
        cost = 1;
      }    
      else {
      cost = 0;
      }
    }
  }
  return cost; 
}

double speed_cost_function(vector<vector<double>> sensor_fusion, double car_s, double car_d, string current_state, int lane, int previous_path_size){ 
  //Penalizes inefficient behavior 
  double target_speed = 49.5; // mph just under speed limit
  double lane_width = 4.0; // 
  double cost = 0;
  for (int i = 0; i < sensor_fusion.size(); i++) {          
    // check if there's a car in the lane being evaluated      
    float d = sensor_fusion[i][6];    
    if (d < (lane_width/2 + lane_width*lane+ lane_width/2) && d > ( lane_width/2 + lane_width*lane-lane_width/2)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = distance(0,0,vx,vy) ;
      double check_s = sensor_fusion[i][5] ;

      check_s += ((double)previous_path_size*0.02*check_speed); // Project s value forward
      // check if that car within buffer zone of 40 meters in front and 5m behind
      // add cost depending on how much slower than the target speed the car is going 
      if((check_s > car_s-5) && ((check_s-car_s)<40)) { 
        double check_cost = (target_speed - check_speed)/target_speed;
        if (check_cost > cost) {
          cost = check_cost; 
        }
      }              

    }
  }
  return cost;
} 


// Transition function for the finite state machine. 
// Determines lowest cost behavior and returns lowest cost state (string)
string transition_function(vector<vector<double>> sensor_fusion, double car_s, double car_d, string current_state, int lane, int previous_path_size) {
  string next_state;
  double current_lane_collision_cost = collision_cost_function(sensor_fusion, car_s, car_d, current_state, lane, previous_path_size);
  double current_lane_speed_cost = speed_cost_function(sensor_fusion, car_s, car_d, current_state, lane, previous_path_size); 
  // weighted sum of cost functions
  double current_lane_total_cost = 10*current_lane_collision_cost + current_lane_speed_cost;
  vector<int> lanes = {0,1,2};
  int potential_lane; 
  double potential_lane_speed_cost;
  double potential_lane_collision_cost;
  double potential_lane_total_cost;
  int optimal_lane = lane; 
  // Compares the cost of the current lane to the cost of the adjacent lanes
  for (int i = 0; i < lanes.size(); i++) { 
    if ( abs(lanes[i]-lane)== 1){  //checks if lane is adjacent to current lane
      potential_lane = lanes[i];
      potential_lane_speed_cost = speed_cost_function(sensor_fusion, car_s, car_d, current_state, potential_lane, previous_path_size); 
      potential_lane_collision_cost = collision_cost_function(sensor_fusion, car_s, car_d, current_state, potential_lane, previous_path_size);
        potential_lane_total_cost = 10*potential_lane_collision_cost + potential_lane_speed_cost; 

      // switches lanes if the cost would be reduced by at least 0.05
      if (potential_lane_total_cost + 0.05 < current_lane_total_cost) { 
        optimal_lane = potential_lane;
      } 
    } 
  } 
  int lane_diff= optimal_lane - lane;
  
  //checks current fsm state and optimal lane and transitions accordingly;
  if (current_state == "Ready") {
    next_state = "Lane Keep";
  }
  else if (current_state == "Lane Keep") {
    if (lane_diff> 0) {
      next_state = "Prepare Turn Right";
      
    } 
    else if (lane_diff <0) { 
      next_state = "Prepare Turn Left";
    } 
    else {
      next_state = "Lane Keep";
    }
    
  } 
  else if (current_state == "Prepare Turn Left") {
    if (optimal_lane == lane-1) {
      next_state = "Turn Left";
    } 
    else { 
      next_state = "Prepare Turn Left";
    } 
  } 
  else if (current_state == "Prepare Turn Right") {
    if (optimal_lane == lane+1) {
      next_state = "Turn Right";
    } 
    else { 
      next_state = "Prepare Turn Right";
    } 
  } 
  else if (current_state == "Turn Left") {
    next_state = "Lane Keep";
  } 
  else if (current_state == "Turn Right") {
    next_state = "Lane Keep"; 
  } 
  else { 
    // invalid state
    std::cout << "invalid state";
  } 
  return next_state;
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

#endif  // HELPERS_H