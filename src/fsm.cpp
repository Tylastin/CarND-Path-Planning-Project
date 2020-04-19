#include "fsm.h"

#include <iostream>
#include <math.h> 


/**
 * Constructor.
 */
fsm::fsm() {
  state_ = "Ready";
}

/**
 * Destructor.
 */
fsm::~fsm() {}

// Cost functions 

double fsm::feasability_cost_function(vector<vector<double>> sensor_fusion, double car_s, double car_d, int lane, int previous_path_size, double car_speed) { 
  
  double lane_width = 4.0; 
  double cost = 0;
  double max_distance_from_center = 1.2 * lane_width;
  double lane_d = lane_width / 2 + lane * lane_width;
  if (fabs(lane_d - car_d) > max_distance_from_center) {
    cost = 1;
  } 
  return cost; 
}

double fsm::collision_cost_function(vector<vector<double>> sensor_fusion, double car_s, double car_d, int lane, int previous_path_size, double car_speed) { 

  double lane_width = 4.0; 
  double cost;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    // check if there's a car in lane being evaluated      
    float d = sensor_fusion[i][6];
    if (d < (lane_width / 2 + lane_width * lane + lane_width / 2) && d > (lane_width / 2 + lane_width * lane - lane_width / 2)) {

      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = distance(0,0,vx,vy) ;
      double check_s = sensor_fusion[i][5] ;

      check_s += ((double)previous_path_size * 0.02 *check_speed); // Project s value forward  
      // if gap isn't big enough to ensure safety add huge penalty
     
      // check if car is in front or behind
      if (check_s > car_s) {      
        if(((check_s-car_s) + (check_speed - car_speed)) < 25) {
        cost = 1;
      	}    
      	else {
      		cost = 0;
      	}    
      } 
      else {
        if(((car_s - check_s) + (car_speed - check_speed)) <  25) {
        	cost = 1;
      	}    
      	else {
      		cost = 0;
      	}
      }  
    }
  }
  return cost; 
}

double fsm::speed_cost_function(vector<vector<double>> sensor_fusion, double car_s, double car_d, int lane, int previous_path_size, double car_speed) { 

  double target_speed = 49.5; // mph just under speed limit
  double lane_width = 4.0; // 
  double cost = 0;
  for (int i = 0; i < sensor_fusion.size(); i++) {          
    // check if there's a car in the lane being evaluated      
    float d = sensor_fusion[i][6];    
    if (d < (lane_width / 2 + lane_width * lane + lane_width / 2) && d > (lane_width / 2 + lane_width * lane - lane_width / 2)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = distance(0,0,vx,vy) ;
      double check_s = sensor_fusion[i][5] ;

      check_s += ((double)previous_path_size * 0.02 * check_speed); // Project s value forward
      // check if that car within buffer zone of 40 meters in front and 5m behind
      // add cost depending on how much slower or faster than the target speed the car is going 
      if((check_s > car_s - 5) && ((check_s-car_s) < 40)) { 
        double check_cost = fabs(target_speed - check_speed)/target_speed;
        if (check_cost > cost) {
          cost = check_cost; 
        }
      }              
    }
  }
  return cost;
} 

string fsm::transition(vector<vector<double>> sensor_fusion, double car_s, double car_d, int lane, int previous_path_size, double car_speed) {
  double current_lane_collision_cost = collision_cost_function(sensor_fusion, car_s, car_d, lane, previous_path_size, car_speed);
  double current_lane_speed_cost = speed_cost_function(sensor_fusion, car_s, car_d, lane, previous_path_size, car_speed); 
  double current_lane_feasability_cost = feasability_cost_function(sensor_fusion, car_s, car_d, lane, previous_path_size, car_speed); 
  
  // weighted sum of cost functions
  double current_lane_total_cost = 30 * current_lane_collision_cost + 10 * current_lane_feasability_cost + 5 * current_lane_speed_cost;
  
  vector<int> lanes = {0,1,2};
  int potential_lane; 
  
  double potential_lane_speed_cost;
  double potential_lane_collision_cost;
  double potential_lane_total_cost;
  double potential_lane_feasability_cost;
  
  int optimal_lane = lane; 
  
  // Compares the cost of the current lane to the cost of the adjacent lanes
  for (int i = 0; i < lanes.size(); i++) { 
    
    if ( abs(lanes[i] - lane) == 1) {  //checks if lane is adjacent to current lane
      potential_lane = lanes[i];
      potential_lane_speed_cost = speed_cost_function(sensor_fusion, car_s, car_d, potential_lane, previous_path_size, car_speed); 
      potential_lane_collision_cost = collision_cost_function(sensor_fusion, car_s, car_d, potential_lane, previous_path_size, car_speed);
      potential_lane_feasability_cost = feasability_cost_function(sensor_fusion, car_s, car_d, potential_lane, previous_path_size, car_speed); 
        potential_lane_total_cost = 30 * potential_lane_collision_cost + 10 * potential_lane_feasability_cost + 5 * potential_lane_speed_cost; 

      // switches lanes if the cost would be reduced by at least 1. 
      if (potential_lane_total_cost + 2 < current_lane_total_cost) { 
        optimal_lane = potential_lane;
      } 
    } 
  } 
  int lane_diff = optimal_lane - lane;
  
  //checks current fsm state and optimal lane and transitions accordingly;
  if (state_ == "Ready") {
    state_ = "Lane Keep";
  }
  else if (state_ == "Lane Keep") {
    if (lane_diff > 0) {
      state_ = "Prepare Turn Right";     
    } 
    else if (lane_diff < 0) { 
      state_ = "Prepare Turn Left";
    } 
    else {
      state_ = "Lane Keep";
    }   
  } 
  else if (state_ == "Prepare Turn Left") {
    if (optimal_lane == lane - 1) {
      state_ = "Turn Left";
    } 
    else { 
      state_ = "Prepare Turn Left";
    } 
  } 
  else if (state_ == "Prepare Turn Right") {
    if (optimal_lane == lane + 1) {
      state_ = "Turn Right";
    } 
    else { 
      state_ = "Prepare Turn Right";
    } 
  } 
  else if (state_ == "Turn Left") {
    state_ = "Lane Keep";
  } 
  else if (state_ == "Turn Right") {
    state_ = "Lane Keep"; 
  } 
  else { 
    // invalid state
    std::cout << "invalid state";
  } 
  return state_;
}

// Calculate distance between two points
double fsm::distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
