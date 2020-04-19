#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "fsm.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::sqrt;
using std::pow;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  double ref_velocity = 0.0; // start at 0 mph
  
  int lane = 1; // start in lane 1 (middle lane)
  
  // Initialize the state machine in the ready state  
  fsm finite_state_machine;
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_velocity, &lane, &finite_state_machine]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
  
          // Initialize spline points vectors
          vector<double> ptsx, ptsy;
		  
          /**
           *  Define a path made up of (x,y) points that the car will visit
           *  sequentially every .02 seconds
           */
               
          double target_speed = 49.5; //just under 50mph speed limit
  		  double lane_width = 4.0; //meters
		  double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          int previous_path_size = previous_path_x.size();
          int relevant_lane;
          
          if(previous_path_size > 0) {       
            car_s = end_path_s;         
          }
               
          // Get finite state machine state after transition
          string state = finite_state_machine.transition(sensor_fusion, car_s, car_d, lane, previous_path_size, car_speed);
          
          // Behavior is determined by the finite state machine state
          if (state == "Lane Keep") {
            relevant_lane = lane;
          }   
          else if (state == "Prepare Turn Right" ) {       
            relevant_lane = lane + 1;
          } 
          else if (state == "Prepare Turn Left" ) {        
            relevant_lane = lane - 1;
          } 
          else if (state == "Turn Right" ) {
            lane = lane+ 1;
            relevant_lane = lane;
          } 
          else if (state == "Turn Left" ) { 
            lane = lane - 1;
            relevant_lane = lane;  
          } 
 
          for (int i = 0; i < sensor_fusion.size(); i++) {
            // check if there's a car in the relevant lane          
            float d = sensor_fusion[i][6];
            if (d < (lane_width/2 + lane_width*relevant_lane+ lane_width/2) && d > ( lane_width/2 + lane_width*relevant_lane-lane_width/2)) {
              
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = distance(0,0,vx,vy) ;
              double check_s = sensor_fusion[i][5] ;
              check_s += ((double)previous_path_size * 0.02 * check_speed); // Project s value forward
              
              // check if that car is in front and within buffer zone of 30 meters
              if((check_s > car_s) && ((check_s - car_s) < 30)) {    
                // Drop target speed to match the speed of the car
                target_speed = check_speed;
              }                
            }
          }
          // Adjust velocity based on the difference between current velocity and target velocity
          const double velocity_change = 0.3; //m/s how much the velocity can increase or decrease in 0.02 seconds
       
          if(ref_velocity < target_speed){
            ref_velocity += velocity_change; 
          }
          else if(ref_velocity > target_speed){       
            ref_velocity -= velocity_change;
          }
          
          /**
           *  Generate drivable trajectories using splines
           */
          
          // Use previous path points for the spline if available otherwise use current position and angle to estimate previous position.
          if (previous_path_size < 2) {  
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } 
          else {              
            ref_x = previous_path_x[previous_path_size - 1];
            ref_y = previous_path_y[previous_path_size - 1];       
            double ref_x_prev = previous_path_x[previous_path_size - 2];
            double ref_y_prev = previous_path_y[previous_path_size - 2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          //In Frenet add eveny spaced 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, (lane_width / 2 + lane_width * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60,(lane_width / 2 + lane_width * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90,(lane_width / 2 + lane_width * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (int i = 0; i< ptsx.size(); i++) {
            // shift car reference angle to 0 degrees (shift in rotation) 
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);           
          } 
          
          // Initialize Spline 
          tk::spline s; 
          s.set_points(ptsx,ptsy);
          
          vector<double> next_x_vals, next_y_vals;
          
          //Start the trajectory with previous path points
          for (int i = 0; i < previous_path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);    
          } 
          
          // Calculate how to break up spline points to travel at reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = distance(0,0, target_x, target_y);
          
          double x_add_on = 0;
              
          // fill up the rest the trajectory with spline points
          for (int i =0; i < 50- previous_path_size; i++) { 
            
            double N = target_dist/(0.02 * ref_velocity / 2.24);
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point; 
            
            // tranform back to global coordinates
            x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            // add start x and y
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } 
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}