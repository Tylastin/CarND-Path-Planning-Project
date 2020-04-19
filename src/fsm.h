#ifndef FSM_H
#define FSM_H


#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// finite state machine for self-driving car behavior planning
class fsm {
  public:
    // Constructor
    fsm();
    
    // Destructor
    ~fsm();
  
    // Transition function
    // Determines lowest cost behavior and returns lowest cost state (string)
    string transition(vector<vector<double>> sensor_fusion, double car_s, double car_d, int lane, int previous_path_size, double car_speed);
    
  private:
    string state_; // state variable
    
    // Cost Functions

    // will prevent the car from switching lanes before it has centered itself after a lane switch
    double feasability_cost_function(vector<vector<double>> sensor_fusion, double car_s, double car_d, int lane, int previous_path_size, double car_speed);

  // Penalizes behavior that would likely result in collision
    double collision_cost_function(vector<vector<double>> sensor_fusion, double car_s, double car_d, int lane, int previous_path_size, double car_speed);

  // Penalizes behavior that would result in moving too far below the speed limit
    double speed_cost_function(vector<vector<double>> sensor_fusion, double car_s, double car_d, int lane, int previous_path_size, double car_speed);

  // Calculates distance between two points
  double distance(double x1, double y1, double x2, double y2);
};

#endif /* FSM_H */
