#ifndef CAR_CONTROL_
#define CAR_CONTROL_

#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include "json.hpp"

// for convenience
using std::string;
using std::vector;

const double safe_distance = 20.0; // in meters
const double lane_width = 4.0; // in meters
const double max_safe_speed = 49.5; // mph

class Vehicle
{
private:
  
public:
  double s, d, vx, vy, speed;
  double lane;
  Vehicle(nlohmann::json sensor_fusion);
  ~Vehicle();
};

Vehicle::Vehicle(nlohmann::json sensor_fusion)
{
  s = sensor_fusion[5];
  d = sensor_fusion[6];
  vx = sensor_fusion[3];
  vy = sensor_fusion[4];

  if (d < 0) {
    lane = -1;
  }
  else if ((0 <= d) & (d < 4))
  {
    lane = 0;
  }
  else if ((0 <= 4) & (d < 8))
  {
    lane = 1;
  }
  else if ((0 <= 8) & (d < 12))
  {
    lane = 2;
  }
}

Vehicle::~Vehicle()
{
}

bool check_safe_distance(double car_s, double vehicle_s){
  /**
   * check_save_distance check if the car is at safe distance from other vehicles
   * @param car_s (double): ego car's s value
   * @param vehicle_s (double): detected vehicles's s value
   * @return check_safe_distance (bool): true if car is behind detected car and distance between them is 
   *                                    less than safe_distance
  */
  return (car_s < vehicle_s && vehicle_s - car_s < safe_distance);
}

double predict_vehicle_future_s(double v_s, double v_speed, int prev_size){
  /**
   * predict_vehicle_future_s estimates the s value of the vehicle in future based on its current speed
   * @param v_s (double): vehicle's current s value
   * @param v_speed (double): vehicle's current speed
   * @param prev_size (int): size of previous path
   * @return predicted speed of vehicle (double)
  */
  
  double retval = v_s + (double)prev_size * 0.02 * v_speed;
  return retval;
}

#endif // End CAR_CONTROL_def
