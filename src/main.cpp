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
#include "car_control.cpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  int lane = 1;
  double ref_vel = 0.0; //mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
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
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          json msgJson;

          /**
           * This state machine contains below 4 states
           *1. too_close: this checks if the car detected via sensor_fusion is too close (< 30m close)
           *2. left_lane_free, right_lane_free: if above condition holds true, it will check whether the left or right lane is free and will set the value accordingly
           *3. prepare_lane_change: based on above both conditions true, will prepare to lane change
           *4. lane_change: when done preparing lane change, lane will be changed
          */

          int prev_size = previous_path_x.size();
          bool too_close = false;
          bool left_lane_free = false;
          bool right_lane_free = false;
          bool prepare_lane_change = false;
          bool lane_change = false;
          if (prev_size > 0) {
            // This means the vehicle has moved some place and doesn't depict the beginning of simulator
            car_s = end_path_s;
          }
          
          // Iterating through all the cars/vehicles detected by the sensor_fusion
          for(size_t i = 0; i < sensor_fusion.size(); i++)
          {
            Vehicle v(sensor_fusion[i]); // Creating struct for every vehicle to check diff conditions
            v.s = predict_vehicle_future_s(v.s, v.speed, prev_size);
            if (lane == v.lane) {
              v.s = predict_vehicle_future_s(v.s, v.speed, prev_size);
              if (check_safe_distance(car_s, v.s)) {
                too_close = true;
                prepare_lane_change = true;
              }
            }
          }

          if (prepare_lane_change) {
            for(size_t i = 0; i < sensor_fusion.size(); i++)
            {
              Vehicle v(sensor_fusion[i]);
              v.s = predict_vehicle_future_s(v.s, v.speed, prev_size);
              if (v.lane < lane && car_d - v.d > 4.0) {
                bool too_close_for_lane_shift = (v.s > car_s - safe_distance / 2) && (v.s < car_s + safe_distance / 2);
                left_lane_free = too_close_for_lane_shift ? false : true;
              }
              else if (v.lane > lane && v.d - car_d > 4.0)
              {
                bool too_close_for_lane_shift = (v.s > car_s - safe_distance / 2) && (v.s < car_s + safe_distance / 2);
                right_lane_free = too_close_for_lane_shift ? false : true;
              }

              if (left_lane_free || right_lane_free) {
                lane_change = true;
              }
            }
          }

          if (prepare_lane_change && !(lane_change)) {
            ref_vel -= 0.224 * 1.5; // Reducing speed since there's no space to lane change, but ego car is approaching nearest vehicle.
          }
          

          if (lane_change && left_lane_free) {
            lane -= 1;
            ref_vel -= 0.224 * 1.5;
          }
          else if (lane_change && right_lane_free)
          {
            lane += 1;
            ref_vel -= 0.224 * 1.5;
          }

          if (too_close) {
            ref_vel -= 0.224;
          }
          else if (ref_vel < max_safe_speed)
          {
            ref_vel += 0.224;
          }

        // List of widely spaced (x, y) waypoints. These will be later interpolated
        // with a spline, filling it with more points which control speed.
        vector<double> pts_x;
        vector<double> pts_y;

        // Reference x, y, yaw state 
        double ref_x = car_x;
        double ref_y = car_y;
        double ref_yaw = deg2rad(car_yaw);

        // If previous size is almost empty, use the car as a starting reference
        if (prev_size < 2) {
          double prev_car_x = car_x - cos(car_yaw);
          double prev_car_y = car_y - sin(car_yaw);

          pts_x.push_back(prev_car_x); pts_x.push_back(car_x);
          pts_y.push_back(prev_car_y); pts_y.push_back(car_y);
        }
        // Use the previous path's end points as starting reference
        else {
          ref_x = previous_path_x[prev_size - 1];
          ref_y = previous_path_y[prev_size - 1];

          double ref_x_prev = previous_path_x[prev_size - 2];
          double ref_y_prev = previous_path_y[prev_size - 2];
          ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

          pts_x.push_back(ref_x_prev); pts_x.push_back(ref_x);
          pts_y.push_back(ref_y_prev); pts_y.push_back(ref_y);
        }

        // In Frenet coordinates, add evenly 30m spaced points ahead of the starting reference
        vector<double> next_wp0 = getXY(car_s + 30, (lane_width * lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp1 = getXY(car_s + 60, (lane_width * lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp2 = getXY(car_s + 90, (lane_width * lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);

        pts_x.push_back(next_wp0[0]); pts_x.push_back(next_wp1[0]); pts_x.push_back(next_wp2[0]);
        pts_y.push_back(next_wp0[1]); pts_y.push_back(next_wp1[1]); pts_y.push_back(next_wp2[1]);

        // Rototranslate into car's reference system to make the math easier
        for (size_t i = 0; i < pts_x.size(); ++i) {
          double shift_x = pts_x[i] - ref_x;
          double shift_y = pts_y[i] - ref_y;
          pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
          pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
        }

        // Create a spline
        tk::spline s;
        s.set_points(pts_x, pts_y);
        
        // Start with all previous points from last time
        for (size_t i = 0; i < previous_path_x.size(); ++i) {
          next_x_vals.push_back(previous_path_x[i]);
          next_y_vals.push_back(previous_path_y[i]);
        }

        // Calculate how to break up spline points to travel at reference velocity
        double target_x = 30.0;
        double target_y = s(target_y);
        double target_dist = sqrt(target_x * target_x + target_y * target_y);

        double x_add_on = 0.0;

        for (size_t i = 1; i <= 30 - previous_path_x.size(); ++i) {

          double N = target_dist / (0.02 * ref_vel / 2.24);
          double x_point = x_add_on + target_x / N;
          double y_point = s(x_point);

          x_add_on = x_point;

          double x_ref = x_point;
          double y_ref = y_point;

          // Rotate back into previous coordinate system
          x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
          y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

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
      } else {
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