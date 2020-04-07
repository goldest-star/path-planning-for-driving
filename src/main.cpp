#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spdlog/spdlog.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::exception;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  // x, y, s, dx, dy
  std::array<vector<double>, 5> map_waypoints;

  // Map data safeguard
  struct PPException : public exception {
    const char *what() const throw() {
      return "Unable to access highway map file!";
    }
  };

  if (!read_map_data("../data/highway_map.csv", map_waypoints)) {
    spdlog::error("Unable to access highway map file!");
    throw PPException();
  }
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // Lanes are numbered (0 | 1 | 2)
  // Start on lane 1 (middle lane)
  int lane = 1;

  // Inicial velocity, and also reference velocity to target.
  double current_vel = 0.0;               // mph
  const double target_vel = 49.7;         // mph
  const double vel_delta = 3 * .224;      // 5m/s
  const double controller_refresh = .02;  // second
  const float lane_width = 4;             // m
  const double security_dist = 30;        // m

  // True when the ego-car is changing lane.
  bool is_changing_lane = false;
  double end_change_lane_s = 0.0;

  h.onMessage([&map_waypoints, &lane, &current_vel, &vel_delta, &target_vel,
               &controller_refresh, &lane_width,
               &security_dist](uWS::WebSocket<uWS::SERVER> ws, char *data,
                               size_t length, uWS::OpCode opCode) {
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

          int prev_size = previous_path_x.size();

          // Avoids collision
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool slow_down = false;
          vector<bool> is_lane_free = {true, true, true};
          vector<double> ahead_speed = {target_vel, target_vel, target_vel};
          vector<double> ahead_dist = {std::numeric_limits<double>::infinity(),
                                       std::numeric_limits<double>::infinity(),
                                       std::numeric_limits<double>::infinity()};

          // Loop on obstacles (vehicles) detected with sensor fusion
          for (uint i = 0; i < sensor_fusion.size(); i++) {
            // Check if lane is free to move to
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_car_s = sensor_fusion[i][5];
            float d = sensor_fusion[i][6];
            double check_speed = sqrt(pow(vx, 2) + pow(vy, 2));
            check_car_s += (static_cast<double>(prev_size) *
                            controller_refresh * check_speed);
            int lane_ = static_cast<int>(d / lane_width);
            // Check if there is a car ahead of us in this lane
            if ((check_car_s >= car_s - 5) &&
                (check_car_s < car_s + security_dist)) {
              is_lane_free[lane_] = false;
              // Shadow velocity of ahead vehicle (no need to slow down further)
              if ((check_car_s > car_s) && (check_car_s < ahead_speed[lane_])) {
                ahead_speed[lane_] = check_speed;
                ahead_dist[lane_] = check_car_s;
              }
            }
          }

          // Plan lane change
          double s_speed;
          if (!is_lane_free[lane]) {
            // See if there is any free lane
            if ((lane > 0) && (is_lane_free[lane - 1])) {
              lane -= 1;
            } else if ((lane < 2) && (is_lane_free[lane + 1])) {
              lane += 1;
            } else {
              // Can't change lane --> slow down for now
              slow_down = true;
              s_speed = ahead_speed[lane];
            }
          }

          if (slow_down) {
            current_vel -= vel_delta;
          } else if (current_vel < target_vel) {
            current_vel += vel_delta;
          }

          // Widely space points
          vector<double> ptsx, ptsy;
          // Ref states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous pathis almost empty, use the car as starting ref
          if (prev_size < 2) {
            // Use two points to get path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // Use previous path as starting reference
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // Path tangent to previous path end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet coords, add 30m spaced waypoints ahead of starting ref
          vector<double> next_wp0 =
              getXY(car_s + 30, lane_width * (lane + 0.5), map_waypoints[2],
                    map_waypoints[0], map_waypoints[1]);
          vector<double> next_wp1 =
              getXY(car_s + 60, lane_width * (lane + 0.5), map_waypoints[2],
                    map_waypoints[0], map_waypoints[1]);
          vector<double> next_wp2 =
              getXY(car_s + 90, lane_width * (lane + 0.5), map_waypoints[2],
                    map_waypoints[0], map_waypoints[1]);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Shift car angle
          for (uint i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          // Spline interpolation
          tk::spline s;

          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with previous path points from last time
          for (uint i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Spline points
          double target_x = 10.0;
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
          double x_add_on = 0;

          // Fill up the rest of our planner, force 50 points
          for (uint i = 1; i <= 50 - previous_path_x.size(); i++) {
            // Miles per hours --> meters / sec
            double N =
                (target_dist / (controller_refresh * current_vel / 2.24));
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);
            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Rotate back
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

  h.onConnection(
      [&h, &current_vel](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        spdlog::info("Environment session connected!");
        // Ensure that new driving sessions starts with zero velocity
        current_vel = 0.0;
      });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    spdlog::info("Disconnected from session");
  });

  int port = 4567;
  if (h.listen(port)) {
    spdlog::info("Listening to port {}", port);
  } else {
    spdlog::error("Failed to listen to port {}", port);
    return -1;
  }

  h.run();
}
