#include <uWS/uWS.h>
#include <string>
#include <vector>
#include "constants.hpp"
#include "controller.h"
#include "helpers.h"
#include "json.hpp"
#include "planner.h"
#include "spdlog/spdlog.h"

// for convenience
using nlohmann::json;
using std::exception;
using std::string;
using std::vector;
using namespace constants;

Helpers helpers;

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

  if (!helpers.read_map_data("../data/highway_map.csv", map_waypoints)) {
    spdlog::error("Unable to access highway map file!");
    throw PPException();
  }

  // Lanes are numbered (0 | 1 | 2)
  // Start on lane 1 (middle lane)
  int lane = 1;

  // Inicial velocity in mph, and also reference velocity to target.
  double velocity = 0.0;

  // Instantiate the motion planner and controller
  Planner behavior_planner(FRONT_MARGIN, REAR_MARGIN, LANE_WIDTH);
  Controller controller(VELOCITY_STEP, LANE_WIDTH, REFRESH, SPLINE_DIST,
                        map_waypoints);

  h.onMessage([&lane, &velocity, &behavior_planner, &controller](
                  uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = helpers.hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_yaw = j[1]["yaw"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          // Avoids collision
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          // Perception
          auto delta_t = static_cast<double>(prev_size) * REFRESH;
          behavior_planner.sense(sensor_fusion, delta_t, car_s);

          // Motion planning
          auto target_vel_ = TARGET_VELOCITY;
          behavior_planner.update(lane, target_vel_);

          // Let controller update its information
          controller.update_readings(car_x, car_y, car_yaw, velocity, car_s,
                                     previous_path_x, previous_path_y);
          // Set the new target speed
          velocity = controller.update_velocity(target_vel_);
          // Compute the trajectory
          std::array<vector<double>, 2> next_coords =
              controller.get_trajectory(lane);

          json msgJson;

          msgJson["next_x"] = next_coords[0];
          msgJson["next_y"] = next_coords[1];

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
      [&h, &velocity](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        spdlog::info("Environment session connected!");
        // Ensure that new driving sessions starts with zero velocity
        velocity = 0.0;
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
