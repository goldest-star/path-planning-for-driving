#pragma once

#include <math.h>
#include <array>
#include <vector>
#include "helpers.h"
#include "spline.h"

using std::array;
using std::cos;
using std::sin;
using std::vector;

class Controller {
 public:
  /**
   * Constructor.
   */
  Controller(const double &vel_delta, const float &lwidth,
             const float &refresh_period,
             const std::array<vector<double>, 5> &map_waypoints);

  /**
   * Destructor.
   */
  virtual ~Controller();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  double update_velocity(const double &target_vel);

  // Update state information about vehicle
  void update_readings(const double &car_x, const double &car_y,
                       const double &car_yaw, const double &car_vel,
                       const double &car_s, const vector<double> car_prev_xs,
                       const vector<double> car_prev_ys);

  std::array<vector<double>, 2> get_trajectory(const uint &lane,
                                               const float &spline_dist);

  // Values to send to the web server
  vector<double> prev_xs;
  vector<double> prev_ys;
  // double refresh, vel_step;
  double x, y, yaw, velocity, s, refresh, ref_x, ref_y, ref_yaw, vel_step;
  float lane_width;
  vector<double> ptsx, ptsy;
  std::array<vector<double>, 5> waypoints;

 private:
  Helpers helpers;

  void frame_trajectory(const uint &lane, const float &spline_dist);
  std::array<vector<double>, 2> extrapolate_trajectory();
};
