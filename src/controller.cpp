#include "controller.h"

/**
 * Constructor.
 */
Controller::Controller(const double &vel_delta, const float &lwidth,
                       const float &refresh_period,
                       const float &t_interp_horizon,
                       const float &t_interp_interval,
                       const uint &t_nb_waypoints,
                       const array<vector<double>, 5> &map_waypoints) {
  vel_step = vel_delta;
  lane_width = lwidth;
  refresh = refresh_period;
  interp_horizon = t_interp_horizon;
  interp_interval = t_interp_interval;
  nb_waypoints = t_nb_waypoints;
  waypoints = map_waypoints;
}

/**
 * Destructor.
 */
Controller::~Controller() = default;

void Controller::update_readings(const double &car_x, const double &car_y,
                                 const double &car_yaw, const double &car_vel,
                                 const double &car_s,
                                 const vector<double> car_prev_xs,
                                 const vector<double> car_prev_ys) {
  x = car_x;
  y = car_y;
  yaw = car_yaw;
  velocity = car_vel;
  s = car_s;
  prev_xs = car_prev_xs;
  prev_ys = car_prev_ys;
}

double Controller::update_velocity(const double &target_vel) {
  // Speed regulator
  if (velocity > target_vel) {
    velocity -= vel_step;
  } else if (velocity + vel_step < target_vel) {
    velocity += vel_step;
  }

  return velocity;
}

std::array<vector<double>, 2> Controller::get_trajectory(const uint &lane) {
  Controller::frame_trajectory(lane);
  return Controller::extrapolate_trajectory();
}

std::array<vector<double>, 2> Controller::extrapolate_trajectory() {
  // Spline interpolation
  tk::spline s;

  s.set_points(ptsx, ptsy);

  vector<double> next_xs;
  vector<double> next_ys;

  // Start with previous path points from last time
  for (uint i = 0; i < prev_xs.size(); ++i) {
    next_xs.push_back(prev_xs[i]);
    next_ys.push_back(prev_ys[i]);
  }

  // Spline points
  double target_y = s(interp_horizon);
  double target_dist = sqrt(pow(interp_horizon, 2) + pow(target_y, 2));
  double x_add_on = 0;
  // Miles per hours --> meters / sec
  double N = (target_dist / (refresh * velocity / 2.24));

  // Fill up the rest of our planner, force 50 points
  for (uint i = 1; i <= nb_waypoints - prev_xs.size(); ++i) {
    double x_point = x_add_on + interp_horizon / N;
    double y_point = s(x_point);
    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Rotate back
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    // Translation
    x_point += ref_x;
    y_point += ref_y;

    next_xs.push_back(x_point);
    next_ys.push_back(y_point);
  }

  array<vector<double>, 2> next_coords = {next_xs, next_ys};

  return next_coords;
}

void Controller::frame_trajectory(const uint &lane) {
  // Widely space points
  ptsx.clear();
  ptsy.clear();
  // Ref states
  ref_x = x;
  ref_y = y;
  ref_yaw = helpers.deg2rad(yaw);

  // if previous path is almost empty, use the car as starting ref
  uint prev_size = prev_xs.size();
  if (prev_size < 2) {
    // Use two points to get path tangent to the car
    double prev_x = x - cos(yaw);
    double prev_y = y - sin(yaw);
    ptsx.push_back(prev_x);
    ptsx.push_back(x);
    ptsy.push_back(prev_y);
    ptsy.push_back(y);
  } else {
    // Use previous path as starting reference
    ref_x = prev_xs[prev_size - 1];
    ref_y = prev_ys[prev_size - 1];

    double ref_x_prev = prev_xs[prev_size - 2];
    double ref_y_prev = prev_ys[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // Path tangent to previous path end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet coords, waypoints are spaced by spline_dist meters ahead of
  // starting ref
  for (uint i = 1; i <= 3; ++i) {
    vector<double> next_wp =
        helpers.getXY(s + i * interp_interval, lane_width * (lane + 0.5),
                      waypoints[2], waypoints[0], waypoints[1]);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);
  }

  // Shift car angle
  for (uint i = 0; i < ptsx.size(); ++i) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    // Make car angle tangent to next position direction
    ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
    ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
  }
}
