#include "planner.h"

// Constructor
Planner::Planner(const double &max_front, const double &max_rear,
                 const float &lwidth, const float &t_change_vel)
    : module(lwidth) {
  front_margin = max_front;
  rear_margin = max_rear;
  change_vel = t_change_vel;
  Planner::reset_env();
}

// Destructor
Planner::~Planner() = default;

void Planner::reset_env() {
  is_avail = {true, true, true};
  is_allowed = {true, true, true};
}

void Planner::sense(const vector<vector<double> > &sensor_fusion,
                    const double &delta_t, const double &car_s) {
  Planner::reset_env();
  module.update(sensor_fusion, delta_t, car_s);
  for (uint i = 0; i < is_avail.size(); ++i) {
    is_avail[i] = (module.front_margins[i] > front_margin) &&
                  (module.rear_margins[i] > rear_margin);
    // Allow transition through lanes with lower front margin
    is_allowed[i] = (module.front_margins[i] > (front_margin / 2)) &&
                    (module.rear_margins[i] > rear_margin);
  }
}

void Planner::update(int &target_lane, double &target_vel,
                     const double &car_d) {
  // Assess current behaviour
  int current_lane = static_cast<int>(car_d / module.lane_width);
  // When should we consider that manoeuver is complete
  bool is_changing_lane = (target_lane != current_lane);
  // bool is_changing_lane = fabs(module.lane_width *
  // (static_cast<float>(target_lane) + 0.5) - car_d) < (module.lane_width / 4);
  // Let the car finish its manoeuver
  if (is_changing_lane) {
    // Match the vehicle ahead's speed while changing lane
    target_vel =
        min(module.front_speeds[current_lane], change_vel * target_vel);
    // Lane availability has changed and no manoeuver is engaged
  } else if (!is_avail[current_lane]) {
    // Check whether nearby lanes are available
    // Other lane available with engageable lanes in between
    int best_lane = current_lane;
    double max_margin = front_margin;
    for (int i = 0; i < is_avail.size(); ++i) {
      if (i != current_lane) {
        // Good candidate
        if (is_avail[i] && (module.front_margins[i] > max_margin)) {
          // Deal with edge case of 2-lanes difference
          if (std::abs(current_lane - i) == 2) {
            // Get inbetween lane
            uint lane = current_lane + 1;
            if (i < current_lane) {
              lane = current_lane - 1;
            }
            // Check if that lane is engageable
            if (!is_allowed[lane]) {
              continue;
            } else {
              // Take intermediate lane for now (the planner will select the
              // next best lane afterwards)
              best_lane = lane;
            }
          } else {
            best_lane = i;
          }
          max_margin = module.front_margins[i];
        }
      }
    }

    // Engage lane change
    if (current_lane != best_lane) {
      target_lane = best_lane;
      // Slow down for lane change to avoid jerk issues
      target_vel =
          min(module.front_speeds[current_lane], change_vel * target_vel);
    } else {
      // Match the velocity of vehicle ahead
      target_vel = module.front_speeds[current_lane];
    }
  }
}
