#pragma once

#include <array>
#include "perception.h"

using std::array;
using std::min;
using std::vector;

class Planner {
 public:
  /**
   * Constructor.
   */
  Planner(const double &max_front, const double &max_rear, const float &lwidth,
          const float &t_change_vel);

  /**
   * Destructor.
   */
  virtual ~Planner();

  void sense(const vector<vector<double> > &sensor_fusion,
             const double &delta_t, const double &car_s);

  void update(int &target_lane, double &target_vel, const double &car_d);

  // Motion planning
  double front_margin, rear_margin;
  float change_vel;
  array<bool, 3> is_avail, is_allowed;

 private:
  void reset_env();
  PerceptionModule module;
};
