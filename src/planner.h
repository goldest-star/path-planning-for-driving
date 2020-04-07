#pragma once

#include <array>
#include "perception.h"

using std::array;
using std::vector;

class Planner {
 public:
  /**
   * Constructor.
   */
  Planner(const float &ref_spline, const double &max_front,
          const double &max_rear, const float &lwidth);

  /**
   * Destructor.
   */
  virtual ~Planner();

  void sense(const vector<vector<double> > &sensor_fusion,
             const double &delta_t, const double &car_s);

  void update(uint &lane, double &target_vel, float &spline_dist_);

  // Motion planning
  float spline_dist;
  double front_margin, rear_margin;
  array<bool, 3> lane_avails, lane_transitions;

 private:
  void reset_env();
  PerceptionModule module;
};
