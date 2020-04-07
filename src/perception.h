#pragma once

#include <limits>
#include "helpers.h"

using std::vector;

class PerceptionModule {
 public:
  /**
   * Constructor.
   */
  PerceptionModule(const float &lwidth);

  /**
   * Destructor.
   */
  virtual ~PerceptionModule();

  void update(const vector<vector<double> > &sensor_fusion,
              const double &delta_t, const double &car_s);

  // Motion planning
  array<double, 3> front_margins, rear_margins, front_speeds, rear_speeds;
  float lane_width;

 private:
  Helpers helpers;
  void reset_env();
};
