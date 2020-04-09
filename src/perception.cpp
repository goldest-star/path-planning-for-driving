#include "perception.h"

// Constructor
PerceptionModule::PerceptionModule(const float &lwidth) {
  lane_width = lwidth;
  PerceptionModule::reset_env();
}

// Destructor
PerceptionModule::~PerceptionModule() = default;

void PerceptionModule::reset_env() {
  front_margins = {std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::infinity()};
  rear_margins = {std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity()};
}

void PerceptionModule::update(const vector<vector<double> > &sensor_fusion,
                              const double &delta_t, const double &car_s) {
  PerceptionModule::reset_env();
  // Loop on obstacles (vehicles) detected with sensor fusion
  for (uint i = 0; i < sensor_fusion.size(); ++i) {
    // Check vehicle position and motion
    double car_s_ = sensor_fusion[i][5];
    float d_ = sensor_fusion[i][6];
    double speed_ = sqrt(pow(static_cast<double>(sensor_fusion[i][3]), 2) +
                         pow(static_cast<double>(sensor_fusion[i][4]), 2));
    // Expected s after next controller update
    car_s_ += delta_t * speed_;
    int lane_ = static_cast<int>(d_ / lane_width);
    // Vehicle is in front
    if (car_s_ > car_s) {
      if ((car_s_ - car_s) < front_margins[lane_]) {
        front_margins[lane_] = car_s_ - car_s;
        front_speeds[lane_] = helpers.mpersec_to_mph(speed_);
      }
      // Vehicle behind
    } else {
      if ((car_s - car_s_) < rear_margins[lane_]) {
        rear_margins[lane_] = car_s - car_s_;
        rear_speeds[lane_] = helpers.mpersec_to_mph(speed_);
      }
    }
  }
}
