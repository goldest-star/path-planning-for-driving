#pragma once

namespace constants {
// Lane width in meters
constexpr float LANE_WIDTH = 4;
// Margin in meters with vehicle ahead before action in required
constexpr double FRONT_MARGIN = 30;
// Margin in meters with vehicle behind before action in required
constexpr double REAR_MARGIN = 10;
// Target velocity (mph)
constexpr double TARGET_VELOCITY = 49.9;
constexpr double CHANGE_VEL = 0.8;
// Velocity step (mph)
constexpr double VELOCITY_STEP = 0.5;
// Number of meters to plan ahead
constexpr float INTERP_HORIZON = 30;
// Distance in meters between points that will be interpolated using spline
constexpr float INTERP_INTERVAL = 30;
// Number of waypoints planned ahead
constexpr uint NB_WAYPOINTS = 50;
// Refresh period in seconds
constexpr double REFRESH = .02;  // 50Hz
}  // namespace constants
