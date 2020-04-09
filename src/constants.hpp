#pragma once

namespace constants {
// Distance in meters between points that will be interpolated using spline
constexpr float SPLINE_DIST = 30;
// Target velocity (mph)
constexpr double TARGET_VELOCITY = 49.9;
// Velocity step (mph)
constexpr double VELOCITY_STEP = 0.7;
// Refresh period in seconds
constexpr double REFRESH = .02;  // 50Hz
// Lane width in meters
constexpr float LANE_WIDTH = 4;
// Margin in meters with vehicle ahead before action in required
constexpr double FRONT_MARGIN = 30;
// Margin in meters with vehicle behind before action in required
constexpr double REAR_MARGIN = 5;
}  // namespace constants
