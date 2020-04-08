#pragma once
#include <math.h>
#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// for convenience
using std::array;
using std::string;
using std::vector;

class Helpers {
 public:
  // Checks if the SocketIO event has JSON data.
  string hasData(string s);

  // Unit conversions
  constexpr double pi() { return M_PI; }
  double deg2rad(double x);
  double rad2deg(double x);
  double mpersec_to_mph(double x);

  // Calculate distance between two points
  double distance(double x1, double y1, double x2, double y2);

  // Calculate closest waypoint to current x, y position
  uint ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                       const vector<double> &maps_y);

  // Returns next waypoint of the closest waypoint
  uint NextWaypoint(double x, double y, double theta,
                    const vector<double> &maps_x, const vector<double> &maps_y);

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  vector<double> getFrenet(double x, double y, double theta,
                           const vector<double> &maps_x,
                           const vector<double> &maps_y);

  // Transform from Frenet s,d coordinates to Cartesian x,y
  vector<double> getXY(double s, double d, const vector<double> &maps_s,
                       const vector<double> &maps_x,
                       const vector<double> &maps_y);

  // Read data
  bool read_map_data(std::string filename,
                     array<vector<double>, 5> &map_waypoints);
};
