#include "../src/helpers.h"
#include "catch.hpp"

SCENARIO("Degrees to radians conversion") {
  GIVEN("1 angle in degrees") {
    Helpers helpers;
    double deg;

    WHEN("angle is null") {
      deg = 0;
      double rad = helpers.deg2rad(deg);
      THEN("no error is thrown") { REQUIRE(rad == 0); }
    }

    WHEN("360°") {
      deg = 360;
      double rad = helpers.deg2rad(deg);
      THEN("no error is thrown") { REQUIRE(rad == (2 * M_PI)); }
    }
  }
}

SCENARIO("Radians to degrees conversion") {
  GIVEN("1 angle in radians") {
    Helpers helpers;
    double rad;

    WHEN("angle is null") {
      rad = 0;
      double deg = helpers.rad2deg(rad);
      THEN("no error is thrown") { REQUIRE(deg == 0); }
    }

    WHEN("2 PI") {
      rad = 2 * M_PI;
      double deg = helpers.rad2deg(rad);
      THEN("no error is thrown") { REQUIRE(deg == 360); }
    }
  }
}

SCENARIO("L2 distance computation") {
  GIVEN("4 scalar values") {
    Helpers helpers;
    double x1, y1, x2, y2;

    WHEN("both 2D points are identical") {
      x1 = 1;
      y1 = 1;
      double d_ = helpers.distance(x1, y1, x1, y1);
      THEN("no error is thrown") { REQUIRE(d_ == 0); }
    }

    WHEN("both 2D points are different") {
      x1 = 1;
      y1 = 1;
      x2 = 2;
      y2 = 2;
      double d_ = helpers.distance(x1, y1, x2, y2);
      THEN("no error is thrown") { REQUIRE(d_ == sqrt(2)); }
    }
  }
}

SCENARIO("Speed conversion") {
  GIVEN("1 speed in m/s") {
    Helpers helpers;
    double speed;

    WHEN("10 m/s") {
      speed = 10;
      double speed_ = helpers.mpersec_to_mph(speed);
      THEN("no error is thrown") {
        REQUIRE(fabs(speed_ - 22.3694185194) < 1e-9);
      }
    }
  }
}
