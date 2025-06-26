// include/state_sharing_demo/transforms.hpp
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace state_sharing_demo {

/**
 * @brief  Equirectangular (flat-earth) LLA→NED transform (header-only).
 */
inline Eigen::Vector3d llhToNed(
    double lat, double lon, double alt,
    double ref_lat, double ref_lon, double ref_alt)
{
  constexpr double deg2rad = M_PI / 180.0;
  constexpr double R_e     = 6378137.0;  // Earth radius [m]

  double dlat  = (lat - ref_lat) * deg2rad;
  double dlon  = (lon - ref_lon) * deg2rad;
  double north = dlat * R_e;
  double east  = dlon * R_e * std::cos(ref_lat * deg2rad);
  double down  = (ref_alt - alt);

  return {north, east, down};
}

}  // namespace state_sharing_demo
