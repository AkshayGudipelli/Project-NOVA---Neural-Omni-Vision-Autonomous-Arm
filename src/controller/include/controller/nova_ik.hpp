// Pure C++ (no ROS) analytical IK for the NOVA 5-DOF arm: pan + 3 pitch joints + roll.
// Convention: joints 2,3,4 all rotate about +Y, so the cumulative angle from vertical is
//   phi1 = q2, phi2 = q2+q3, phi3 = q2+q3+q4   and   tool elevation = pi/2 - phi3.
// Elevation -90 deg = gripper pointing straight down.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace nova {

struct IkParams {
  double d1 = 0.07;          // base_link -> joint2 height   (URDF: 0.02 + 0.05)
  double L1 = 0.105;         // joint2 -> joint3
  double L2 = 0.095;         // joint3 -> joint4
  double L3 = 0.095;         // joint4 -> grasp point (0.04 to roll axis + 0.055 to finger-pad middle)
  double base_yaw = 1.5707963267948966;  // arm "forward" direction in base_link (+Y in your scene)
  double min_clear_z = 0.02; // elbow/wrist must stay this far above the table (base_link z=0)
  std::array<double, 5> lo{{-1.5708, -1.5708, -1.5708, -1.5708, -3.1415}};
  std::array<double, 5> hi{{ 1.5708,  1.5708,  1.5708,  1.5708,  3.1415}};
};

struct IkResult {
  bool ok = false;
  std::array<double, 5> q{{0, 0, 0, 0, 0}};
  double elevation = 0.0;   // tool elevation actually used [rad]
  std::string reason;
};

inline double wrapPi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// Target = desired grasp point in base_link [m]. Tries steep (top-down) approaches first.
inline IkResult solveIK(double x, double y, double z, const IkParams& p = IkParams(),
                        double min_elev_deg = -90.0, double max_elev_deg = -20.0) {
  IkResult res;
  const double q1 = wrapPi(std::atan2(y, x) - p.base_yaw);
  if (q1 < p.lo[0] || q1 > p.hi[0]) { res.reason = "base pan out of servo range"; return res; }
  const double r = std::hypot(x, y);
  const double zr = z - p.d1;

  for (double e = min_elev_deg; e <= max_elev_deg + 1e-9; e += 5.0) {
    const double th = e * M_PI / 180.0;
    const double rw = r - p.L3 * std::cos(th);
    const double zw = zr - p.L3 * std::sin(th);
    const double D2 = rw * rw + zw * zw, D = std::sqrt(D2);
    if (D < 1e-6) continue;
    const double cb = (p.L1 * p.L1 + p.L2 * p.L2 - D2) / (2.0 * p.L1 * p.L2);
    if (cb < -1.0 || cb > 1.0) continue;                      // wrist centre unreachable
    const double q3 = M_PI - std::acos(cb);                   // elbow-up
    const double ca = std::clamp((p.L1 * p.L1 + D2 - p.L2 * p.L2) / (2.0 * p.L1 * D), -1.0, 1.0);
    const double q2 = M_PI / 2.0 - (std::atan2(zw, rw) + std::acos(ca));
    const double q4 = (M_PI / 2.0 - th) - (q2 + q3);
    const std::array<double, 5> q{{q1, q2, q3, q4, 0.0}};
    bool within = true;
    for (int i = 0; i < 4; ++i) if (q[i] < p.lo[i] || q[i] > p.hi[i]) within = false;
    if (!within) continue;
    const double elbow_z = p.d1 + p.L1 * std::cos(q2);
    const double wrist_z = elbow_z + p.L2 * std::cos(q2 + q3);
    if (elbow_z < p.min_clear_z || wrist_z < p.min_clear_z) continue;
    res.ok = true; res.q = q; res.elevation = th; return res;
  }
  res.reason = "no joint-limit-feasible solution (too close, too far, or too low)";
  return res;
}

}  // namespace nova
