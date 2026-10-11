#include <cstdio>
#include "controller/nova_ik.hpp"
int main(int argc, char** argv) {
  double x, y, z;
  while (std::scanf("%lf %lf %lf", &x, &y, &z) == 3) {
    auto r = nova::solveIK(x, y, z);
    if (r.ok) std::printf("OK %.6f %.6f %.6f %.6f %.6f %.4f\n", r.q[0], r.q[1], r.q[2], r.q[3], r.q[4], r.elevation);
    else std::printf("FAIL %s\n", r.reason.c_str());
  }
}
