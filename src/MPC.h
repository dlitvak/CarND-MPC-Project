#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using std::vector;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  static constexpr double LATENCY_SECONDS = 0.1;
  static constexpr double Lf = 2.67;

  struct Solution {
      vector<double> xv;
      vector<double> yv;
      double psi;
      double vel;
      double cte;
      double epsi;
      double delta;
      double acc;
  };

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  Solution Solve(const Eigen::VectorXd &state,
                            const Eigen::VectorXd &coeffs);

  /**
   * Convert yaw angle to slope
   *
   * @param psi - yaw angle (-2pi, 2pi)
   * @return - slope angle (-pi/2, pi/2)
   */
  static double normalize_Psi_To_Slope_Angle(double psi) {
    // M_PI_2 is pi/2
    while (psi > M_PI_2) psi -= M_PI;
    while (psi < -M_PI_2) psi += M_PI;

    return psi;
  }
};

#endif  // MPC_H
