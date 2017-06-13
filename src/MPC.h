#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


// Delta t to our prediction steps
// Use the actuator latency 100ms + 0.04 to account for our computation time
const double dt = 0.14;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

const double ref_cte  = 0.0;
const double ref_epsi = 0.0;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Update the actuators values throttle, steer
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  double throttle;
  double steer;
  vector<double> traj_x_vals;
  vector<double> traj_y_vals;

};

#endif /* MPC_H */
