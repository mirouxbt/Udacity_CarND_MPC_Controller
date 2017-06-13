#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;



// Velocity reference for cost
extern double ref_v;

// Number of steps to define the Horizon
extern size_t N;

// Factors for cost computation
extern double factor_cte;
extern double factor_epsi;
extern double factor_v;
extern double factor_delta;
extern double factor_throttle;
extern double factor_seq_delta;
extern double factor_seq_throttle; 



// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start;
size_t y_start;
size_t psi_start;
size_t v_start;
size_t cte_start;
size_t epsi_start;
size_t delta_start;
size_t a_start;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  
  // `fg`   is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    fg[0] = 0.0;

    // Cost to reference
    for (int i = 0; i < N; i++) {
      fg[0] += factor_cte  * CppAD::pow(vars[cte_start  + i] - ref_cte , 2);
      fg[0] += factor_epsi * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      fg[0] += factor_v    * CppAD::pow(vars[v_start    + i] - ref_v   , 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += factor_delta    * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += factor_throttle * CppAD::pow(vars[a_start     + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
      fg[0] += factor_seq_delta    * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += factor_seq_throttle * CppAD::pow(vars[a_start     + i + 1] - vars[a_start     + i], 2);
    }

    //
    // Setup Constraints
    //

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start]    = vars[x_start];
    fg[1 + y_start]    = vars[y_start];
    fg[1 + psi_start]  = vars[psi_start];
    fg[1 + v_start]    = vars[v_start];
    fg[1 + cte_start]  = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    
    // The remaining constraints
    for (int i = 0; i < N - 1; i++) {
      AD<double> x1    = vars[x_start    + i + 1];
      AD<double> y1    = vars[y_start    + i + 1];
      AD<double> psi1  = vars[psi_start  + i + 1];
      AD<double> v1    = vars[v_start    + i + 1];
      AD<double> cte1  = vars[cte_start  + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      AD<double> x0    = vars[x_start     + i];
      AD<double> y0    = vars[y_start     + i];
      AD<double> psi0  = vars[psi_start   + i];
      AD<double> v0    = vars[v_start     + i];
      AD<double> cte0  = vars[cte_start   + i];
      AD<double> epsi0 = vars[epsi_start  + i];


      AD<double> d0    = vars[delta_start + i];
      AD<double> a0    = vars[a_start     + i];
      
      // Y reference, used for CTE
      AD<double> f0 = 0.0;
      for (int i = 0; i < coeffs.size(); i++) {
        f0 += coeffs[i] * CppAD::pow(x0, i);
      }
      
      // Here we use the derivative of the polynom
      // to compute the psi on the trajectory to follow
      AD<double> psid0 = 0.0;
      for (int i = 1; i < coeffs.size(); i++) {
        psid0 += coeffs[i] * CppAD::pow(x0, i-1) * i;
      }
      
      // Here we follow the equations of our kinetic model
      fg[2 + x_start    + i] = x1    - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start    + i] = y1    - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start  + i] = psi1  - (psi0 + (v0 * -d0 * dt) / Lf);
      fg[2 + v_start    + i] = v1    - (v0 + a0 * dt);
      fg[2 + cte_start  + i] = cte1  - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[2 + epsi_start + i] = epsi1 - (psi0 - psid0 + (v0 * -d0 * dt) / Lf);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() : traj_x_vals(N), traj_y_vals(N) {
  
}
MPC::~MPC() {}

void MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  // We have 6 states (x, y, psi, v, cte, epsi) for N steps
  // We have 2 actuators (delta, a) for N - 1 steps
  size_t n_vars = N * 6 + (N-1) * 2;
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Define the starting point of our vars
  x_start      = 0;
  y_start      = x_start     + N;
  psi_start    = y_start     + N;
  v_start      = psi_start   + N;
  cte_start    = v_start     + N;
  epsi_start   = cte_start   + N;
  delta_start  = epsi_start  + N;
  a_start      = delta_start + N - 1; // Here -1 as there is N-1 actuators




  // Get better name for initial state
  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];


  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  // Set  the inital state
  vars[x_start]    = x;
  vars[y_start]    = y;
  vars[psi_start]  = psi;
  vars[v_start]    = v;
  vars[cte_start]  = cte;
  vars[epsi_start] = epsi;


  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e9;
    vars_upperbound[i] =  1.0e9;
  }

  // Set delta to -/+ 25 deg in rad
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] =  0.436332;
  }

  // Set acceleration to -/+ 1
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] =  1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Init for initial state
  constraints_lowerbound[x_start]    = x;
  constraints_lowerbound[y_start]    = y;
  constraints_lowerbound[psi_start]  = psi;
  constraints_lowerbound[v_start]    = v;
  constraints_lowerbound[cte_start]  = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start]    = x;
  constraints_upperbound[y_start]    = y;
  constraints_upperbound[psi_start]  = psi;
  constraints_upperbound[v_start]    = v;
  constraints_upperbound[cte_start]  = cte;
  constraints_upperbound[epsi_start] = epsi;


  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Update the actuators and potential trajectory
  for (int i=0; i < N; i++) {
    traj_x_vals[i] = solution.x[i];
    traj_y_vals[i] = solution.x[y_start + i];
  }
  
  steer    = solution.x[delta_start] / 0.436332313; // 0.436332313 = deg2rad(25)
  throttle = solution.x[a_start];
  
  return;
}
