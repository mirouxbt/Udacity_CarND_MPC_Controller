#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
//./mpc 120 14 3 200 0.05 1 0.1 280000 100
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


// Velocity reference for cost
double ref_v    = 50.0 * 0.447038889;  // MPH to m/s-1

// Number of steps to define the Horizon
size_t N = 14;

// Factors for cost computation
double factor_cte        = 80.0;
double factor_epsi       = 200.0;
double factor_v          = 20.0;
double factor_delta      = 1.0;
double factor_throttle   = 0.1;
double factor_seq_delta  = 280000.0;
double factor_seq_throttle = 100.0;  






// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main(int argc, char **argv) {
  uWS::Hub h;

  // Get parameters if any
  if (argc == 10) {
    ref_v               = atof(argv[1]) * 0.447038889;  // MPH to m/s-1
    N                   = atoi(argv[2]);
    factor_cte          = atof(argv[3]);
    factor_epsi         = atof(argv[4]);
    factor_v            = atof(argv[5]);
    factor_delta        = atof(argv[6]);
    factor_throttle     = atof(argv[7]);
    factor_seq_delta    = atof(argv[8]);
    factor_seq_throttle = atof(argv[9]);
  }

  // MPC is initialized here!
  MPC mpc;

  
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx  = j[1]["ptsx"];
          vector<double> ptsy  = j[1]["ptsy"];
          double         px    = j[1]["x"];
          double         py    = j[1]["y"];
          double         psi   = j[1]["psi"];
          double         v     = j[1]["speed"];
          double         delta = j[1]["steering_angle"];
          double         a     = j[1]["throttle"];


          // Converting measurements
          v     *= 0.447038889; // Conversion to m/s
          delta *= 0.436332313; // conversion to rad - 0.436332313 = deg2rad(25)

          // Convert trajectory in car coordinate
          double cospsi = cos(psi);
          double sinpsi = sin(psi);
          for (int i=0; i < ptsx.size(); i++) {
              double x0 = ptsx[i] - px;
              double y0 = ptsy[i] - py;
              ptsx[i] =  x0*cospsi + y0*sinpsi;
              ptsy[i] = -x0*sinpsi + y0*cospsi;
          }

          
          // 1 - Fit the reference trajectory - Need to convert to Eigen vector
          Eigen::VectorXd ptsx_v(ptsx.size());
          Eigen::VectorXd ptsy_v(ptsy.size());
          for (int i=0; i<ptsx.size(); i++) {
            ptsx_v[i] = ptsx[i];
            ptsy_v[i] = ptsy[i];
          }
          
          auto coeffs = polyfit(ptsx_v, ptsy_v, 2);
          

          // 2 - Compute the cte, f(x) - y - In car coordinates x=0 y=0
          //     f(x) = a + bx + cx2 + dx3 => f(0) = a
          double cte =  coeffs[0];
          //   - Get the psi error, psi - atan(f'(x)) - In car coord psi=0 x=0
          //     f'(x) = b + 2cx + 3dx2 => f'(0) = b
          double epsi = - atan(coeffs[1]);
          
          // 3 - To integrate the latency, our current state is altency ahead
          //     When effectively we will use the actuators
          //     So we need to estimate our state in the future
          //     dt integrate actuator latency + computation latency
          double px_c   = v * dt;
          double py_c   = 0.0;
          double psi_c  = v * -delta * dt / Lf;
          double v_c    = v + a * dt;
          double cte_c  = cte + v * sin(epsi) * dt;
          double epsi_c = epsi + v * -delta * dt / Lf;

          
          // 4 - Use MPC to get new actuators
          Eigen::VectorXd state(6);
          state << px_c, py_c, psi_c, v_c, cte_c, epsi_c;

          mpc.Solve(state, coeffs);
          
          json msgJson;

          msgJson["steering_angle"] = mpc.steer;
          msgJson["throttle"]       = mpc.throttle;

          //Display the MPC predicted trajectory 
          msgJson["mpc_x"] = mpc.traj_x_vals;
          msgJson["mpc_y"] = mpc.traj_y_vals;

          //Display the waypoints/reference line
          msgJson["next_x"] = ptsx;
          msgJson["next_y"] = ptsy;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << " SV:" << mpc.steer << " TH:" << mpc.throttle << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
