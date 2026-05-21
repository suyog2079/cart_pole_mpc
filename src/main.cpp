#include "network.hpp"
#include "nmpc.hpp"
#include <arpa/inet.h>
#include <atomic>
#include <cmath>
#include <iostream>
#include <netinet/in.h>
#include <pid.hpp>
#include <sstream>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace casadi;

const double M = 1.0;
const double m = 0.1;
const double l = 0.5;
const double g = 9.81;
const double Kv = 40.0;
const double cx = 2.5;
const double ctheta = 0.4;
const double Fmax = 30.0;

const double Ts = 0.01; // 100 Hz control
const int N = 20;       // horizon
const int nx = 4;
const int nu = 1;
const int ng = 0; // no path constraints

const double Q_x = 10.0;
const double Q_xdot = 1.0;
const double Q_theta = 100.0;
const double Q_thetadot = 1.0;
const double R_u = 0.1;

// Bounds
const double X_MIN = -2.5;
const double X_MAX = 2.5;
const double U_MIN = -5.0;
const double U_MAX = 5.0;

std::atomic<double> target_x{0.0};
std::atomic<double> target_theta_rad{0.0};
std::atomic<bool> new_target{false};

/**
 * @brief
 *
 * @param x state: [x, x_dot, theta, theta_dot]
 * @param u control velocity
 * @return state derivative: [x_dot, x_ddot, theta_dot, theta_ddot]
 */
MX dynamics_mx(const MX &x, const MX &u) {
  MX x_vel = x(1);
  MX theta = x(2);
  MX theta_d = x(3);

  MX F = Kv * (u - x_vel);
  F = if_else(F > Fmax, Fmax, if_else(F < -Fmax, -Fmax, F));

  MX s = sin(theta);
  MX c = cos(theta);
  MX denom = M + m * s * s;

  MX x_ddot =
      (F + m * s * (l * theta_d * theta_d + g * c) - cx * x_vel) / denom;
  MX theta_ddot = (-F * c - m * l * theta_d * theta_d * c * s -
                   (M + m) * g * s - ctheta * theta_d) /
                  (l * denom);

  return MX::vertcat({x_vel, x_ddot, theta_d, theta_ddot});
}

/**
 * @brief Stage cost: quadratic on state error and control, with angle error
 * handled
 *
 * @param x state: [x, x_dot, theta, theta_dot]
 * @param u control velocity
 * @param ref reference state: [x_ref, 0, theta_ref, 0]
 * @return
 */
MX stage_cost_mx(const MX &x, const MX &u, const MX &ref) {
  MX x_err = x - ref;
  MX theta_err =
      atan2(sin(x_err(2)), cos(x_err(2))); // angular error short path
  MX cost = Q_x * x_err(0) * x_err(0) + Q_xdot * x_err(1) * x_err(1) +
            Q_theta * theta_err * theta_err + Q_thetadot * x_err(3) * x_err(3) +
            R_u * u * u;
  return cost;
}

MX terminal_cost_mx(const MX &x, const MX &ref) {
  MX x_err = x - ref;
  MX theta_err = atan2(sin(x_err(2)), cos(x_err(2)));
  MX cost = Q_x * x_err(0) * x_err(0) + Q_xdot * x_err(1) * x_err(1) +
            Q_theta * theta_err * theta_err + Q_thetadot * x_err(3) * x_err(3);
  return cost;
}

MX path_constraints_mx(const MX &x, const MX &u) { return MX(0, 1); }

void input_thread_func() {
  std::string line;
  std::cout << "\n--- Control Interface ---\n";
  std::cout << "Enter: x_position(m) theta_angle(degrees)\n";
  std::cout << "Example: 0 180   (x=0, upright)\n";
  std::cout << "Example: 0.5 0    (x=0.5m, pole down)\n";
  std::cout << "Type 'quit' to exit.\n\n";

  while (true) {
    std::cout << "> ";
    std::getline(std::cin, line);
    if (line == "quit")
      exit(0);
    if (line.empty())
      continue;

    double x_val, theta_deg;
    std::istringstream iss(line);
    if (iss >> x_val >> theta_deg) {
      double theta_rad = theta_deg * M_PI / 180.0;
      target_x.store(x_val);
      target_theta_rad.store(theta_rad);
      new_target.store(true);
      std::cout << "Target set: x = " << x_val << " m, theta = " << theta_deg
                << "° (" << theta_rad << " rad)\n";
    } else {
      std::cout << "Invalid. Use: x theta_deg\n";
    }
  }
}

int main() {
  pthread_t tcp_thread;
  pthread_create(&tcp_thread, nullptr, tcp_client, nullptr);
  pthread_detach(tcp_thread);

  PID pid(10.0, 1.0, 0.5, Ts, 10.0, Fmax);

  // Initialize command to zero
  shared_send.store({0.0});

  // Wait for first state
  while (shared_updated.load() == 0)
    usleep(1000);

  std::thread input_thread(input_thread_func);
  input_thread.detach();

  double local_target_x = target_x.load();
  double local_target_theta = target_theta_rad.load();
  std::vector<double> ref(nx, 0.0); // [x_des, 0, theta_des, 0]
  ref[0] = local_target_x;
  ref[2] = local_target_theta;
  while (true) {
    if (shared_updated.load() != 1)
      continue;
    shared_updated.store(0);

    if (new_target.load()) {
      local_target_x = target_x.load();
      local_target_theta = target_theta_rad.load();
      new_target.store(false);
      std::cout << "MPC target updated: x = " << local_target_x
                << " m, theta = " << local_target_theta << " rad\n";
      ref[0] = local_target_x;
      ref[2] = local_target_theta;
    }

    recv_data state = shared_recv.load();
printf("target: %f, state: %f\n", local_target_theta, state.theta);
    send_data control = {pid.calculate(local_target_theta, state.theta)};

    shared_send.store(control);
  }

  MX x_sym = MX::sym("x", nx);
  MX u_sym = MX::sym("u", nu);
  MX x_dot = dynamics_mx(x_sym, u_sym);
  Function dyn_fun = Function("dyn", {x_sym, u_sym}, {x_dot});

  MX ref_sym = MX::sym("ref", nx);
  MX cost_stage = stage_cost_mx(x_sym, u_sym, ref_sym);
  Function stage_fun = Function("stage", {x_sym, u_sym, ref_sym}, {cost_stage});

  MX cost_term = terminal_cost_mx(x_sym, ref_sym);
  Function term_fun = Function("term", {x_sym, ref_sym}, {cost_term});

  Function path_fun = Function("path", {x_sym, u_sym}, {MX(0, 1)});

  NMPC nmpc(nx, nu, N, Ts, ng, dyn_fun, stage_fun, term_fun, path_fun);

  nmpc.set_state_bounds(0, X_MIN, X_MAX);
  nmpc.set_control_bounds(0, U_MIN, U_MAX);

  int total_vars = nx * (N + 1) + nu * N;
  std::vector<double> x_init(total_vars, 0.0);

  auto last_time = std::chrono::steady_clock::now();

  while (true) {
    if (new_target.load()) {
      local_target_x = target_x.load();
      local_target_theta = target_theta_rad.load();
      new_target.store(false);
      std::cout << "MPC target updated: x = " << local_target_x
                << " m, theta = " << local_target_theta << " rad\n";
      ref[0] = local_target_x;
      ref[2] = local_target_theta;
    }

    if (shared_updated.load() != 1) {
      printf("no new data recieved\n");
      continue;
    }
    shared_updated.store(0);

    recv_data state = shared_recv.load();

    std::vector<double> x0 = {state.x, state.x_dot, state.theta,
                              state.theta_dot};

    DM sol = nmpc.solve(x0, ref);
    int u_start = nx * (N + 1);
    double u0 = sol(u_start).scalar();

    printf("control input: %f\n", u0);
    printf("==================================================\n");
    printf("==================================================\n");
    printf("==================================================\n");
    printf("==================================================\n");
    printf("==================================================\n");
    printf("==================================================\n");
    printf("==================================================\n");

    shared_send.store({u0});
  }

  printf("Exiting main loop\n");

  return 0;
}
