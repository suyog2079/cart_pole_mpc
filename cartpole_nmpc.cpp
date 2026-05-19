#include <arpa/inet.h>
#include <atomic>
#include <casadi/casadi.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace casadi;

// ---------- Physical constants ----------
const double M = 1.0; //mass of cart
const double m = 0.1; // mass of pole
const double l = 0.5; // length to pole center of mass
const double g = 9.81; // acceleration due to gravity
const double Kv = 40.0; // motor velocity constant
const double cx = 2.5; // cart friction
const double ctheta = 0.4; // pole friction
const double Fmax = 30.0; // max force from motor

// MPC parameters
const double Ts = 0.02; // 50 Hz control
const int N = 20;

// Cost weights
const double Q_x = 50.0;
const double Q_xdot = 1.0;
const double Q_theta = 100.0;
const double Q_thetadot = 1.0;
const double R_u = 0.1;

// Input limits
const double u_max = 5.0;

// Shared target (atomic)
std::atomic<double> target_x{0.0};
std::atomic<double> target_theta_rad{0.0};
std::atomic<bool> new_target{false};

// ---------- Dynamics ----------
SX f_c(const SX &x, const SX &u) {
  SX x_vel = x(1);
  SX theta = x(2);
  SX theta_d = x(3);

  SX F = Kv * (u - x_vel);
  F = if_else(F > Fmax, Fmax, if_else(F < -Fmax, -Fmax, F));

  SX s = sin(theta);
  SX c = cos(theta);
  SX denom = M + m * s * s;

  SX x_ddot =
      (F + m * s * (l * theta_d * theta_d + g * c) - cx * x_vel) / denom;
  SX theta_ddot = (-F * c - m * l * theta_d * theta_d * c * s -
                   (M + m) * g * s - ctheta * theta_d) /
                  (l * denom);

  return SX::vertcat({x_vel, x_ddot, theta_d, theta_ddot});
}

SX rk4_step(const SX &x, const SX &u, double dt) {
  SX k1 = f_c(x, u);
  SX k2 = f_c(x + dt / 2 * k1, u);
  SX k3 = f_c(x + dt / 2 * k2, u);
  SX k4 = f_c(x + dt * k3, u);
  return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
}

// ---------- TCP communication with timeout ----------
int connect_simulator(const std::string &ip, int port) {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    perror("socket");
    exit(1);
  }
  sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
  if (connect(sock, (sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("connect");
    close(sock);
    exit(1);
  }
  return sock;
}

void send_command(int sock, double u) {
  std::string cmd = std::to_string(u) + "\n";
  send(sock, cmd.c_str(), cmd.size(), 0);
}

// Non‑blocking receive with timeout (0.5 seconds)
bool receive_state(int sock, double state[4]) {
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(sock, &fds);
  struct timeval tv = {1, 0}; // 0.5 sec timeout
  if (select(sock + 1, &fds, nullptr, nullptr, &tv) <= 0) {
    return false; // timeout or error
  }
  static std::string buffer;
  char chunk[256];
  int n = recv(sock, chunk, sizeof(chunk) - 1, 0);
  if (n <= 0)
    return false;
  chunk[n] = '\0';
  buffer += chunk;
  size_t pos = buffer.find('\n');
  if (pos == std::string::npos)
    return false;
  std::string line = buffer.substr(0, pos);
  buffer.erase(0, pos + 1);
  // Format: "x=...,theta=...,xdot=...,thetadot=..."
  // Python sends: "x,theta,xdot,thetadot"
  return sscanf(line.c_str(), "%lf,%lf,%lf,%lf",
                &state[0], // x
                &state[2], // theta
                &state[1], // x_dot
                &state[3]) // theta_dot
         == 4;
}

// ---------- Input thread ----------
void input_thread_func() {
  std::string line;
  std::cout << "\n--- Control Interface ---\n";
  std::cout << "Enter: x_position(m) theta_angle(degrees)\n";
  std::cout << "Example: 1 180\n";
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

// ---------- Main ----------
int main() {
    int sock = connect_simulator("127.0.0.1", 8080);
    std::cout << "Connected to cart‑pole simulator.\n";

    // ---- Handshake: send dummy command and wait for first state ----
    send_command(sock, 0.0);
    double state[4];
    bool got_state = false;
    for (int attempt = 0; attempt < 10; ++attempt) {
        if (receive_state(sock, state)) {
            got_state = true;
            std::cout << "Initial state received: x=" << state[0] << " θ=" << state[2] << "\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (!got_state) {
        std::cerr << "Could not get initial state from simulator. Exiting.\n";
        close(sock);
        return 1;
    }

    // Start input thread
    std::thread input_thread(input_thread_func);
    input_thread.detach();

    // Build NLP
    SX x0 = SX::sym("x0", 4);
    SX u = SX::sym("u", N);
    SX xd = x0;
    SX x_des_sym = SX::sym("x_des");
    SX theta_des_sym = SX::sym("theta_des");
    SX cost = 0;

    for (int i = 0; i < N; ++i) {
        xd = rk4_step(xd, u(i), Ts);
        SX theta_err = atan2(sin(xd(2) - theta_des_sym), cos(xd(2) - theta_des_sym));
        cost += Q_x     * (xd(0) - x_des_sym)*(xd(0) - x_des_sym)
              + Q_xdot  * xd(1)*xd(1)
              + Q_theta * theta_err*theta_err
              + Q_thetadot * xd(3)*xd(3)
              + R_u     * u(i)*u(i);
    }
    SX theta_err_T = atan2(sin(xd(2) - theta_des_sym), cos(xd(2) - theta_des_sym));
    cost += Q_x     * (xd(0) - x_des_sym)*(xd(0) - x_des_sym)
          + Q_xdot  * xd(1)*xd(1)
          + Q_theta * theta_err_T*theta_err_T
          + Q_thetadot * xd(3)*xd(3);

    SX params = SX::vertcat({x0, x_des_sym, theta_des_sym});
    SXDict nlp = {{"x", u}, {"p", params}, {"f", cost}};

    Dict opts;
    opts["print_time"] = false;
    opts["ipopt.print_level"] = 0;      // set to 5 for debugging
    opts["ipopt.max_iter"] = 100;
    opts["ipopt.tol"] = 1e-4;
    Function solver = nlpsol("solver", "ipopt", nlp, opts);

    std::vector<double> lbu(N, -u_max), ubu(N, u_max);
    std::vector<double> u_opt(N, 0.0);

    auto last_time = std::chrono::steady_clock::now();
    double local_target_x = target_x.load();
    double local_target_theta = target_theta_rad.load();

    // ---- Main control loop (no goto) ----
    while (true) {
        // Try to receive state
        if (!receive_state(sock, state)) {
            std::cerr << "Warning: receive timeout. Re-sending previous command.\n";
            send_command(sock, u_opt[0]);   // last known command
            // Maintain rate and continue (skip solving)
            auto now = std::chrono::steady_clock::now();
            auto elapsed = now - last_time;
            auto sleep_dur = std::chrono::duration<double>(Ts) - elapsed;
            if (sleep_dur.count() > 0) std::this_thread::sleep_for(sleep_dur);
            last_time = now;
            continue;
        }

        // Check for new target from user
        if (new_target.load()) {
            local_target_x = target_x.load();
            local_target_theta = target_theta_rad.load();
            new_target.store(false);
            std::cout << "MPC target updated: x = " << local_target_x
                      << " m, theta = " << local_target_theta << " rad\n";
        }

        // Solve NMPC
        std::vector<double> p_vec = {
            state[0], state[1], state[2], state[3],
            local_target_x, local_target_theta
        };
        DMDict arg = {
            {"x0", DM(u_opt)},
            {"lbx", DM(lbu)},
            {"ubx", DM(ubu)},
            {"p", DM(p_vec)}
        };
        DMDict res = solver(arg);
        bool solver_ok = true;
        if (res.find("success") != res.end()) {
            solver_ok = (res.at("success").scalar() > 0.5);
        }
        if (!solver_ok) {
            std::cerr << "Solver failed. Sending zero command.\n";
            send_command(sock, 0.0);
        } else {
            DM u_sol_dm = res.at("x");
            std::vector<double> u_sol = u_sol_dm.nonzeros();
            if (u_sol.size() != N) {
                std::cerr << "Solver returned wrong size, using zeros.\n";
                u_sol.assign(N, 0.0);
            }
            double u_first = u_sol[0];
            send_command(sock, u_first);
            // std::cout << "State: x=" << state[0] << " θ=" << state[2]
            //           << "  →  u=" << u_first << std::endl;
            // Shift warm start
            for (int i = 0; i < N-1; ++i) u_opt[i] = u_sol[i+1];
            u_opt[N-1] = u_sol[N-1];
        }

        // Maintain 50 Hz loop rate
        auto now = std::chrono::steady_clock::now();
        auto elapsed = now - last_time;
        auto sleep_dur = std::chrono::duration<double>(Ts) - elapsed;
        if (sleep_dur.count() > 0) std::this_thread::sleep_for(sleep_dur);
        last_time = now;
    }

    close(sock);
    return 0;
}
