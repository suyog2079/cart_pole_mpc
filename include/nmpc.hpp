#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>

using namespace casadi;

class NMPC {
public:
  int nx, nu, N, ng;
  double dt;

  Function dynamics, stage_cost, terminal_cost, path_constraints;
  Function nlp_solver;

  std::vector<MX> constraints;
  std::vector<double> lbx, ubx, lbg, ubg;
	std::vector<double> x_prev;
	bool has_prev = false;

  NMPC(int _nx, int _nu, int _N, double _dt, int _ng, Function _dynamics,
       Function _stage_cost, Function _terminal_cost,
       Function _path_constraints)
      : nx(_nx), nu(_nu), N(_N), dt(_dt), ng(_ng), dynamics(_dynamics),
        stage_cost(_stage_cost), terminal_cost(_terminal_cost),
        path_constraints(_path_constraints) {
    build_nlp();
  }

  MX rk4(const MX &x, const MX &u) {
    MX k1 = dynamics(std::vector<MX>{x, u})[0];
    MX k2 = dynamics(std::vector<MX>{x + dt / 2.0 * k1, u})[0];
    MX k3 = dynamics(std::vector<MX>{x + dt / 2.0 * k2, u})[0];
    MX k4 = dynamics(std::vector<MX>{x + dt * k3, u})[0];
    return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4);
  }

  void build_nlp() {
    MX X = MX::sym("X", nx, N + 1);
    MX U = MX::sym("U", nu, N);
    MX P = MX::sym("P", nx * 2);

    MX objective = 0;
    std::vector<MX> g_vec;

    g_vec.push_back(X(Slice(), 0) - P(Slice(0, nx)));

    for (int k = 0; k < N; ++k) {
      MX xk = X(Slice(), k);
      MX uk = U(Slice(), k);
      MX ref = P(Slice(nx, 2 * nx));

      MX x_next = rk4(xk, uk);
      g_vec.push_back(X(Slice(), k + 1) - x_next);

      objective += stage_cost(std::vector<MX>{xk, uk, ref})[0];

      if (ng > 0) {
        MX cg = path_constraints(std::vector<MX>{xk, uk})[0];
        g_vec.push_back(cg);
      }
    }

    MX xN = X(Slice(), N);
    MX ref_final = P(Slice(nx, 2 * nx));
    objective += terminal_cost(std::vector<MX>{xN, ref_final})[0];

    MX OPT_vars = vertcat(reshape(X, nx * (N + 1), 1), reshape(U, nu * N, 1));
    MX G = vertcat(g_vec);

    MXDict nlp = {{"x", OPT_vars}, {"f", objective}, {"g", G}, {"p", P}};

    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;

    nlp_solver = nlpsol("solver", "ipopt", nlp, opts);

    constraints = g_vec;
    initializeBounds();
  }

  void initializeBounds() {
    int total_vars = nx * (N + 1) + nu * N;
    int total_constraints = nx + nx * N + ng * N;

    lbx.assign(total_vars, -inf);
    ubx.assign(total_vars, inf);
    lbg.assign(total_constraints, -inf);
    ubg.assign(total_constraints, inf);

    int n_eq = nx + nx * N;
    for (int i = 0; i < n_eq; ++i) {
      lbg[i] = ubg[i] = 0.0;
    }
  }

  void set_state_bounds(int state_index, double lower, double upper) {
    for (int k = 0; k <= N; ++k) {
      int idx = k * nx + state_index;
      if (idx < (int)lbx.size()) {
        lbx[idx] = lower;
        ubx[idx] = upper;
      }
    }
  }

  void set_control_bounds(int control_index, double lower, double upper) {
    int start = nx * (N + 1);
    for (int k = 0; k < N; ++k) {
      int idx = start + k * nu + control_index;
      if (idx < (int)lbx.size()) {
        lbx[idx] = lower;
        ubx[idx] = upper;
      }
    }
  }

  void set_constraint_bounds(int constraint_index, double lower, double upper) {
    int offset = nx + nx * N;
    for (int k = 0; k < N; ++k) {
      int idx = offset + k * ng + constraint_index;
      if (idx < (int)lbg.size()) {
        lbg[idx] = lower;
        ubg[idx] = upper;
      }
    }
  }

  DM solve(const std::vector<double> &x0, const std::vector<double> &ref) {
    std::vector<double> p = x0;
    p.insert(p.end(), ref.begin(), ref.end());

    int total_vars = nx * (N + 1) + nu * N;
    std::vector<double> x_init(total_vars, 0.0);

    if (has_prev && (int)x_prev.size() == total_vars) {
      // Shift state trajectory: drop x[0], repeat x[N] at end
      for (int k = 0; k < N; ++k)
        for (int i = 0; i < nx; ++i)
          x_init[k * nx + i] = x_prev[(k + 1) * nx + i];
      for (int i = 0; i < nx; ++i)
        x_init[N * nx + i] = x_prev[N * nx + i]; // hold last state
      // Shift control: drop u[0], repeat u[N-1] at end
      int u_off = nx * (N + 1);
      for (int k = 0; k < N - 1; ++k)
        x_init[u_off + k] = x_prev[u_off + k + 1];
      x_init[u_off + N - 1] = x_prev[u_off + N - 1]; // hold last control
    }

    std::map<std::string, DM> args;
    args["x0"] = x_init;
    args["lbx"] = lbx;
    args["ubx"] = ubx;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    args["p"] = p;

    auto sol = nlp_solver(args);
    DM sol_x = sol.at("x");

    x_prev = sol_x.nonzeros();
    has_prev = true;
    return sol_x;
  }

private:
  MX objective;
};
