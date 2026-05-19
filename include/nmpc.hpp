#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>

using namespace casadi;

class NMPC {
public:
  int nx;
  int nu;
  int N;  // prediction horizon
  int ng; // number of constraints
  double dt;

  MX X;
  MX U;
  MX P; // initial params
  MX objective;

  Function dynamics;
  Function stage_cost;
  Function terminal_cost;
  Function path_constraints;

  Function nlp_solver;

  std::vector<MX> constraints;
  std::vector<double> lbx;
  std::vector<double> ubx;

  std::vector<double> lbg;
  std::vector<double> ubg;

  NMPC(int _nx, int _nu, int _N, double _dt, int _ng, Function _dynamics,
       Function _stage_cost, Function _terminal_cost,
       Function _path_contstraints)
      : nx(_nx), nu(_nu), N(_N), dt(_dt), ng(_ng), dynamics(_dynamics),
        stage_cost(_stage_cost), terminal_cost(_terminal_cost),
        path_constraints(_path_contstraints) {
    build_nlp();
  }

  MX rk4(const MX &x, const MX &u) {
    MX k1 = dynamics({x, u})[0];

    MX k2 = dynamics({x + dt / 2.0 * k1, u})[0];

    MX k3 = dynamics({x + dt / 2.0 * k2, u})[0];

    MX k4 = dynamics({x + dt * k3, u})[0];

    return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4);
  }

  void build_nlp() {
    X = MX::sym("X", nx, N + 1);
    U = MX::sym("U", nu, N);
    P = MX::sym("P", nx * 2);

    objective = 0;

    g.push_back(X(Slice(), 0) - P(Slice(0, nx)));

    for (int k = 0; k < N; k++) {
      MX xk = X(Slice(), k);
      MX uk = U(Slice(), k);
      MX ref = P(Slice(nx, 2 * nx));

      MX x_next = rk4(xk, uk);

      g.push_back(X(Slice(), k + 1) - x_next);

      objective += stage_cost({xk, uk, ref})[0];

      if (ng > 0) {
        MX cg = path_constraints({xk, uk})[0];
        g.push_back(cg);
      }
    }

    objective += terminalCost({X(Slice(), N), P(Slice(nx, 2 * nx))})[0];

    // using slack variables
    MX OPT_variables =
        vertcat(reshape(X, nx * (N + 1), 1), reshape(U, nu * N, 1));

    MX G = vertcat(g);

    Dict nlp = {{"x", OPT_variables}, {"f", objective}, {"g", G}, {"p", P}};

    Dict opts;

    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;

    solver = nlpsol("solver", "ipopt", nlp, opts);

    initialize_bounds();
  }

  void initializeBounds() {
    int total_vars = nx * (N + 1) + nu * N;

    lbx.resize(total_vars, -inf);
    ubx.resize(total_vars, inf);

    int total_constraints = nx + nx * N + ng * N;

    lbg.resize(total_constraints, 0.0);
    ubg.resize(total_constraints, 0.0);
  }

  void setStateBounds(int state_index, double lower, double upper) {
    for (int k = 0; k < N + 1; k++) {
      int idx = k * nx + state_index;

      lbx[idx] = lower;
      ubx[idx] = upper;
    }
  }

  void setControlBounds(int control_index, double lower, double upper) {
    int start = nx * (N + 1);

    for (int k = 0; k < N; k++) {
      int idx = start + k * nu + control_index;

      lbx[idx] = lower;
      ubx[idx] = upper;
    }
  }

  void setConstraintBounds(int constraint_index, double lower, double upper) {
    int offset = nx + nx * N;

    for (int k = 0; k < N; k++) {
      int idx = offset + k * ng + constraint_index;

      lbg[idx] = lower;
      ubg[idx] = upper;
    }
  }

  DM solve(const std::vector<double> &x0, const std::vector<double> &ref) {
    std::vector<double> p;

    p.insert(p.end(), x0.begin(), x0.end());

    p.insert(p.end(), ref.begin(), ref.end());

    int total_vars = nx * (N + 1) + nu * N;

    std::vector<double> x_init(total_vars, 0.0);

    std::map<std::string, DM> args;

    args["x0"] = x_init;
    args["lbx"] = lbx;
    args["ubx"] = ubx;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    args["p"] = p;

    auto sol = solver(args);

    return sol.at("x");
  }
};
