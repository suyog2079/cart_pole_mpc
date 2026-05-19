#include <iostream>
#include <casadi/casadi.hpp>

using namespace casadi;

class NMPC {
public:
	int nx;
	int nu;
	int N; // prediction horizon
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


