// Scalar ordinary-barrier access for the isolated RB-13 mechanical fixtures.
#include <ipc/barrier/barrier.hpp>

extern "C" double rb13_value(double d, double h)
{
	return ipc::barrier(d * d, h * h);
}

extern "C" double rb13_force(double d, double h)
{
	return -2 * d * ipc::barrier_first_derivative(d * d, h * h);
}

extern "C" double rb13_curvature(double d, double h)
{
	return 2 * ipc::barrier_first_derivative(d * d, h * h)
		   + 4 * d * d * ipc::barrier_second_derivative(d * d, h * h);
}
