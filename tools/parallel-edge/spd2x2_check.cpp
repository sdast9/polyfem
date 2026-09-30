// Residual and forward error of ipc::detail::solve_spd_2x2 on the nearly parallel edge-edge system
// that trips the Debug assertion in closest_point.hpp (A, b from the rollback AL-budget scene).
// Build with the include/define flags of `ninja -t commands ...closest_point.cpp.o` plus -msse4.2.
// Unpatched toolkit f8dafef3: solve_spd_2x2 resid 4.3e-9, LDLT 1.1e-16; with cloud/parallel-edge-fix: 1.1e-16.
#include <ipc/tangent/closest_point.hpp>
#include <cstdio>
int main()
{
	Eigen::Matrix2d A;
	A << 0.067896765590525515, 1.1037422437870266, 1.1037422437870266, 17.942643306292943;
	Eigen::Vector2d b(0.62082203665333358, 10.092200801559782);
	Eigen::Vector2d x = ipc::detail::solve_spd_2x2<double>(A, b);
	Eigen::Vector2d xr = A.ldlt().solve(b);
	Eigen::Matrix<long double, 2, 2> AL = A.cast<long double>();
	Eigen::Matrix<long double, 2, 1> bL = b.cast<long double>();
	Eigen::Matrix<long double, 2, 1> xe = AL.inverse() * bL;
	printf("solve_spd_2x2 resid=%.3e  ldlt resid=%.3e  |x-x_ld|=%.3e  |ldlt-x_ld|=%.3e\n", (A * x - b).norm(), (A * xr - b).norm(), (double)((x.cast<long double>() - xe).norm()), (double)((xr.cast<long double>() - xe).norm()));
}
