// RB-13 stage 2 scalar equilibrium checks using the effective IPC primitive.
#include <ipc/barrier/barrier.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>

double force(double d, double h)
{
	return -2 * d * ipc::barrier_first_derivative(d * d, h * h);
}

double curvature(double d, double h)
{
	return 2 * ipc::barrier_first_derivative(d * d, h * h)
		   + 4 * d * d * ipc::barrier_second_derivative(d * d, h * h);
}

int main()
{
	double K, p, k, h;
	std::cout << std::setprecision(17);
	while (std::cin >> K >> p >> k >> h)
	{
		if (!std::isfinite(K) || !std::isfinite(p) || !std::isfinite(k)
			|| !std::isfinite(h) || K <= 0 || k < 0 || h <= 0)
		{
			std::cout << "invalid_input\n";
			continue;
		}
		if (k == 0 && p <= 0)
		{
			std::cout << "no_positive_equilibrium\n";
			continue;
		}
		double lo = 0, hi = h, d = p;
		if (k > 0 && p < h)
		{
			for (int i = 0; i < 120; ++i)
			{
				const double mid = (lo + hi) / 2;
				if (mid == lo || mid == hi)
					break;
				if (K * (mid - p) - k * force(mid, h) < 0)
					lo = mid;
				else
					hi = mid;
			}
			d = (lo + hi) / 2;
		}
		else
		{
			lo = hi = d;
		}
		const double F = k == 0 || d >= h ? 0 : k * force(d, h);
		const double C = d >= h ? 0 : curvature(d, h);
		const double residual = (K * (d - p) - F) / (K * h + std::abs(F));
		if (!std::isfinite(d) || !std::isfinite(F) || !std::isfinite(C)
			|| !std::isfinite(residual))
		{
			std::cout << "nonfinite_evaluation\n";
			continue;
		}
		std::cout << "ok " << d << ' ' << residual << ' ' << hi - lo
				  << ' ' << F << ' ' << C << '\n';
	}
	return std::cin.eof() ? 0 : 1;
}
