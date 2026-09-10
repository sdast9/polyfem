// Standalone RB-13 probe: compile the effective IPC barrier.cpp, not a copy.
#include <ipc/barrier/barrier.hpp>

#include <iomanip>
#include <iostream>

int main()
{
	double s, support;
	std::cout << std::setprecision(17);
	while (std::cin >> s >> support)
	{
		std::cout << ipc::barrier(s, support) << ' '
				  << ipc::barrier_first_derivative(s, support) << ' '
				  << ipc::barrier_second_derivative(s, support) << '\n';
	}
	return std::cin.eof() ? 0 : 1;
}
