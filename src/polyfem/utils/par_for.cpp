#include "par_for.hpp"

#include <vector>
#include <algorithm>
#include <cstdlib>

namespace polyfem
{
	namespace utils
	{
		// Apple's Accelerate (the default Eigen::AccelerateLDLT solver and the
		// vecLib BLAS) runs its own threads, which neither TBB's global_control
		// nor Eigen limits. vecLib reads VECLIB_MAXIMUM_THREADS when it starts
		// work, so exporting it here, before the first factorization, caps it.
		// Its multithreaded sparse factorization is not bitwise deterministic:
		// only a one-thread cap makes runs reproducible
		// (docs/it-reproducibility-20260928.md). A value the user preset in the
		// environment is kept.
		void NThread::limit_accelerate_threads(const unsigned limit)
		{
#ifdef __APPLE__
			static constexpr const char *variable = "VECLIB_MAXIMUM_THREADS";
			const char *current = std::getenv(variable);
			if (current != nullptr && (accelerate_exported_.empty() || accelerate_exported_ != current))
			{
				accelerate_exported_.clear();
				accelerate_threads_ = current;
				accelerate_source_ = "environment";
				return;
			}
			if (limit == 0)
			{
				if (current != nullptr)
					unsetenv(variable);
				accelerate_exported_.clear();
				accelerate_threads_.clear();
				accelerate_source_ = "unlimited";
				return;
			}
			accelerate_exported_ = std::to_string(limit);
			setenv(variable, accelerate_exported_.c_str(), /*overwrite=*/1);
			accelerate_threads_ = accelerate_exported_;
			accelerate_source_ = "max_threads";
#endif
		}

		void par_for(const int size, const std::function<void(int, int, int)> &func)
		{
#ifdef POLYFEM_WITH_CPP_THREADS
			const size_t n_threads = get_n_threads();
			if (n_threads == 1)
				func(0, size, /*thread_id=*/0); // actually the full for loop
			else
			{
				std::vector<std::thread> threads(n_threads);

				for (int t = 0; t < n_threads; t++)
				{
					threads[t] = std::thread(std::bind(
						func,
						t * size / n_threads,
						(t + 1) == n_threads ? size : (t + 1) * size / n_threads,
						t));
				}
				std::for_each(threads.begin(), threads.end(), [](std::thread &x) { x.join(); });
			}
#endif
		}
	} // namespace utils
} // namespace polyfem
