#pragma once

#include <functional>
#include <string>
#include <thread>

#include <Eigen/Core>

#if defined(POLYFEM_WITH_TBB) || defined(POLYFEM_WITH_CPP_THREADS)
#include <tbb/global_control.h>
#endif

namespace polyfem
{
	namespace utils
	{
		class NThread
		{
		public:
			static NThread &get()
			{
				static NThread instance;
				return instance;
			}

			inline size_t num_threads() const { return num_threads_; }

			void set_num_threads(const int max_threads)
			{
				const unsigned int tmp = max_threads <= 0 ? std::numeric_limits<int>::max() : max_threads;
				const unsigned int num_threads = std::min(tmp, std::thread::hardware_concurrency());

				num_threads_ = num_threads;
#if defined(POLYFEM_WITH_TBB) || defined(POLYFEM_WITH_CPP_THREADS)
				thread_limiter = std::make_shared<tbb::global_control>(tbb::global_control::max_allowed_parallelism, num_threads);
#endif
				Eigen::setNbThreads(num_threads);
				limit_accelerate_threads(max_threads > 0 ? num_threads : 0);
			}

			/// The VECLIB_MAXIMUM_THREADS value in force after the last
			/// set_num_threads, empty when unset (Apple builds only).
			inline const std::string &accelerate_threads() const { return accelerate_threads_; }
			/// Where accelerate_threads() came from: "max_threads", "environment"
			/// (preset by the user, kept), "unlimited" or "not_applicable".
			inline const std::string &accelerate_source() const { return accelerate_source_; }

		private:
			NThread() {}

			/// Caps Apple Accelerate's own threads (0 = no cap), see par_for.cpp.
			void limit_accelerate_threads(const unsigned limit);

			size_t num_threads_;
			std::string accelerate_threads_;
			std::string accelerate_source_ = "not_applicable";
			/// The value this class exported, empty when it exported none.
			std::string accelerate_exported_;

#if defined(POLYFEM_WITH_TBB) || defined(POLYFEM_WITH_CPP_THREADS)
			/// limits the number of used threads
			std::shared_ptr<tbb::global_control> thread_limiter;
#endif
		};

		void par_for(const int size, const std::function<void(int, int, int)> &func);
		inline size_t get_n_threads() { return NThread::get().num_threads(); }
	} // namespace utils
} // namespace polyfem
