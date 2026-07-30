#pragma once

#include <polyfem/Common.hpp>
#include <map>
#include <memory>

#include <units/units.hpp>

namespace polyfem
{
	namespace utils
	{
		class ExpressionValue
		{
		public:
			ExpressionValue();

			void set_unit_type(const std::string &unit_type)
			{
				unit_type_ = units::unit_from_string(unit_type);
				unit_type_set_ = true;
				for (auto &expr : mat_expr_)
					expr.set_unit_type(unit_type);
			}

			void init(const json &vals, const std::string &root_path);
			void init(const double val);
			void init(const Eigen::MatrixXd &val);
			void init(const std::string &expr, const std::string &root_path);
#ifdef POLYFEM_WITH_PYTHON
			void init_python(const std::string &path, const std::string &function_name);
#endif

			void init(const std::function<double(double x, double y, double z)> &func);
			void init(const std::function<double(double x, double y, double z, double t)> &func);
			void init(const std::function<double(double x, double y, double z, double t, int index)> &func);

			void init(const std::function<Eigen::MatrixXd(double x, double y, double z)> &func, const int coo);
			void init(const std::function<Eigen::MatrixXd(double x, double y, double z, double t)> &func, const int coo);

			void set_t(const json &t);

			double operator()(double x, double y, double z = 0, double t = 0, int index = -1) const;

			void clear();

			bool is_zero() const
			{
				return expr_.empty() && mat_size() == 0 && mat_expr_.empty() && !sfunc_ && !tfunc_ && fabs(value_) < 1e-10;
			}
			bool is_mat() const
			{
				if (expr_.empty() && mat_size() > 0)
					return true;
				return false;
			}

			const Eigen::MatrixXd &get_mat() const
			{
				assert(is_mat());
				return *mat_;
			}

			void set_mat(const Eigen::MatrixXd &mat)
			{
				assert(is_mat());
				assert(mat_->rows() == mat.rows());
				assert(mat_->cols() == mat.cols());
				// mat_ may be shared with the process-wide file cache (see
				// init(string)); never write through a shared buffer.
				if (mat_.use_count() > 1)
					mat_ = std::make_shared<Eigen::MatrixXd>(*mat_);
				*mat_ = mat;
			}

			double get_val() const
			{
				return value_;
			}

		private:
			std::function<double(double x, double y, double z, double t, int index)> sfunc_;
			std::function<Eigen::MatrixXd(double x, double y, double z, double t)> tfunc_;
			int tfunc_coo_;

			std::string expr_;
			double value_;
			// Shared so that a value loaded from a file (init(string) below) is
			// read and stored once per path per process: these are initialized
			// once per mesh element, so an owning copy would be O(n^2) in both
			// time and memory. Mutation goes through set_mat's copy-on-write.
			std::shared_ptr<Eigen::MatrixXd> mat_;
			std::vector<ExpressionValue> mat_expr_;
			std::map<double, int> t_index_;

			Eigen::Index mat_size() const { return mat_ ? mat_->size() : 0; }

			units::precise_unit unit_type_;
			units::precise_unit unit_;
			bool unit_type_set_ = false;
		};
	} // namespace utils
} // namespace polyfem
