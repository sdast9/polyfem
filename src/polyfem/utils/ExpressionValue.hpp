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
			void set_index(const int index) { index_ = index; }

			/// @brief RB-11: bind a per-element value list/file to the mesh.
			///        A list with one entry per element of the whole mesh
			///        (n_global rows) is indexed by the global element id; one
			///        with one entry per element of the body it is given for
			///        (n_body rows) by the body-local index; any other length does
			///        not describe this mesh and is a named error. Constants and
			///        expressions are untouched.
			/// @param local_index  the element's index within its body (-1 when
			///                     the material is not given per body)
			/// @param n_body       elements of the body this value belongs to
			/// @param n_global     elements of the whole FE mesh
			/// @param what         parameter name for the error message
			void bind_per_element(const int local_index, const Eigen::Index n_body, const Eigen::Index n_global, const std::string &what);
			/// @brief True when the value is a per-element list/file indexed by
			///        the global element id (after bind_per_element).
			bool is_per_element_global() const { return mat_size() > 1 && index_ < 0 && t_index_.empty(); }

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
				// Cached storage is immutable. Mutations replace only this value.
				mat_ = std::make_shared<const Eigen::MatrixXd>(mat);
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
			// Shared within an explicitly owned input snapshot; set_mat detaches.
			std::shared_ptr<const Eigen::MatrixXd> mat_;
			std::vector<ExpressionValue> mat_expr_;
			std::map<double, int> t_index_;
			int index_ = -1;

			Eigen::Index mat_size() const { return mat_ ? mat_->size() : 0; }

			units::precise_unit unit_type_;
			units::precise_unit unit_;
			bool unit_type_set_ = false;
		};
	} // namespace utils
} // namespace polyfem
