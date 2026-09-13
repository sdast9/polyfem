#pragma once

#include <polyfem/Common.hpp>
#include <polyfem/utils/Types.hpp>
#include <polyfem/utils/ExpressionValue.hpp>
#include <polyfem/utils/MaterialFileCache.hpp>

#include <memory>
#include <string>
#include <vector>

namespace polyfem::assembler
{
	inline constexpr const char *MATERIAL_ELEMENT_INDEX = "__polyfem_material_element_index";
	/// @brief RB-11: `[elements of the body, elements of the whole mesh]`,
	///        attached by Assembler::set_materials so per-element value
	///        lists/files can be bound (global rows or body-local rows) and
	///        their length validated at load time.
	inline constexpr const char *MATERIAL_ELEMENT_COUNTS = "__polyfem_material_element_counts";

	/// @brief Copy the per-element binding keys from one material json to another
	///        (composite materials hand them to their children).
	void copy_material_element_binding(const json &from, json &to);
	/// @brief Bind a material value to the mesh from the binding keys in params
	///        (see ExpressionValue::bind_per_element); a no-op without them.
	void bind_material_value(utils::ExpressionValue &value, const json &params, const std::string &what);

	/// @brief RB-11: whether any material entry (or composite child) reads a
	///        per-element value file (a string naming an existing file) or a
	///        per-element fibre file. Remeshing re-binds materials on local
	///        patches by patch-local element ids, so such inputs cannot be
	///        transferred there; callers refuse the combination with a named
	///        error instead of misindexing.
	/// @param[in]  materials  the `materials` json (object or array)
	/// @param[in]  root_path  path root for resolving file names
	/// @param[out] where      the first offending parameter, for the message
	bool materials_use_per_element_files(const json &materials, const std::string &root_path, std::string &where);

	class GenericMatParam
	{
	public:
		GenericMatParam(const std::string &param_name);

		double operator()(const RowVectorNd &p, double t, int index) const;
		double operator()(double x, double y, double z, double t, int index) const;

		void add_multimaterial(const int index, const json &params, const std::string &unit_type, const std::string &root_path);

	private:
		const std::string param_name_;
		std::vector<utils::ExpressionValue> param_;

		friend class GenericMatParams;
	};

	class GenericMatParams
	{
	public:
		GenericMatParams(const std::string &param_name);

		const GenericMatParam &operator[](const size_t i) const { return params_[i]; }
		size_t size() const { return params_.size(); }

		void add_multimaterial(const int index, const json &params, const std::string &unit_type, const std::string &root_path);

	private:
		const std::string param_name_;
		std::vector<GenericMatParam> params_;
	};

	class ElasticityTensor
	{
	public:
		void resize(const int size);

		double operator()(int i, int j) const;
		double &operator()(int i, int j);

		void set_from_entries(const std::vector<double> &entries, const std::string &stress_unit, const std::string &root_path);
		void set_from_lambda_mu(const double lambda, const double mu, const std::string &stress_unit, const std::string &root_path);
		void set_from_young_poisson(const double young, const double poisson, const std::string &stress_unit, const std::string &root_path);

		void set_orthotropic(
			double Ex, double Ey, double Ez,
			double nuXY, double nuXZ, double nuYZ,
			double muYZ, double muZX, double muXY, const std::string &stress_unit, const std::string &root_path);
		void set_orthotropic(double Ex, double Ey, double nuXY, double muXY, const std::string &stress_unit, const std::string &root_path);
		void set_transversely_isotropic(
			double Et, double Ea,
			double nu_t, double nu_a,
			double Ga, const std::string &stress_units, const std::string &root_path);

		template <int DIM>
		double compute_stress(const std::array<double, DIM> &strain, const int j) const;

		void rotate_stiffness(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 6> &rotation_mtx_voigt);
		void unrotate_stiffness();

	private:
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 6> stiffness_tensor_;
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 6> reference_stiffness_tensor_;
		int size_;
	};

	class LameParameters
	{
	public:
		LameParameters();

		void add_multimaterial(const int index, const json &params, const bool is_volume, const std::string &stress_unit, const std::string &root_path);

		void lambda_mu(double px, double py, double pz, double x, double y, double z, double t, int el_id, double &lambda, double &mu) const;
		void lambda_mu(const Eigen::MatrixXd &param, const Eigen::MatrixXd &p, double t, int el_id, double &lambda, double &mu) const
		{
			assert(param.size() == 2 || param.size() == 3);
			assert(param.size() == p.size());
			lambda_mu(
				param(0), param(1), param.size() == 3 ? param(2) : 0.0,
				p(0), p(1), p.size() == 3 ? p(2) : 0.0,
				t,
				el_id, lambda, mu);
		}

		Eigen::MatrixXd lambda_mat_, mu_mat_;

	private:
		void set_e_nu(const int index, const json &E, const json &nu, const std::string &stress_unit, const std::string &root_path);

		int size_;
		std::vector<utils::ExpressionValue> lambda_or_E_, mu_or_nu_;
		bool is_lambda_mu_;
	};

	class Density
	{
	public:
		Density();
		virtual ~Density() = default;

		virtual void add_multimaterial(const int index, const json &params, const std::string &density_unit, const std::string &root_path);

		virtual double operator()(double px, double py, double pz, double x, double y, double z, double t, int el_id) const;
		virtual double operator()(const Eigen::MatrixXd &param, const Eigen::MatrixXd &p, double t, int el_id) const
		{
			assert(param.size() == 2 || param.size() == 3);
			assert(param.size() == p.size());
			return (*this)(param(0), param(1), param.size() == 3 ? param(2) : 0.0,
						   p(0), p(1), p.size() == 3 ? p(2) : 0.0,
						   t, el_id);
		}

	private:
		void set_rho(const json &rho);

		std::vector<utils::ExpressionValue> rho_;
	};

	class NoDensity : public Density
	{
	public:
		using Density::operator();

		NoDensity() {}

		void add_multimaterial(const int index, const json &params, const std::string &density_unit, const std::string &root_path) override
		{
			throw std::runtime_error("NoDensity does not support multimaterial");
		}

		double operator()(double px, double py, double pz, double x, double y, double z, double t, int el_id) const override
		{
			return 1.0;
		}
	};

	class ThermalMassDensity : public Density
	{
	public:
		using Density::operator();

		ThermalMassDensity();

		void add_multimaterial(const int index, const json &params, const std::string &density_unit, const std::string &root_path) override;
		void add_multimaterial(const int index, const json &params, const std::string &density_unit, const std::string &heat_capacity_unit, const std::string &root_path);

		double operator()(double px, double py, double pz, double x, double y, double z, double t, int el_id) const override;
		double rho(const RowVectorNd &p, double t, int el_id) const;
		double heat_capacity(const RowVectorNd &p, double t, int el_id) const;

	private:
		GenericMatParam rho_;
		GenericMatParam heat_capacity_;
	};

	class FiberDirection
	{
	public:
		FiberDirection();
		virtual ~FiberDirection() = default;

		void resize(const int size);

		/// @param binding the material json carrying MATERIAL_ELEMENT_INDEX /
		///        MATERIAL_ELEMENT_COUNTS (RB-11), used to bind a per-element
		///        fibre file to global or body-local rows and validate its length
		void add_multimaterial(const int index, const json &params, const std::string &unit, const std::string &root_path, const json &binding = json::object());

		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 1, 3, 3> operator()(double px, double py, double pz, double x, double y, double z, double t, int el_id) const;

		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 1, 3, 3> operator()(const Eigen::MatrixXd &param, const Eigen::MatrixXd &p, double t, int el_id) const
		{
			assert(param.size() == 2 || param.size() == 3);
			assert(param.size() == p.size());
			return (*this)(param(0), param(1), param.size() == 3 ? param(2) : 0.0,
						   p(0), p(1), p.size() == 3 ? p(2) : 0.0,
						   t, el_id);
		}

		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 1, 6, 6> stiffness_rotation_voigt(double px, double py, double pz, double x, double y, double z, double t, int el_id) const;

		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 1, 6, 6> stiffness_rotation_voigt(const Eigen::MatrixXd &param, const Eigen::MatrixXd &p, double t, int el_id) const
		{
			assert(param.size() == 2 || param.size() == 3);
			assert(param.size() == p.size());
			return this->stiffness_rotation_voigt(param(0), param(1), param.size() == 3 ? param(2) : 0.0,
												  p(0), p(1), p.size() == 3 ? p(2) : 0.0,
												  t, el_id);
		}

		bool has_rotation() const { return has_rotation_; }

	private:
		std::vector<Eigen::Matrix<utils::ExpressionValue, Eigen::Dynamic, Eigen::Dynamic, 1, 3, 3>> dir_;
		int size_ = -1; // set by resize(); -1 = unknown, the dimension checks are skipped
		bool has_rotation_ = false;

		// Per-element fiber file branch: global el_id -> unit a0.
		// Populated only when "fiber_direction" uses the per_element_file object
		// form; operator() then short-circuits to (*per_el_fibers_)[el_id] and dir_
		// is left empty. See FiberDirection::add_multimaterial in MatParams.cpp.
		// Shared within the input snapshot because add_multimaterial runs once per
		// element: re-reading and re-storing the file per element is O(n^2) in
		// both time and memory.
		std::shared_ptr<const std::vector<Eigen::Vector3d>> per_el_fibers_;
		bool use_per_element_file_ = false;
		// Identity of the loaded file, so a second, different file reaching the
		// same instance is an error rather than a silent last-writer-wins.
		std::string per_el_key_;
		std::weak_ptr<utils::MaterialFileCache> per_el_snapshot_;
		// RB-11: rows of the file are global element ids (true) or body-local
		// indices (false, then local_index_[el_id] gives the row).
		bool per_el_rows_global_ = true;
		std::vector<int> per_el_local_index_;
	};

} // namespace polyfem::assembler
