#include "HookeLinearElasticity.hpp"

#include <polyfem/autogen/auto_elasticity_rhs.hpp>
#include <polyfem/utils/Logger.hpp>

namespace polyfem::assembler
{
	using namespace basis;
	namespace
	{
		template <class Matrix>
		Matrix strain_from_disp_grad(const Matrix &disp_grad)
		{
			Matrix mat = (disp_grad + disp_grad.transpose());

			for (int i = 0; i < mat.size(); ++i)
				mat(i) *= 0.5;

			return mat;
		}

		template <int dim>
		Eigen::Matrix<double, dim, dim> strain(
			const Eigen::MatrixXd &grad,
			const Eigen::MatrixXd &jac_it,
			int k, int coo)
		{
			Eigen::Matrix<double, dim, dim> jac;
			jac.setZero();
			jac.row(coo) = grad.row(k);
			jac = jac * jac_it;

			return strain_from_disp_grad(jac);
		}

		template <typename T, unsigned long N>
		T stress(const ElasticityTensor &elasticity_tensor, const std::array<T, N> &strain, const int j)
		{
			T res = elasticity_tensor(j, 0) * strain[0];

			for (unsigned long k = 1; k < N; ++k)
				res += elasticity_tensor(j, k) * strain[k];

			return res;
		}
	} // namespace

	HookeLinearElasticity::HookeLinearElasticity()
	{
	}

	void HookeLinearElasticity::add_multimaterial(const int index, const json &params, const Units &units, const io::ResourceIO &resources)
	{
		assert(size() == 2 || size() == 3);

		// One tensor and frame per distinct definition, with a slot per element.
		// The binding index changes per element and is not part of the definition.
		const auto stress_unit = units.stress();
		const auto length_unit = units.length();
		json definition = {{"stress_unit", stress_unit}, {"length_unit", length_unit}};
		for (const char *key : {"elasticity_tensor", "young", "E", "nu", "lambda", "mu", "fiber_direction"})
			if (params.contains(key) && !params[key].empty())
				definition[key] = params[key];
		if (definition.contains("fiber_direction"))
		{
			if (params.contains(MATERIAL_ELEMENT_INDEX))
				definition["per_element_frame"] = true;
			const auto &frame = definition["fiber_direction"];
			if (frame.is_object() && frame.value("type", "") == std::string("per_element_file"))
			{
				if (params.contains(MATERIAL_ELEMENT_COUNTS))
					definition[MATERIAL_ELEMENT_COUNTS] = params[MATERIAL_ELEMENT_COUNTS];
			}
		}

		auto it = material_slots_.find(definition);
		if (it == material_slots_.end())
		{
			Material value;
			value.tensor.resize(size());
			value.frame.resize(size());
			if (definition.contains("elasticity_tensor"))
				value.tensor.set_from_entries(params["elasticity_tensor"].get<std::vector<double>>(), stress_unit, resources);
			else if (params.contains("young"))
			{
				if (params["young"].is_number() && params["nu"].is_number())
					value.tensor.set_from_young_poisson(params["young"], params["nu"], stress_unit, resources);
			}
			else if (params.contains("E"))
			{
				if (params["E"].is_number() && params["nu"].is_number())
					value.tensor.set_from_young_poisson(params["E"], params["nu"], stress_unit, resources);
			}
			else if (params.contains("lambda"))
			{
				if (params["lambda"].is_number() && params["mu"].is_number())
					value.tensor.set_from_lambda_mu(params["lambda"], params["mu"], stress_unit, resources);
			}
			if (definition.contains("fiber_direction"))
				value.frame.add_multimaterial(0, params["fiber_direction"], length_unit, resources, params);

			const std::size_t slot = material_slots_.empty() ? 0 : materials_.size();
			if (slot == 0)
				materials_.front() = std::move(value);
			else
				materials_.push_back(std::move(value));
			it = material_slots_.emplace(std::move(definition), slot).first;
		}
		if (index >= element_materials_.size())
			element_materials_.resize(index + 1, 0);
		element_materials_[index] = it->second;
	}

	const HookeLinearElasticity::Material &HookeLinearElasticity::material(const int element_id) const
	{
		if (element_materials_.empty())
			return materials_.front();
		const int index = element_materials_.size() == 1 || element_id < 0 ? 0 : element_id;
		return materials_[element_materials_.at(index)];
	}

	const ElasticityTensor &HookeLinearElasticity::elasticity_tensor(const int element_id) const
	{
		return material(element_id).tensor;
	}

	void HookeLinearElasticity::set_size(const int size)
	{
		Assembler::set_size(size);
		material_slots_.clear();
		element_materials_.clear();
		materials_.clear();
		auto &value = materials_.emplace_back();
		value.tensor.resize(size);
		value.frame.resize(size);
	}

	Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 9, 1>
	HookeLinearElasticity::assemble(const LinearAssemblerData &data) const
	{
		const Eigen::MatrixXd &gradi = data.vals.basis_values[data.i].grad;
		const Eigen::MatrixXd &gradj = data.vals.basis_values[data.j].grad;

		// (C : gradi) : gradj
		Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 9, 1> res(size() * size());
		res.setZero();
		assert(gradi.cols() == size());
		assert(gradj.cols() == size());
		assert(size_t(gradi.rows()) == data.vals.jac_it.size());

		const auto &value = material(data.vals.element_id);
		const bool rotate = value.frame.has_rotation();
		for (long k = 0; k < gradi.rows(); ++k)
		{
			Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 9, 1> res_k(size() * size());

			ElasticityTensor rotated;
			if (rotate)
			{
				rotated = value.tensor;
				rotated.rotate_stiffness(value.frame.stiffness_rotation_voigt(data.vals.quadrature.points.row(k), data.vals.val.row(k), data.t, data.vals.element_id));
			}
			const auto &elasticity_tensor = rotate ? rotated : value.tensor;

			if (size() == 2)
			{
				const Eigen::Matrix2d eps_x_i = strain<2>(gradi, data.vals.jac_it[k], k, 0);
				const Eigen::Matrix2d eps_y_i = strain<2>(gradi, data.vals.jac_it[k], k, 1);

				const Eigen::Matrix2d eps_x_j = strain<2>(gradj, data.vals.jac_it[k], k, 0);
				const Eigen::Matrix2d eps_y_j = strain<2>(gradj, data.vals.jac_it[k], k, 1);

				std::array<double, 3> e_x, e_y;
				e_x[0] = eps_x_i(0, 0);
				e_x[1] = eps_x_i(1, 1);
				e_x[2] = 2 * eps_x_i(0, 1);

				e_y[0] = eps_y_i(0, 0);
				e_y[1] = eps_y_i(1, 1);
				e_y[2] = 2 * eps_y_i(0, 1);

				Eigen::Matrix2d sigma_x;
				sigma_x << elasticity_tensor.compute_stress<3>(e_x, 0), elasticity_tensor.compute_stress<3>(e_x, 2),
					elasticity_tensor.compute_stress<3>(e_x, 2), elasticity_tensor.compute_stress<3>(e_x, 1);

				Eigen::Matrix2d sigma_y;
				sigma_y << elasticity_tensor.compute_stress<3>(e_y, 0), elasticity_tensor.compute_stress<3>(e_y, 2),
					elasticity_tensor.compute_stress<3>(e_y, 2), elasticity_tensor.compute_stress<3>(e_y, 1);

				res_k(0) = (sigma_x * eps_x_j).trace();
				res_k(2) = (sigma_x * eps_y_j).trace();

				res_k(1) = (sigma_y * eps_x_j).trace();
				res_k(3) = (sigma_y * eps_y_j).trace();
			}
			else
			{
				const Eigen::Matrix3d eps_x_i = strain<3>(gradi, data.vals.jac_it[k], k, 0);
				const Eigen::Matrix3d eps_y_i = strain<3>(gradi, data.vals.jac_it[k], k, 1);
				const Eigen::Matrix3d eps_z_i = strain<3>(gradi, data.vals.jac_it[k], k, 2);

				const Eigen::Matrix3d eps_x_j = strain<3>(gradj, data.vals.jac_it[k], k, 0);
				const Eigen::Matrix3d eps_y_j = strain<3>(gradj, data.vals.jac_it[k], k, 1);
				const Eigen::Matrix3d eps_z_j = strain<3>(gradj, data.vals.jac_it[k], k, 2);

				std::array<double, 6> e_x, e_y, e_z;
				e_x[0] = eps_x_i(0, 0);
				e_x[1] = eps_x_i(1, 1);
				e_x[2] = eps_x_i(2, 2);
				e_x[3] = 2 * eps_x_i(1, 2);
				e_x[4] = 2 * eps_x_i(0, 2);
				e_x[5] = 2 * eps_x_i(0, 1);

				e_y[0] = eps_y_i(0, 0);
				e_y[1] = eps_y_i(1, 1);
				e_y[2] = eps_y_i(2, 2);
				e_y[3] = 2 * eps_y_i(1, 2);
				e_y[4] = 2 * eps_y_i(0, 2);
				e_y[5] = 2 * eps_y_i(0, 1);

				e_z[0] = eps_z_i(0, 0);
				e_z[1] = eps_z_i(1, 1);
				e_z[2] = eps_z_i(2, 2);
				e_z[3] = 2 * eps_z_i(1, 2);
				e_z[4] = 2 * eps_z_i(0, 2);
				e_z[5] = 2 * eps_z_i(0, 1);

				Eigen::Matrix3d sigma_x;
				sigma_x << elasticity_tensor.compute_stress<6>(e_x, 0), elasticity_tensor.compute_stress<6>(e_x, 5), elasticity_tensor.compute_stress<6>(e_x, 4),
					elasticity_tensor.compute_stress<6>(e_x, 5), elasticity_tensor.compute_stress<6>(e_x, 1), elasticity_tensor.compute_stress<6>(e_x, 3),
					elasticity_tensor.compute_stress<6>(e_x, 4), elasticity_tensor.compute_stress<6>(e_x, 3), elasticity_tensor.compute_stress<6>(e_x, 2);

				Eigen::Matrix3d sigma_y;
				sigma_y << elasticity_tensor.compute_stress<6>(e_y, 0), elasticity_tensor.compute_stress<6>(e_y, 5), elasticity_tensor.compute_stress<6>(e_y, 4),
					elasticity_tensor.compute_stress<6>(e_y, 5), elasticity_tensor.compute_stress<6>(e_y, 1), elasticity_tensor.compute_stress<6>(e_y, 3),
					elasticity_tensor.compute_stress<6>(e_y, 4), elasticity_tensor.compute_stress<6>(e_y, 3), elasticity_tensor.compute_stress<6>(e_y, 2);

				Eigen::Matrix3d sigma_z;
				sigma_z << elasticity_tensor.compute_stress<6>(e_z, 0), elasticity_tensor.compute_stress<6>(e_z, 5), elasticity_tensor.compute_stress<6>(e_z, 4),
					elasticity_tensor.compute_stress<6>(e_z, 5), elasticity_tensor.compute_stress<6>(e_z, 1), elasticity_tensor.compute_stress<6>(e_z, 3),
					elasticity_tensor.compute_stress<6>(e_z, 4), elasticity_tensor.compute_stress<6>(e_z, 3), elasticity_tensor.compute_stress<6>(e_z, 2);

				res_k(0) = (sigma_x * eps_x_j).trace();
				res_k(3) = (sigma_x * eps_y_j).trace();
				res_k(6) = (sigma_x * eps_z_j).trace();

				res_k(1) = (sigma_y * eps_x_j).trace();
				res_k(4) = (sigma_y * eps_y_j).trace();
				res_k(7) = (sigma_y * eps_z_j).trace();

				res_k(2) = (sigma_z * eps_x_j).trace();
				res_k(5) = (sigma_z * eps_y_j).trace();
				res_k(8) = (sigma_z * eps_z_j).trace();
			}

			res += res_k * data.da(k);
		}

		return res;
	}

	bool HookeLinearElasticity::compute_output_stress(const OutputData &data, const ElementAssemblyValues &vals,
													  const Eigen::Index p, const MatrixNd &displacement_grad, OutputStress &output) const
	{
		const auto &local_pts = data.local_pts;
		const auto el_id = data.el_id;
		const auto &value = material(el_id);
		const bool rotate = value.frame.has_rotation();
		ElasticityTensor rotated;
		if (rotate)
		{
			rotated = value.tensor;
			rotated.rotate_stiffness(value.frame.stiffness_rotation_voigt(local_pts.row(p), vals.val.row(p), data.t, el_id));
		}
		const auto &elasticity_tensor = rotate ? rotated : value.tensor;

		MatrixNd strain = (displacement_grad + displacement_grad.transpose()) / 2;
		MatrixNd sigma(size(), size());

		if (size() == 2)
		{
			std::array<double, 3> eps;
			eps[0] = strain(0, 0);
			eps[1] = strain(1, 1);
			eps[2] = 2 * strain(0, 1);

			sigma << elasticity_tensor.compute_stress<3>(eps, 0), elasticity_tensor.compute_stress<3>(eps, 2),
				elasticity_tensor.compute_stress<3>(eps, 2), elasticity_tensor.compute_stress<3>(eps, 1);
		}
		else
		{
			std::array<double, 6> eps;
			eps[0] = strain(0, 0);
			eps[1] = strain(1, 1);
			eps[2] = strain(2, 2);
			eps[3] = 2 * strain(1, 2);
			eps[4] = 2 * strain(0, 2);
			eps[5] = 2 * strain(0, 1);

			sigma << elasticity_tensor.compute_stress<6>(eps, 0), elasticity_tensor.compute_stress<6>(eps, 5), elasticity_tensor.compute_stress<6>(eps, 4),
				elasticity_tensor.compute_stress<6>(eps, 5), elasticity_tensor.compute_stress<6>(eps, 1), elasticity_tensor.compute_stress<6>(eps, 3),
				elasticity_tensor.compute_stress<6>(eps, 4), elasticity_tensor.compute_stress<6>(eps, 3), elasticity_tensor.compute_stress<6>(eps, 2);
		}

		if (output.needs_energy)
			output.energy = 0.5 * (sigma * strain).trace();
		output.native = sigma;
		return true;
	}

	Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 3, 1>
	HookeLinearElasticity::compute_rhs(const AutodiffHessianPt &pt) const
	{
		assert(pt.size() == size());
		Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 3, 1> res;

		// The generated divergence operator differentiates displacement only.
		const auto &value = material(0);
		if (!value.frame.has_uniform_rotation())
			log_and_throw_error("HookeLinearElasticity manufactured RHS requires a constant fiber_direction rotation shared by all elements. "
								"Supply an explicit RHS for spatially varying, time-dependent, or per-element rotations.");
		if (materials_.size() > 1)
			log_and_throw_error("HookeLinearElasticity manufactured RHS requires a shared material tensor. Supply an explicit RHS for multiple materials.");

		const bool rotate = value.frame.has_rotation();
		ElasticityTensor rotated;
		if (rotate)
		{
			rotated = value.tensor;
			rotated.rotate_stiffness(value.frame.stiffness_rotation_voigt(0, 0, 0, 0, 0, 0, 0, 0));
		}
		const auto &tensor = rotate ? rotated : value.tensor;
		if (size() == 2)
			autogen::hooke_2d_function(pt, tensor, res);
		else if (size() == 3)
			autogen::hooke_3d_function(pt, tensor, res);
		else
			assert(false);

		return res;
	}

	std::map<std::string, Assembler::ParamFunc> HookeLinearElasticity::parameters() const
	{
		std::map<std::string, ParamFunc> res;
		const int size = this->size() == 2 ? 3 : 6;

		for (int i = 0; i < size; ++i)
		{
			for (int j = i; j < size; ++j)
			{
				res[fmt::format("C_{}{}", i, j)] = [this, i, j](const RowVectorNd &, const RowVectorNd &, double, int element_id) {
					return elasticity_tensor(element_id)(i, j);
				};
			}
		}
		return res;
	}

	double HookeLinearElasticity::compute_energy(const NonLinearAssemblerData &data) const
	{
		return compute_energy_aux<double>(data);
	}

	Eigen::VectorXd HookeLinearElasticity::assemble_gradient(const NonLinearAssemblerData &data) const
	{
		const int n_bases = data.vals.basis_values.size();
		return polyfem::gradient_from_energy(
			size(), n_bases, data,
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar1<double, Eigen::Matrix<double, 6, 1>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar1<double, Eigen::Matrix<double, 8, 1>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar1<double, Eigen::Matrix<double, 12, 1>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar1<double, Eigen::Matrix<double, 18, 1>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar1<double, Eigen::Matrix<double, 24, 1>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar1<double, Eigen::Matrix<double, 30, 1>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar1<double, Eigen::Matrix<double, 60, 1>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar1<double, Eigen::Matrix<double, 81, 1>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar1<double, Eigen::Matrix<double, Eigen::Dynamic, 1, 0, SMALL_N, 1>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar1<double, Eigen::Matrix<double, Eigen::Dynamic, 1, 0, BIG_N, 1>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar1<double, Eigen::VectorXd>>(data); });
	}

	void HookeLinearElasticity::assemble_hessian(const NonLinearAssemblerData &data, LocalHessian &hessian) const
	{
		const int n_bases = data.vals.basis_values.size();
		polyfem::hessian_from_energy(
			size(), n_bases, data, hessian,
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar2<double, Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar2<double, Eigen::Matrix<double, 8, 1>, Eigen::Matrix<double, 8, 8>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar2<double, Eigen::Matrix<double, 12, 1>, Eigen::Matrix<double, 12, 12>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar2<double, Eigen::Matrix<double, 18, 1>, Eigen::Matrix<double, 18, 18>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar2<double, Eigen::Matrix<double, 24, 1>, Eigen::Matrix<double, 24, 24>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar2<double, Eigen::Matrix<double, 30, 1>, Eigen::Matrix<double, 30, 30>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar2<double, Eigen::Matrix<double, 60, 1>, Eigen::Matrix<double, 60, 60>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar2<double, Eigen::Matrix<double, 81, 1>, Eigen::Matrix<double, 81, 81>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar2<double, Eigen::Matrix<double, Eigen::Dynamic, 1, 0, SMALL_N, 1>, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, SMALL_N, SMALL_N>>>(data); },
			[&](const NonLinearAssemblerData &data) { return compute_energy_aux<DScalar2<double, Eigen::VectorXd, Eigen::MatrixXd>>(data); });
	}

	template <typename T>
	T HookeLinearElasticity::compute_energy_aux(const NonLinearAssemblerData &data) const
	{
		typedef Eigen::Matrix<T, Eigen::Dynamic, 1> AutoDiffVect;
		typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, 0, 3, 3> AutoDiffGradMat;

		AutoDiffVect local_disp;
		get_local_disp(data, size(), local_disp);

		AutoDiffGradMat disp_grad(size(), size());

		T energy = T(0.0);

		const auto &value = material(data.vals.element_id);
		const bool rotate = value.frame.has_rotation();
		const int n_pts = data.da.size();
		for (long p = 0; p < n_pts; ++p)
		{
			compute_disp_grad_at_quad(data, local_disp, p, size(), disp_grad);

			ElasticityTensor rotated;
			if (rotate)
			{
				rotated = value.tensor;
				rotated.rotate_stiffness(value.frame.stiffness_rotation_voigt(
					data.vals.quadrature.points.row(p), data.vals.val.row(p), data.t, data.vals.element_id));
			}
			const auto &tensor = rotate ? rotated : value.tensor;

			AutoDiffGradMat strain = strain_from_disp_grad(disp_grad);
			AutoDiffGradMat stress_tensor(size(), size());

			if (size() == 2)
			{
				std::array<T, 3> eps;
				eps[0] = strain(0, 0);
				eps[1] = strain(1, 1);
				eps[2] = 2 * strain(0, 1);

				stress_tensor << stress(tensor, eps, 0), stress(tensor, eps, 2),
					stress(tensor, eps, 2), stress(tensor, eps, 1);
			}
			else
			{
				std::array<T, 6> eps;
				eps[0] = strain(0, 0);
				eps[1] = strain(1, 1);
				eps[2] = strain(2, 2);
				eps[3] = 2 * strain(1, 2);
				eps[4] = 2 * strain(0, 2);
				eps[5] = 2 * strain(0, 1);

				stress_tensor << stress(tensor, eps, 0), stress(tensor, eps, 5), stress(tensor, eps, 4),
					stress(tensor, eps, 5), stress(tensor, eps, 1), stress(tensor, eps, 3),
					stress(tensor, eps, 4), stress(tensor, eps, 3), stress(tensor, eps, 2);
			}

			energy += (stress_tensor * strain).trace() * data.da(p);
		}

		return energy * 0.5;
	}
} // namespace polyfem::assembler
