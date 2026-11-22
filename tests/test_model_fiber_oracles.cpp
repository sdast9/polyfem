#include <polyfem/Units.hpp>
#include <polyfem/assembler/ActiveFiber.hpp>
#include <polyfem/assembler/GrowthNeoHookean.hpp>
#include <polyfem/assembler/HGODispersion.hpp>
#include <polyfem/assembler/HGOFiber.hpp>
#include <polyfem/assembler/HookeLinearElasticity.hpp>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>

using namespace polyfem;
using namespace polyfem::assembler;

namespace
{
	const io::FileSystemIO resources(".");
	const Units si_units;

	// SI throughout. Gradients and stretches are dimensionless. Stress and energy
	// density are in Pa (J/m^3), and a 2D element has unit out-of-plane thickness.
	// All expectations are analytic. The absolute floor covers cancellation in
	// exp(z)-1 near zero. The relative term allows for the small tensor solves,
	// rotations and accumulation of quadrature contributions in double precision.
	constexpr double absolute_tolerance = 2e-11;
	constexpr double relative_tolerance = 2e-12;

	void check_scalar(double actual, double expected)
	{
		CAPTURE(actual, expected);
		CHECK(std::abs(actual - expected) <= absolute_tolerance + relative_tolerance * std::abs(expected));
	}

	template <typename Actual, typename Expected>
	void check_matrix(const Eigen::MatrixBase<Actual> &actual, const Eigen::MatrixBase<Expected> &expected)
	{
		REQUIRE(actual.rows() == expected.rows());
		REQUIRE(actual.cols() == expected.cols());
		const double max_error = (actual - expected).cwiseAbs().maxCoeff();
		const double scale = expected.cwiseAbs().maxCoeff();
		INFO("actual:\n"
			 << actual << "\nexpected:\n"
			 << expected);
		CAPTURE(max_error, scale);
		CHECK(max_error <= absolute_tolerance + relative_tolerance * scale);
	}

	// P1 simplex with X0 = 0 and Xi = e_i. The Jacobian is I, the centroid
	// integrates all these spatially constant densities exactly, and every row
	// of grad(N) is known. Local DOFs are node-major, then displacement component.
	struct AffineSimplex
	{
		const int dim;
		Eigen::MatrixXd grad_N;
		ElementAssemblyValues vals;
		QuadratureVector weights;
		Eigen::MatrixXd zero;

		explicit AffineSimplex(int dimension, int el_id = 0)
			: dim(dimension), grad_N(dim + 1, dim), zero(Eigen::MatrixXd::Zero(dim * (dim + 1), 1))
		{
			grad_N.row(0).setConstant(-1);
			grad_N.bottomRows(dim).setIdentity();
			vals.element_id = el_id;
			vals.is_volume_ = dim == 3;
			vals.quadrature.points = Eigen::MatrixXd::Constant(1, dim, 1.0 / (dim + 1));
			vals.val = vals.quadrature.points;
			vals.det = Eigen::VectorXd::Ones(1);
			vals.jac_it = {Eigen::MatrixXd::Identity(dim, dim)};
			weights = QuadratureVector::Constant(1, dim == 3 ? 1.0 / 6.0 : 0.5);
			vals.quadrature.weights = weights;
			vals.basis_values.resize(dim + 1);
			for (int i = 0; i <= dim; ++i)
			{
				auto &b = vals.basis_values[i];
				const Eigen::RowVectorXd vertex = i == 0 ? Eigen::RowVectorXd::Zero(dim).eval() : grad_N.row(i).eval();
				b.global = {basis::Local2Global(i, vertex, 1.0)};
				b.val = Eigen::MatrixXd::Constant(1, 1, 1.0 / (dim + 1));
				b.grad = grad_N.row(i);
				b.grad_t_m = b.grad;
			}
		}

		Eigen::VectorXd displacement(const Eigen::MatrixXd &grad_u) const
		{
			Eigen::VectorXd u = zero.col(0);
			for (int i = 0; i < dim; ++i)
				u.segment((i + 1) * dim, dim) = grad_u.col(i);
			return u;
		}

		Eigen::VectorXd nodal_gradient(const Eigen::MatrixXd &stress) const
		{
			Eigen::VectorXd g(zero.rows());
			for (int i = 0; i <= dim; ++i)
				g.segment(i * dim, dim) = weights(0) * stress * grad_N.row(i).transpose();
			return g;
		}
	};

	// Expose the same pointwise stress and energy callback that drives VTK
	// output. The element below separately exercises the assembled residual.
	template <typename Material>
	class SampledMaterial : public Material
	{
	public:
		void check_output(const AffineSimplex &el, double t, const MatrixNd &F, double energy, const Eigen::MatrixXd &cauchy) const
		{
			const basis::ElementBases bases;
			const OutputData data(t, el.vals.element_id, bases, bases, el.vals.quadrature.points, el.zero);
			typename Material::OutputStress output(F);
			output.needs_energy = true;
			const MatrixNd grad_u = F - MatrixNd::Identity(F.rows(), F.cols());
			REQUIRE(this->compute_output_stress(data, el.vals, 0, grad_u, output));
			REQUIRE(output.has_stress);
			check_scalar(output.energy, energy);
			check_matrix(output.tensor(ElasticityTensorType::CAUCHY), cauchy);
		}
	};

	struct Response
	{
		double energy;
		Eigen::MatrixXd stress;  // first Piola for finite strain, Cauchy for Hooke
		Eigen::MatrixXd tangent; // stress derivative applied to the prescribed increment of F
	};

	template <typename Material>
	void check_finite_strain(const SampledMaterial<Material> &law, const AffineSimplex &element,
							 const Eigen::MatrixXd &F, const Eigen::MatrixXd &increment, const Response &expected, double t = 0)
	{
		const Eigen::MatrixXd u = element.displacement(F - Eigen::MatrixXd::Identity(F.rows(), F.cols()));
		const NonLinearAssemblerData data(element.vals, t, 0, u, element.zero, element.weights);
		check_scalar(law.compute_energy(data), element.weights(0) * expected.energy);
		check_matrix(law.assemble_gradient(data), element.nodal_gradient(expected.stress));
		LocalHessian hessian;
		law.assemble_hessian(data, hessian);
		const Eigen::MatrixXd K = hessian.visit([](const auto &H) -> Eigen::MatrixXd { return H; });
		const Eigen::VectorXd v = element.displacement(increment);
		check_matrix(K * v, element.nodal_gradient(expected.tangent));
		check_matrix(K, K.transpose());
		law.check_output(element, t, F, expected.energy, expected.stress * F.transpose() / F.determinant());
	}

	Eigen::Matrix3d spatial_rotation()
	{
		// The quaternion (1,2,2,4)/5 is unit and mixes all three axes.
		return Eigen::Quaterniond(0.2, 0.4, 0.4, 0.8).toRotationMatrix();
	}

	Eigen::Matrix3d deformed_state()
	{
		Eigen::Matrix3d F;
		F << 1.21, 0.13, -0.04, 0.02, 0.95, 0.06, -0.03, 0.08, 1.12;
		return F;
	}

	Eigen::Matrix3d increment_3d()
	{
		Eigen::Matrix3d H;
		H << 0.22, -0.37, 0.11, 0.17, 0.31, -0.26, -0.05, 0.12, -0.13;
		return H;
	}

	json vector_json(const Eigen::VectorXd &a)
	{
		return std::vector<double>(a.data(), a.data() + a.size());
	}

	json frame_json(const Eigen::MatrixXd &R)
	{
		const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rows = R;
		return std::vector<double>(rows.data(), rows.data() + rows.size());
	}

	struct EngineeringConstants
	{
		std::vector<double> json_entries;
		Eigen::MatrixXd normal_compliance;
		Eigen::MatrixXd shear_moduli;
	};

	EngineeringConstants plane_stress_constants()
	{
		const double Ex = 80e6, Ey = 22e6, nu_xy = 0.25, Gxy = 6e6;
		Eigen::Matrix2d compliance, G;
		compliance << 1 / Ex, -nu_xy / Ex, -nu_xy / Ex, 1 / Ey;
		G << 0, Gxy, Gxy, 0;
		return {{Ex, Ey, nu_xy, Gxy}, compliance, G};
	}

	EngineeringConstants volume_constants()
	{
		const double Ex = 92e6, Ey = 54e6, Ez = 36e6, nu_xy = 0.24, nu_xz = 0.16, nu_yz = 0.19;
		const double Gyz = 3.8e6, Gzx = 8.3e6, Gxy = 6.1e6;
		Eigen::Matrix3d compliance, G;
		compliance << 1 / Ex, -nu_xy / Ex, -nu_xz / Ex,
			-nu_xy / Ex, 1 / Ey, -nu_yz / Ey,
			-nu_xz / Ex, -nu_yz / Ey, 1 / Ez;
		G << 0, Gxy, Gzx, Gxy, 0, Gyz, Gzx, Gyz, 0;
		return {{Ex, Ey, Ez, nu_xy, nu_xz, nu_yz, Gyz, Gzx, Gxy}, compliance, G};
	}

	Eigen::MatrixXd orthotropic_stress(const EngineeringConstants &mat, const Eigen::MatrixXd &R, const Eigen::MatrixXd &grad_u)
	{
		// The columns of R are the material axes. Solve the normal compliance
		// equations and use sigma_ij = 2 G_ij eps_ij componentwise in those axes.
		// Transform the physical stress tensor, without Voigt rotations or packing.
		const Eigen::MatrixXd strain = 0.5 * R.transpose() * (grad_u + grad_u.transpose()) * R;
		Eigen::MatrixXd stress = 2 * mat.shear_moduli.cwiseProduct(strain);
		stress.diagonal() = mat.normal_compliance.ldlt().solve(strain.diagonal());
		return R * stress * R.transpose();
	}

	Eigen::MatrixXd hooke_stiffness(const HookeLinearElasticity &law, const AffineSimplex &element)
	{
		const int d = element.dim;
		Eigen::MatrixXd K(d * (d + 1), d * (d + 1));
		for (int i = 0; i <= d; ++i)
			for (int j = 0; j <= d; ++j)
			{
				const auto block = law.assemble(LinearAssemblerData(element.vals, 0, i, j, element.weights));
				K.block(i * d, j * d, d, d) = Eigen::Map<const Eigen::MatrixXd>(block.data(), d, d);
			}
		return K;
	}

	void check_rotated_hooke(const EngineeringConstants &mat, const Eigen::MatrixXd &R, const Eigen::MatrixXd &grad_u)
	{
		const int dim = R.rows();
		SampledMaterial<HookeLinearElasticity> law;
		law.set_size(dim);
		law.add_multimaterial(0, {{"elasticity_tensor", mat.json_entries}, {"fiber_direction", frame_json(R)}}, si_units, resources);
		const AffineSimplex element(dim);
		const Eigen::MatrixXd stress = orthotropic_stress(mat, R, grad_u);
		const double density = 0.5 * stress.cwiseProduct(grad_u).sum(); // symmetric stress annihilates the skew part
		law.check_output(element, 0, Eigen::MatrixXd::Identity(dim, dim) + grad_u, density, stress);
		const Eigen::MatrixXd K = hooke_stiffness(law, element);
		const Eigen::VectorXd u = element.displacement(grad_u);
		check_matrix(K * u, element.nodal_gradient(stress));
		check_scalar(0.5 * u.dot(K * u), element.weights(0) * density);
		check_matrix(K, K.transpose());
	}

	struct Invariant
	{
		double value;
		Eigen::MatrixXd gradient;
		Eigen::MatrixXd directional_gradient;
	};

	double contraction(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b)
	{
		return a.cwiseProduct(b).sum();
	}

	// q(F) = J^-beta (F : F A). For symmetric A, D(F : F A)[H] = 2 FA : H.
	// DJ[H]/J = F^-T : H and D(F^-T)[H] = -F^-T H^T F^-T.
	// These identities give the directional derivative of dq/dF by the product rule.
	Invariant fiber_invariant(const Eigen::MatrixXd &F, const Eigen::MatrixXd &H, const Eigen::MatrixXd &A, double beta)
	{
		const Eigen::MatrixXd FA = F * A;
		const Eigen::MatrixXd inverse_transpose = F.inverse().transpose();
		const double trace = contraction(inverse_transpose, H);
		const double a = contraction(F, FA);
		const double da = 2 * contraction(FA, H);
		const double w = std::pow(F.determinant(), -beta);
		const Eigen::MatrixXd gradient = w * (2 * FA - beta * a * inverse_transpose);
		const Eigen::MatrixXd tangent = w * (2 * H * A - beta * da * inverse_transpose + beta * a * inverse_transpose * H.transpose() * inverse_transpose)
										- beta * trace * gradient;
		return {w * a, gradient, tangent};
	}

	// For W = f(q), P = f' dq/dF and DP[H] is the chain-rule derivative.
	Response response(const Invariant &q, const Eigen::MatrixXd &H, double W, double first, double second)
	{
		return {W, first * q.gradient, first * q.directional_gradient + second * contraction(q.gradient, H) * q.gradient};
	}

	Response hgo_response(const Eigen::MatrixXd &F, const Eigen::MatrixXd &H, const Eigen::VectorXd &a, double k1, double k2)
	{
		const Invariant q = fiber_invariant(F, H, a * a.transpose(), 2.0 / F.rows());
		const double s = q.value - 1;
		if (s <= 0)
			return response(q, H, 0, 0, 0);
		const double z = k2 * s * s;
		const double e = std::exp(z);
		return response(q, H, k1 / (2 * k2) * std::expm1(z), k1 * s * e, k1 * e * (1 + 2 * z));
	}

	Response dispersion_response(const Eigen::MatrixXd &F, const Eigen::MatrixXd &H, const Eigen::VectorXd &a,
								 double k1, double k2, double kappa, double k_chi)
	{
		const Eigen::VectorXd n = a.normalized();
		const Eigen::MatrixXd A = kappa * Eigen::MatrixXd::Identity(F.rows(), F.cols()) + (1 - F.rows() * kappa) * n * n.transpose();
		const Invariant q = fiber_invariant(F, H, A, 0);
		const double s = q.value - 1;
		const double chi = 1 / (1 + std::exp(-k_chi * s));
		const double dchi = k_chi * chi * (1 - chi);
		const double ddchi = k_chi * (1 - 2 * chi) * dchi;
		const double z = k2 * s * s;
		const double b = k1 / (2 * k2) * std::expm1(z);
		const double db = k1 * s * std::exp(z);
		const double ddb = k1 * (1 + 2 * z) * std::exp(z);
		return response(q, H, chi * b, chi * db + dchi * b, chi * ddb + 2 * dchi * db + ddchi * b);
	}

	// This model uses the isochoric invariant of the unit director. At any
	// fixed activation the multiplier does not depend on the deformation.
	Response active_response(const Eigen::MatrixXd &F, const Eigen::MatrixXd &H, const Eigen::VectorXd &a, double tension)
	{
		const Eigen::VectorXd n = a.normalized();
		const Invariant q = fiber_invariant(F, H, n * n.transpose(), 2.0 / F.rows());
		return response(q, H, -0.5 * tension * (q.value - 1), -0.5 * tension, 0);
	}

	struct TemporaryFolder
	{
		const std::filesystem::path path = std::filesystem::temp_directory_path() / ("polyfem-fiber-oracle-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
		TemporaryFolder() { REQUIRE(std::filesystem::create_directory(path)); }
		~TemporaryFolder()
		{
			std::error_code error;
			std::filesystem::remove_all(path, error);
		}
	};

	void write_fiber_rows(const std::filesystem::path &path, const std::vector<Eigen::Vector3d> &directions)
	{
		std::ofstream file(path);
		file << "# vtk DataFile Version 3.0\nfiber oracle\nASCII\nDATASET UNSTRUCTURED_GRID\n"
			 << "CELL_DATA " << directions.size() << "\nVECTORS DECOY double\n";
		for (size_t i = 0; i < directions.size(); ++i)
			file << "0 1 0\n";
		file << "VECTORS TEST_DIRECTOR double\n";
		for (const auto &a : directions)
			file << a.transpose() << '\n';
		file.close();
		REQUIRE(file);
	}
} // namespace

TEST_CASE("orthotropic Hooke plane stress in oblique material axes", "[model_oracle][orthotropic]")
{
	Eigen::Matrix2d R, grad_u;
	R << 0.8, -0.6, 0.6, 0.8;
	grad_u << 0.013, 0.017, -0.003, -0.007;
	check_rotated_hooke(plane_stress_constants(), R, grad_u);
}

TEST_CASE("orthotropic Hooke distinguishes all three engineering shear moduli", "[model_oracle][orthotropic]")
{
	const auto mat = volume_constants();
	const AffineSimplex element(3);
	SampledMaterial<HookeLinearElasticity> law;
	law.set_size(3);
	law.add_multimaterial(0, {{"elasticity_tensor", mat.json_entries}}, si_units, resources);
	const Eigen::MatrixXd K = hooke_stiffness(law, element);
	const double gamma = 0.01;
	for (const auto &[i, j] : std::array<std::pair<int, int>, 3>{{{1, 2}, {2, 0}, {0, 1}}})
	{
		CAPTURE(i, j);
		Eigen::Matrix3d F = Eigen::Matrix3d::Identity(), stress = Eigen::Matrix3d::Zero();
		F(i, j) = gamma;
		const double tau = mat.shear_moduli(i, j) * gamma;
		stress(i, j) = stress(j, i) = tau;
		const double density = 0.5 * tau * gamma;
		law.check_output(element, 0, F, density, stress);
		const Eigen::VectorXd u = element.displacement(F - Eigen::Matrix3d::Identity());
		check_matrix(K * u, element.nodal_gradient(stress));
		check_scalar(0.5 * u.dot(K * u), element.weights(0) * density);
	}
}

TEST_CASE("orthotropic Hooke mixed strains in a fully rotated 3D frame", "[model_oracle][orthotropic]")
{
	const Eigen::Matrix3d grad_u = 0.06 * increment_3d();
	check_rotated_hooke(volume_constants(), spatial_rotation(), grad_u);
}

TEST_CASE("HGOFiber one-sided cutoff and tangent in 2D and 3D", "[model_oracle][fiber]")
{
	const double k1 = 1720, k2 = 3.7;
	for (const int dim : {2, 3})
	{
		const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim, dim);
		const Eigen::VectorXd a = I.col(0);
		const Eigen::MatrixXd H = increment_3d().topLeftCorner(dim, dim);
		SampledMaterial<HGOFiber> law;
		law.set_size(dim);
		law.add_multimaterial(0, {{"k1", k1}, {"k2", k2}, {"fiber_direction", vector_json(a)}}, si_units, resources);
		const AffineSimplex element(dim);
		for (const double delta : {-0.16, -2e-5, 2e-5, 0.19})
		{
			CAPTURE(dim, delta);
			// The determinant of F/1.17 is one, so q - 1 = delta. The
			// dilation makes an accidental use of full I4 easy to detect.
			const double stretch = std::sqrt(1 + delta);
			Eigen::MatrixXd F = 1.17 * std::pow(stretch, -1.0 / (dim - 1)) * I;
			F(0, 0) = 1.17 * stretch;
			check_finite_strain(law, element, F, H, hgo_response(F, H, a, k1, k2));
		}
	}
}

TEST_CASE("HGODispersion logistic compression and transition stresses", "[model_oracle][fiber]")
{
	const double k1 = 1300, k2 = 3.2, kappa = 0.17, k_chi = 23;
	const Eigen::Vector3d a(3, 0, 4); // HGODispersion uses a unit director
	SampledMaterial<HGODispersion> law;
	law.set_size(3);
	law.add_multimaterial(0, {{"k1", k1}, {"k2", k2}, {"kappa", kappa}, {"k_chi", k_chi}, {"fiber_direction", vector_json(a)}}, si_units, resources);
	const AffineSimplex element(3);
	const Eigen::Matrix3d H = increment_3d();
	for (const double delta : {-0.04, -0.008, 0.0, 0.008, 0.04})
	{
		CAPTURE(delta);
		// Under F = sqrt(1+delta) I, the unit-trace A gives q - 1 = delta.
		// Unlike HGOFiber, the full-invariant law responds to this dilation.
		const Eigen::Matrix3d F = std::sqrt(1 + delta) * Eigen::Matrix3d::Identity();
		check_finite_strain(law, element, F, H, dispersion_response(F, H, a, k1, k2, kappa, k_chi));
	}
}

TEST_CASE("ActiveFiber clamps time-dependent activation under extension and compression", "[model_oracle][fiber]")
{
	const Eigen::Vector3d a(0, 3, -4);
	const Eigen::Matrix3d F = deformed_state();
	const Eigen::Matrix3d compressed = 0.93 * Eigen::Vector3d(1.27, 0.78, 0.81).asDiagonal();
	const Eigen::Matrix3d H = increment_3d();
	const double Tmax = 750;
	SampledMaterial<ActiveFiber> law;
	law.set_size(3);
	law.add_multimaterial(0, {{"Tmax", Tmax}, {"activation", "0.75*t - 0.2"}, {"fiber_direction", vector_json(a)}}, si_units, resources);
	const AffineSimplex element(3);
	for (const auto &[time, activation] : std::array<std::pair<double, double>, 3>{{{0, 0}, {0.8, 0.4}, {2, 1}}})
	{
		CAPTURE(time, activation);
		for (const auto &state : {F, compressed})
			check_finite_strain(law, element, state, H, active_response(state, H, a, Tmax * activation), time);
	}
}

TEST_CASE("fiber sign and rigid rotations preserve the analytic material response", "[model_oracle][fiber]")
{
	const Eigen::Vector3d a(0.6, 0.0, 0.8);
	const Eigen::Matrix3d F = deformed_state(), H = increment_3d(), Q = spatial_rotation();
	const AffineSimplex element(3);
	const double k1 = 1100, k2 = 2.5, kappa = 0.19, k_chi = 19, Tmax = 900, activation = 0.7;
	const auto check = [&](auto &law, json params, const Response &base) {
		REQUIRE(base.stress.norm() > 1); // every invariance check has a nonzero signal
		law.set_size(3);
		const auto at = [&](const Eigen::Matrix3d &left, const Eigen::Matrix3d &right, double sign) {
			params["fiber_direction"] = vector_json(sign * right * a);
			law.add_multimaterial(0, params, si_units, resources);
			const Response expected{base.energy, left * base.stress * right.transpose(), left * base.tangent * right.transpose()};
			check_finite_strain(law, element, left * F * right.transpose(), left * H * right.transpose(), expected);
		};
		const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
		at(I, I, 1);
		at(I, I, -1);
		at(Q, I, 1); // spatial objectivity, reference fiber unchanged
		at(Q, Q, 1); // reference axes and deformed body rotate together
	};
	SECTION("HGOFiber")
	{
		SampledMaterial<HGOFiber> law;
		check(law, {{"k1", k1}, {"k2", k2}}, hgo_response(F, H, a, k1, k2));
	}
	SECTION("HGODispersion")
	{
		SampledMaterial<HGODispersion> law;
		check(law, {{"k1", k1}, {"k2", k2}, {"kappa", kappa}, {"k_chi", k_chi}}, dispersion_response(F, H, a, k1, k2, kappa, k_chi));
	}
	SECTION("ActiveFiber")
	{
		SampledMaterial<ActiveFiber> law;
		check(law, {{"Tmax", Tmax}, {"activation", activation}}, active_response(F, H, a, Tmax * activation));
	}
}

TEST_CASE("HGOFiber global and body-local VTK rows drive the corresponding elements", "[model_oracle][fiber]")
{
	const TemporaryFolder folder;
	const auto path = folder.path / "fiber.vtk";
	const io::FileSystemIO files(folder.path.string());
	const double k1 = 1440, k2 = 2.6;
	const Eigen::Matrix3d F = deformed_state(), H = increment_3d();
	const std::vector<int> bodies{17, 42, 17, 42};
	const auto check = [&](const std::vector<Eigen::Vector3d> &rows, const std::array<int, 4> &row_index) {
		write_fiber_rows(path, rows);
		json mat = {{"type", "HGOFiber"}, {"k1", k1}, {"k2", k2}, {"fiber_direction", {{"type", "per_element_file"}, {"path", "fiber.vtk"}, {"field", "TEST_DIRECTOR"}}}};
		mat["id"] = 17;
		json other = mat;
		other["id"] = 42;
		SampledMaterial<HGOFiber> law;
		law.set_size(3);
		law.set_materials(bodies, json::array({mat, other}), si_units, files);
		for (int el = 0; el < 4; ++el)
		{
			CAPTURE(el, row_index[el]);
			const AffineSimplex element(3, el);
			const Eigen::Vector3d a = rows[row_index[el]].normalized();
			check_finite_strain(law, element, F, H, hgo_response(F, H, a, k1, k2));
		}
	};
	SECTION("four rows follow the interleaved global order")
	{
		check({{2, 0, 0}, {0, 3, 0}, {0, 0, -2}, {3, 0, 4}}, {0, 1, 2, 3});
	}
	SECTION("two rows are indexed independently within each body")
	{
		check({{2, 0, 0}, {0, 3, 0}}, {0, 0, 1, 1});
	}
}

TEST_CASE("GrowthNeoHookean stress and tangent use the original reference volume", "[model_oracle][growth]")
{
	const double lambda = 3200, mu = 1300;
	Eigen::Matrix3d G;
	G << 1.12, 0.07, -0.03, 0.07, 0.92, 0.04, -0.03, 0.04, 1.19;
	SampledMaterial<GrowthNeoHookean> law;
	law.set_size(3);
	law.add_multimaterial(0, {{"lambda", lambda}, {"mu", mu}, {"g11", G(0, 0)}, {"g22", G(1, 1)}, {"g33", G(2, 2)}, {"g12", G(0, 1)}, {"g13", G(0, 2)}, {"g23", G(1, 2)}}, si_units, resources);
	const AffineSimplex element(3);
	const Eigen::Matrix3d H = increment_3d(), Ginv = G.inverse();
	const Eigen::Matrix3d grown_stress_free = spatial_rotation() * G;
	for (const Eigen::Matrix3d &F : {grown_stress_free, deformed_state()})
	{
		// Use the stated frozen G. Chain differentiation of det(G) W(F G^-1)
		// pushes both stress and its derivative back to the original reference.
		const Eigen::Matrix3d Fe = F * Ginv, He = H * Ginv, invT = Fe.inverse().transpose();
		const double logJ = std::log(Fe.determinant());
		const double W = 0.5 * mu * (Fe.squaredNorm() - 3) - mu * logJ + 0.5 * lambda * logJ * logJ;
		const Eigen::Matrix3d P = mu * Fe + (lambda * logJ - mu) * invT;
		const Eigen::Matrix3d dP = mu * He + lambda * contraction(invT, He) * invT - (lambda * logJ - mu) * invT * He.transpose() * invT;
		const double Jg = G.determinant();
		check_finite_strain(law, element, F, H, {Jg * W, Jg * P * Ginv.transpose(), Jg * dP * Ginv.transpose()});
	}
}
