#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/utils/MatrixUtils.hpp>

using namespace polyfem;
using namespace polyfem::solver;

namespace
{
	class MappedForm : public BarrierContactForm
	{
	public:
		MappedForm(const ipc::CollisionMesh &mesh, const Eigen::MatrixXd &h)
			: BarrierContactForm(mesh, 1., 1., false, false, false, false, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
								 BarrierStiffnessMode::SemiImplicit, json::object())
		{
			set_weight(1.);
			set_barrier_stiffness(1.);
			set_system_hessian_provider([h](const Eigen::VectorXd &, StiffnessMatrix &out) { out = h.sparseView(); });
		}
		void start(const Eigen::VectorXd &x)
		{
			init(x);
			refresh_semi_implicit_stiffness(x, false);
		}
	};

	// Independent surface-coordinate oracle. The reference form has identity
	// indexing; only the test constructs the small dense P H P^T reference.
	void compare(const ipc::CollisionMesh &mesh, const Eigen::MatrixXd &b,
				 const Eigen::MatrixXd &h, const Eigen::VectorXd &x)
	{
		const int d = mesh.dim();
		Eigen::MatrixXd p = Eigen::MatrixXd::Zero(d * b.rows(), d * b.cols());
		for (int i = 0; i < b.rows(); ++i)
			for (int j = 0; j < b.cols(); ++j)
				p.block(d * i, d * j, d, d) = b(i, j) * Eigen::MatrixXd::Identity(d, d);
		Eigen::MatrixXd padded = Eigen::MatrixXd::Zero(x.size(), x.size());
		padded.topLeftCorner(h.rows(), h.cols()) = h;
		ipc::CollisionMesh surface(mesh.rest_positions(), mesh.edges(), mesh.faces());
		MappedForm actual(mesh, h), expected(surface, p * padded * p.transpose());
		Eigen::VectorXd y = p * x;
		actual.start(x);
		expected.start(y);
		REQUIRE(actual.collision_set().size() == 1);
		REQUIRE(expected.collision_set().size() == 1);
		const double k = expected.collision_set()[0].stiffness_scale;
		CHECK(std::abs(actual.collision_set()[0].stiffness_scale - k) < 1e-10 * (1 + std::abs(k)));
		CHECK(std::abs(actual.value(x) - expected.value(y)) < 1e-10 * (1 + std::abs(expected.value(y))));
		Eigen::VectorXd ag, eg;
		actual.first_derivative(x, ag);
		expected.first_derivative(y, eg);
		CHECK((ag - p.transpose() * eg).norm() < 1e-10 * (1 + eg.norm()));
		StiffnessMatrix ah, eh;
		actual.second_derivative(x, ah);
		expected.second_derivative(y, eh);
		CHECK((Eigen::MatrixXd(ah) - p.transpose() * eh * p).norm() < 1e-10 * (1 + eh.norm()));
		actual.solution_changed(x);
		CHECK(std::abs(actual.collision_set()[0].stiffness_scale - k) < 1e-10 * (1 + std::abs(k)));
		actual.refresh_semi_implicit_stiffness(x, false);
		CHECK(std::abs(actual.collision_set()[0].stiffness_scale - k) < 1e-10 * (1 + std::abs(k)));
	}
} // namespace

TEST_CASE("Contact stiffness uses FEM node IDs for exact collision maps", "[contact_stiffness_mapping]")
{
	const int d = GENERATE(2, 3);
	const bool fem_only = GENERATE(false, true);
	const bool moving = GENERATE(false, true);
	CAPTURE(d, fem_only, moving);
	Eigen::MatrixXd rest = Eigen::MatrixXd::Zero(4, d);
	rest(0, 0) = -1;
	rest(2, 0) = 1;
	rest(3, 1) = .2;
	rest(1, 0) = 7; // excluded proxy vertex
	Eigen::MatrixXi edges(1, 2);
	edges << 0, 2;
	// Two appended obstacle proxies select system nodes 3 and 4, even
	// though their proxy IDs 0 and 2 are inside a FEM-only Hessian.
	Eigen::MatrixXd t = Eigen::MatrixXd::Zero(4, 5);
	t(0, 3) = t(1, 2) = t(2, 4) = t(3, 1) = 1.;
	ipc::CollisionMesh mesh(std::vector<bool>{true, false, true, true},
							std::vector<bool>(4, false), rest, edges, Eigen::MatrixXi(), t.sparseView());
	Eigen::MatrixXd b(3, 5);
	b.row(0) = t.row(0);
	b.row(1) = t.row(2);
	b.row(2) = t.row(3);
	const int n = d * (fem_only ? 3 : 5);
	// Distinct diagonal and cross-node/cross-component terms catch more
	// than diagonal-block permutations. This matrix is well conditioned SPD.
	Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(n, .1, 1.);
	Eigen::MatrixXd h = v * v.transpose();
	h.diagonal() += Eigen::VectorXd::LinSpaced(n, 10., 100.);
	Eigen::VectorXd x = Eigen::VectorXd::Zero(5 * d);
	if (moving)
		x[3 * d + 1] = x[4 * d + 1] = .05;
	compare(mesh, b, h, x);
}

TEST_CASE("Identity and explicit rectangular selection share stiffness", "[contact_stiffness_mapping]")
{
	const bool explicit_map = GENERATE(false, true);
	Eigen::MatrixXd rest(3, 2);
	rest << -1, 0, 1, 0, 0, .2;
	Eigen::MatrixXi edges(1, 2);
	edges << 0, 1;
	Eigen::MatrixXd b = Eigen::MatrixXd::Identity(3, explicit_map ? 4 : 3);
	StiffnessMatrix t;
	if (explicit_map)
		t = b.sparseView();
	ipc::CollisionMesh mesh(rest, edges, Eigen::MatrixXi(), t);
	Eigen::MatrixXd h = Eigen::MatrixXd::Identity(2 * b.cols(), 2 * b.cols());
	h.diagonal() = Eigen::VectorXd::LinSpaced(h.rows(), 10., 100.);
	compare(mesh, b, h, Eigen::VectorXd::Zero(h.rows()));
}

TEST_CASE("Unresolved contact maps retain whole-stencil legacy stiffness", "[contact_stiffness_mapping]")
{
	const int variant = GENERATE(0, 1, 2, 3);
	CAPTURE(variant);
	Eigen::MatrixXd rest(3, 2);
	rest << -1, 0, 1, 0, 0, .2;
	Eigen::MatrixXi edges(1, 2);
	edges << 0, 1;
	Eigen::MatrixXd b = Eigen::MatrixXd::Zero(3, 4);
	b(0, 3) = b(1, 1) = 1.;
	if (variant == 0) // genuine interpolation, with other rows permuted
		b(2, 0) = b(2, 2) = .5;
	else if (variant == 1) // scaled selector is not an exact node selection
		b(2, 2) = 2.;
	else if (variant == 2) // duplicate selector cannot prescribe independent motion
		b(2, 3) = 1.;
	// variant 3 has an empty row.
	ipc::CollisionMesh mesh(rest, edges, Eigen::MatrixXi(), b.sparseView());
	Eigen::VectorXd diagonal(8);
	diagonal << 10, 10, 20, 20, 100, 100, 400, 400;
	MappedForm form(mesh, Eigen::MatrixXd(diagonal.asDiagonal()));
	form.start(Eigen::VectorXd::Zero(8));
	REQUIRE(form.collision_set().size() == 1);
	CHECK(std::abs(form.collision_set()[0].stiffness_scale - 215. / 3.) < 1e-10);
}
