// Contacts between Dirichlet-clamped primitives (docs/clamped-contacts-20260928.md):
// the clamped-vertex map, the collision classes, the opt-in controller
// exclusion and the can_collide filter. The scene-level effect is measured
// by tools/clamped on the EF-01 matrix, IT and ball-burst.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polysolve/nonlinear/PostStepData.hpp>

#include <ipc/collisions/normal/normal_collisions.hpp>

#include <algorithm>
#include <vector>

using namespace polyfem;
using namespace polyfem::solver;

namespace
{
	// Two vertex-over-edge pairs (2D, dhat 1): a pinched pair (gap .01) made
	// of vertices 0-2 and a comfortable one (gap .8, inside the rms band) of
	// vertices 3-5.
	ipc::CollisionMesh two_pair_mesh()
	{
		Eigen::MatrixXd vertices(6, 2);
		vertices << -1, 0, 1, 0, 0, .01, -1, 5, 1, 5, 0, 5.8;
		Eigen::MatrixXi edges(2, 2);
		edges << 0, 1, 3, 4;
		return ipc::CollisionMesh(vertices, edges);
	}

	// DOFs of vertices 0-2: the pinched pair is fully clamped.
	std::vector<int> pinched_pair_dofs() { return {0, 1, 2, 3, 4, 5}; }

	class ClampForm : public BarrierContactForm
	{
	public:
		ClampForm(const ipc::CollisionMesh &mesh, const json &options)
			: BarrierContactForm(mesh, 1., 1., false, false, false, false, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000, BarrierStiffnessMode::SemiImplicit,
								 options, Eigen::VectorXd::Ones(6))
		{
			set_system_hessian_provider([](const Eigen::VectorXd &, StiffnessMatrix &h) {
				h.resize(12, 12);
				h.setIdentity();
				h *= 100.;
			});
		}
		using BarrierContactForm::collision_set;
	};

	// Trim change over three accepted iterations at the rest configuration
	// (the in-solve collapse bump waits for a three-iteration cooldown).
	double trim_change_after_iterations(const json &options, const std::vector<int> &dirichlet)
	{
		const auto mesh = two_pair_mesh();
		ClampForm f(mesh, options);
		f.set_dirichlet_dofs(dirichlet, 2);
		const Eigen::VectorXd x = Eigen::VectorXd::Zero(12);
		f.init(x);
		f.refresh_semi_implicit_stiffness(x, false);
		const double before = f.barrier_stiffness();
		for (int i = 1; i <= 3; ++i)
			f.post_step(polysolve::nonlinear::PostStepData(i, json::object(), x, Eigen::VectorXd::Zero(12)));
		return f.barrier_stiffness() / before;
	}
} // namespace

TEST_CASE("Clamped collision vertices need every component of every parent", "[clamped_contacts]")
{
	const auto mesh = two_pair_mesh();
	// Vertex 0 fully prescribed, vertex 1 only in x, vertex 5 fully.
	const auto clamped = BarrierContactForm::clamped_collision_vertices(mesh, {0, 1, 2, 10, 11}, 2);
	CHECK(clamped == std::vector<bool>{true, false, false, false, false, true});

	// An interpolated vertex (row with two parents) is clamped only when both are.
	Eigen::MatrixXd rest(3, 2);
	rest << 0, 0, 1, 0, .5, 1;
	Eigen::MatrixXi edges(1, 2);
	edges << 0, 1;
	std::vector<Eigen::Triplet<double>> t{{0, 0, 1.}, {1, 1, 1.}, {2, 2, .5}, {2, 3, .5}};
	Eigen::SparseMatrix<double> map(3, 4);
	map.setFromTriplets(t.begin(), t.end());
	const ipc::CollisionMesh interpolated(rest, edges, Eigen::MatrixXi(), map);
	CHECK(BarrierContactForm::clamped_collision_vertices(interpolated, {4, 5}, 2) == std::vector<bool>{false, false, false});
	CHECK(BarrierContactForm::clamped_collision_vertices(interpolated, {4, 5, 6, 7}, 2) == std::vector<bool>{false, false, true});
}

TEST_CASE("Collision classes follow the stencil vertices", "[clamped_contacts]")
{
	const auto mesh = two_pair_mesh();
	ClampForm f(mesh, json::object());
	const Eigen::VectorXd x = Eigen::VectorXd::Zero(12);
	f.init(x);
	REQUIRE(f.collision_set().size() == 2);
	// Nothing installed: every collision is free.
	for (size_t i = 0; i < 2; ++i)
		CHECK(f.clamp_class(f.collision_set(), i) == 0);

	f.set_dirichlet_dofs({0, 1, 2, 3}, 2); // the edge of the pinched pair
	std::vector<int> classes;
	for (size_t i = 0; i < 2; ++i)
		classes.push_back(f.clamp_class(f.collision_set(), i));
	std::sort(classes.begin(), classes.end());
	CHECK(classes == std::vector<int>{0, 1});

	f.set_dirichlet_dofs(pinched_pair_dofs(), 2);
	classes.clear();
	for (size_t i = 0; i < 2; ++i)
		classes.push_back(f.clamp_class(f.collision_set(), i));
	std::sort(classes.begin(), classes.end());
	CHECK(classes == std::vector<int>{0, 2});
}

TEST_CASE("Trim-predictor record compares the statistics without fully clamped contacts", "[clamped_contacts][trim_predictors]")
{
	const auto mesh = two_pair_mesh();
	ClampForm f(mesh, json::object());
	f.set_dirichlet_dofs(pinched_pair_dofs(), 2);
	const Eigen::VectorXd x = Eigen::VectorXd::Zero(12);
	f.init(x);
	f.refresh_semi_implicit_stiffness(x, false);
	const json r = f.trim_predictors(x, true)["clamped"];
	CHECK(r["mode"] == "keep");
	CHECK(r["clamped_vertex_count"] == 3);
	CHECK(r["active"]["fully"] == 1);
	CHECK(r["active"]["free"] == 1);
	CHECK(r["min_pair_class"] == 2);
	CHECK(r["all"]["min_gap"].get<double>() == Catch::Approx(.01));
	CHECK(r["all"]["collapse"] == true);
	CHECK(r["excluding_fully"]["min_gap"].get<double>() == Catch::Approx(.8));
	CHECK(r["excluding_fully"]["band_rms"].get<double>() == Catch::Approx(.8));
	CHECK(r["excluding_fully"]["collapse"] == false);
	CHECK(r["batch"]["fully_keys_in_batch"].get<int>() >= 1);
	// Without a Dirichlet list the record is absent: missing, not zero.
	ClampForm g(mesh, json::object());
	g.init(x);
	CHECK_FALSE(g.trim_predictors(x, false).contains("clamped"));
}

TEST_CASE("Excluding fully clamped contacts from the controller", "[clamped_contacts][trim_controller]")
{
	// keep: the clamped pinched pair sets the minimum gap and bumps the trim.
	CHECK(trim_change_after_iterations(json::object(), pinched_pair_dofs()) > 1);
	CHECK(trim_change_after_iterations(json{{"clamped_contacts", "keep"}}, pinched_pair_dofs()) > 1);
	// exclude_statistics: the controller sees only the comfortable pair.
	CHECK(trim_change_after_iterations(json{{"clamped_contacts", "exclude_statistics"}}, pinched_pair_dofs()) == 1);
	// Without a fully clamped collision the option follows keep exactly.
	CHECK(trim_change_after_iterations(json{{"clamped_contacts", "exclude_statistics"}}, {0, 1, 2, 3})
		  == trim_change_after_iterations(json::object(), {0, 1, 2, 3}));
	// A pinched pair with a free vertex still counts.
	CHECK(trim_change_after_iterations(json{{"clamped_contacts", "exclude_statistics"}}, {0, 1, 2, 3}) > 1);
	// The option acts on the controller only: the pair keeps its barrier.
	const auto mesh = two_pair_mesh();
	ClampForm keep(mesh, json::object()), excl(mesh, json{{"clamped_contacts", "exclude_statistics"}});
	const Eigen::VectorXd x = Eigen::VectorXd::Zero(12);
	for (ClampForm *f : {&keep, &excl})
	{
		f->set_dirichlet_dofs(pinched_pair_dofs(), 2);
		f->init(x);
	}
	CHECK(excl.collision_set().size() == 2);
	CHECK(excl.value(x) == keep.value(x));
}

TEST_CASE("Clamped-contact option is validated and reported", "[clamped_contacts]")
{
	const auto mesh = two_pair_mesh();
	CHECK_THROWS(ClampForm(mesh, json{{"clamped_contacts", "ignore"}}));
	CHECK_FALSE(ClampForm(mesh, json::object()).model_description()["coefficient_law"]["controller"].contains("clamped_contacts"));
	ClampForm f(mesh, json{{"clamped_contacts", "exclude_statistics"}});
	f.set_dirichlet_dofs(pinched_pair_dofs(), 2);
	const json m = f.model_description()["coefficient_law"]["controller"]["clamped_contacts"];
	CHECK(m["mode"] == "exclude_statistics");
	CHECK(m["clamped_vertex_count"] == 3);
	CHECK(BarrierContactForm::parse_clamped_contacts(json(nullptr)) == BarrierContactForm::ClampedContacts::Keep);
}

TEST_CASE("exclude_collisions filter drops only all-clamped candidates", "[clamped_contacts]")
{
	auto mesh = two_pair_mesh();
	mesh.can_collide = BarrierContactForm::clamped_collision_filter(
		BarrierContactForm::clamped_collision_vertices(mesh, pinched_pair_dofs(), 2));
	ipc::NormalCollisions collisions;
	collisions.build(mesh, mesh.rest_positions(), 1.);
	CHECK(collisions.size() == 1);
	// One free vertex keeps the pinched pair.
	mesh.can_collide = BarrierContactForm::clamped_collision_filter(
		BarrierContactForm::clamped_collision_vertices(mesh, {0, 1, 2, 3}, 2));
	collisions.build(mesh, mesh.rest_positions(), 1.);
	CHECK(collisions.size() == 2);
}
