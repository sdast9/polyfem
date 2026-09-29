// Gradient balance on free DOFs (docs/gradient-balance-free-dofs-20260929.md):
// the opt-in semi_implicit.gradient_balance_dofs = free zeroes the Dirichlet
// rows of both gradients before calibrate_trim's balance. The scene-level
// effect is measured by tools/balance on the EF-01 matrix, IT and BB.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>

#include <ipc/collisions/normal/normal_collisions.hpp>

#include <cmath>
#include <vector>

using namespace polyfem;
using namespace polyfem::solver;

namespace
{
	// One vertex over the midpoint of a clamped edge (2D, dhat 1, gap .5):
	// the edge plays an obstacle (Dirichlet, no energy), the vertex is free
	// and pushed onto it by a constant load. The barrier gradient on the
	// edge's vertices is half the vertex's each, so on all DOFs
	// ||gB||^2 = 1.5 ||gB_free||^2 while <gB,gE> lives on the free rows:
	// the full-DOF balance is 1/1.5 of the free one, its cosine 1/sqrt(1.5).
	ipc::CollisionMesh obstacle_mesh()
	{
		Eigen::MatrixXd vertices(3, 2);
		vertices << -1, 0, 1, 0, 0, .5;
		Eigen::MatrixXi edges(1, 2);
		edges << 0, 1;
		return ipc::CollisionMesh(vertices, edges);
	}

	const std::vector<int> edge_dofs{0, 1, 2, 3};

	class BalanceForm : public BarrierContactForm
	{
	public:
		BalanceForm(const ipc::CollisionMesh &mesh, const json &options, const double load)
			: BarrierContactForm(mesh, 1., 1., false, false, false, false, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000, BarrierStiffnessMode::SemiImplicit,
								 options, Eigen::VectorXd::Ones(3))
		{
			set_system_hessian_provider([](const Eigen::VectorXd &, StiffnessMatrix &h) {
				h.resize(6, 6);
				h.setIdentity();
				h *= 100.;
			});
			// E = load * y_vertex: pushes the free vertex onto the edge.
			set_system_gradient_provider([load](const Eigen::VectorXd &x, Eigen::VectorXd &g) {
				g = Eigen::VectorXd::Zero(x.size());
				g[5] = load;
			});
		}
	};

	struct Balance
	{
		double trim_after, kappa_record;
		json record;
	};

	Balance calibrate(const json &options, const double load, const std::vector<int> &dirichlet = edge_dofs)
	{
		const auto mesh = obstacle_mesh(); // the form keeps a reference
		BalanceForm f(mesh, options, load);
		f.set_dirichlet_dofs(dirichlet, 2);
		const Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
		f.init(x);
		f.refresh_semi_implicit_stiffness(x, false);
		const json r = f.trim_predictors(x, true);
		f.calibrate_trim(x);
		return {f.barrier_stiffness(), r["gradient_balance"]["kappa_gb"].get<double>(), r};
	}
} // namespace

TEST_CASE("Free-DOF gradient balance drops the Dirichlet rows", "[gradient_balance_dofs][trim_controller]")
{
	constexpr double load = 1e6; // large enough that both balances raise the trim above 1
	const Balance all = calibrate(json::object(), load);
	const Balance explicit_all = calibrate(json{{"gradient_balance_dofs", "all"}}, load);
	const Balance free = calibrate(json{{"gradient_balance_dofs", "free"}}, load);

	// Option off is the production path bit for bit.
	CHECK(explicit_all.trim_after == all.trim_after);
	CHECK(explicit_all.kappa_record == all.kappa_record);
	CHECK_FALSE(all.record["gradient_balance"].contains("dofs"));

	// The upward-only calibration applied each mode's balance.
	REQUIRE(all.trim_after > 1);
	CHECK(all.trim_after == Catch::Approx(all.kappa_record));
	CHECK(free.trim_after == Catch::Approx(free.kappa_record));
	CHECK(free.record["gradient_balance"]["dofs"] == "free");

	// Obstacle geometry: free / all = ||gB||^2 / ||gB_free||^2 = 1.5, and the
	// free balance opposes the load exactly.
	CHECK(free.trim_after / all.trim_after == Catch::Approx(1.5));
	CHECK(free.record["gradient_balance"]["cos_opposition"].get<double>() == Catch::Approx(1.));
	CHECK(all.record["gradient_balance"]["cos_opposition"].get<double>() == Catch::Approx(1. / std::sqrt(1.5)));

	// The observational clamped block's free_dofs balance is the one the
	// option applies.
	const json g = all.record["clamped"]["gradient_balance"];
	CHECK(g["free_dofs"]["kappa_gb"].get<double>() == Catch::Approx(free.kappa_record));
	// Row split from the record's norms (tools/balance/reduce.py): the
	// obstacle's rows carry no energy, so ||gE|| is the same on both.
	CHECK(g["free_dofs"]["energy_gradient_norm"].get<double>() == g["all"]["energy_gradient_norm"].get<double>());
	const double bf = g["free_dofs"]["barrier_gradient_norm"].get<double>(), ba = g["all"]["barrier_gradient_norm"].get<double>();
	CHECK(ba * ba / (bf * bf) == Catch::Approx(1.5));

	// Without Dirichlet rows the option is an exact no-op.
	CHECK(calibrate(json{{"gradient_balance_dofs", "free"}}, load, {}).trim_after
		  == calibrate(json::object(), load, {}).trim_after);
}

TEST_CASE("Free-DOF gradient balance option is validated and reported", "[gradient_balance_dofs]")
{
	const auto mesh = obstacle_mesh();
	CHECK_THROWS(BalanceForm(mesh, json{{"gradient_balance_dofs", "reduced"}}, 1.));
	CHECK_FALSE(BarrierContactForm::parse_balance_free_dofs(json(nullptr)));
	CHECK_FALSE(BarrierContactForm::parse_balance_free_dofs(json::object()));
	CHECK(BarrierContactForm::parse_balance_free_dofs(json{{"gradient_balance_dofs", "free"}}));

	// The default is not named in the manifest (manifests stay as before);
	// free is, with its own status line.
	BalanceForm off(mesh, json::object(), 1.);
	const json d = off.model_description();
	CHECK_FALSE(d["coefficient_law"]["controller"].contains("gradient_balance"));
	BalanceForm on(mesh, json{{"gradient_balance_dofs", "free"}}, 1.);
	on.set_dirichlet_dofs(edge_dofs, 2);
	const json m = on.model_description();
	CHECK(m["coefficient_law"]["controller"]["gradient_balance"]["dofs"] == "free");
	CHECK(m["coefficient_law"]["controller"]["gradient_balance"]["dirichlet_dof_count"] == 4);
	CHECK(m["model_selection_status"].get<std::string>().find("free-DOF gradient balance") != std::string::npos);
}
