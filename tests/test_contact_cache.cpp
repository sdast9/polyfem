#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>

#include <algorithm>
#include <array>
#include <future>
#include <vector>

using namespace polyfem;
using namespace polyfem::solver;

namespace
{
	const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);

	ipc::CollisionMesh make_mesh(bool alternate = false)
	{
		Eigen::MatrixXd vertices(3, 2);
		vertices << -1, 0, 1, 0, 0, .2;
		Eigen::MatrixXi edges(1, 2);
		edges << 0, (alternate ? 2 : 1);
		return ipc::CollisionMesh(vertices, edges);
	}

	// Only the reference exposes a direct rebuild, bypassing the production
	// position-cache path entirely. Stiffness/derivatives use the real form.
	class ReferenceForm : public BarrierContactForm
	{
	public:
		ReferenceForm(const ipc::CollisionMesh &mesh, double support, BarrierStiffnessMode mode)
			: BarrierContactForm(mesh, support, 1., false, false, false, false, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000, mode,
								 json::object(), Eigen::VectorXd::Ones(3))
		{
			set_system_hessian_provider([](const Eigen::VectorXd &, StiffnessMatrix &h) {
				h.resize(6, 6);
				h.setIdentity();
				h *= 100.;
			});
		}

		void direct_rebuild(const Eigen::VectorXd &x)
		{
			collision_set_.build(collision_mesh_, compute_displaced_surface(x), dhat_, dmin_, broad_phase_.get());
			assign_collision_stiffness(collision_set_);
		}
	};

	struct Sample
	{
		std::vector<std::array<long, 5>> stencils;
		double energy;
		Eigen::VectorXd gradient;
		Eigen::MatrixXd hessian;
	};

	Sample sample(const BarrierContactForm &form, const ipc::CollisionMesh &mesh, const Eigen::VectorXd &x)
	{
		Sample result;
		const auto &collisions = form.collision_set();
		for (size_t i = 0; i < collisions.size(); ++i)
		{
			const auto ids = collisions[i].vertex_ids(mesh.edges(), mesh.faces());
			const long type = collisions.is_vertex_vertex(i) ? 0 : collisions.is_edge_vertex(i) ? 1
															   : collisions.is_edge_edge(i)     ? 2
																								: 3;
			result.stencils.push_back({{type, long(ids[0]), long(ids[1]), long(ids[2]), long(ids[3])}});
		}
		std::sort(result.stencils.begin(), result.stencils.end());
		result.energy = form.value(x);
		form.first_derivative(x, result.gradient);
		StiffnessMatrix h;
		form.second_derivative(x, h);
		result.hessian = Eigen::MatrixXd(h);
		return result;
	}

	Sample reference(const ipc::CollisionMesh &mesh, double support, BarrierStiffnessMode mode, const Eigen::VectorXd &x = zero)
	{
		ReferenceForm form(mesh, support, mode);
		form.direct_rebuild(zero);
		form.refresh_semi_implicit_stiffness(zero, false);
		form.direct_rebuild(x);
		return sample(form, mesh, x);
	}

	void check(const Sample &actual, const Sample &expected)
	{
		CHECK(actual.stencils == expected.stencils);
		CHECK(std::isfinite(actual.energy));
		CHECK(actual.gradient.allFinite());
		CHECK(actual.hessian.allFinite());
		// Unit-scale geometry, gap .2, Hessian 100 I, far from singularity.
		CHECK(std::abs(actual.energy - expected.energy) <= 1e-10 + 1e-10 * std::abs(expected.energy));
		CHECK((actual.gradient - expected.gradient).norm() <= 1e-10 + 1e-10 * expected.gradient.norm());
		CHECK((actual.hessian - expected.hessian).norm() <= 1e-10 + 1e-10 * expected.hessian.norm());
	}
} // namespace

TEST_CASE("Contact forms own their collision sets independent of evaluation order", "[contact_cache]")
{
	const auto mode = GENERATE(BarrierStiffnessMode::Fixed, BarrierStiffnessMode::SemiImplicit);
	const bool reverse = GENERATE(false, true);
	const int variant = GENERATE(0, 1, 2); // same geometry; different topology; different support
	CAPTURE(int(mode), reverse, variant);
	auto mesh_a = make_mesh();
	auto mesh_b = make_mesh(variant == 1);
	const double support_b = variant == 2 ? .1 : 1.;
	const auto expected_a = reference(mesh_a, 1., mode);
	const auto expected_b = reference(mesh_b, support_b, mode);
	REQUIRE(expected_a.stencils.size() == 1);
	REQUIRE(expected_a.energy > 0);
	ReferenceForm a(mesh_a, 1., mode), b(mesh_b, support_b, mode);
	auto evaluate = [&](ReferenceForm &form, const ipc::CollisionMesh &mesh, const Sample &expected) {
		form.init(zero);
		form.refresh_semi_implicit_stiffness(zero, false);
		check(sample(form, mesh, zero), expected);
	};
	if (reverse)
	{
		evaluate(b, mesh_b, expected_b);
		evaluate(a, mesh_a, expected_a);
		evaluate(b, mesh_b, expected_b);
	}
	else
	{
		evaluate(a, mesh_a, expected_a);
		evaluate(b, mesh_b, expected_b);
		evaluate(a, mesh_a, expected_a);
	}
	// A new simulation/form must initialize even after an equal-position form.
	ReferenceForm fresh(mesh_a, 1., mode);
	fresh.init(zero);
	fresh.refresh_semi_implicit_stiffness(zero, false);
	check(sample(fresh, mesh_a, zero), expected_a);
}

TEST_CASE("Contact rebuilds follow candidate and filter lifecycle at unchanged positions", "[contact_cache]")
{
	const auto mode = GENERATE(BarrierStiffnessMode::Fixed, BarrierStiffnessMode::SemiImplicit);
	auto mesh = make_mesh();
	const auto expected = reference(mesh, 1., mode);
	ReferenceForm form(mesh, 1., mode);
	form.init(zero);
	form.refresh_semi_implicit_stiffness(zero, false);
	form.solution_changed(zero);
	check(sample(form, mesh, zero), expected);

	// Collision topology has const accessors; the supported mutable filter
	// changes candidate eligibility without changing any coordinates.
	mesh.can_collide = [](size_t, size_t) { return false; };
	const auto empty = reference(mesh, 1., mode);
	REQUIRE(empty.stencils.empty());
	form.line_search_begin(zero, zero);
	form.solution_changed(zero);
	check(sample(form, mesh, zero), empty);
	form.line_search_end();
	form.solution_changed(zero);
	check(sample(form, mesh, zero), empty);

	mesh.can_collide = [](size_t, size_t) { return true; };
	// End of the candidate interval must permit a full rebuild at the same x.
	form.update_quantities(1., zero);
	check(sample(form, mesh, zero), expected);
	form.line_search_begin(zero, zero);
	form.solution_changed(zero);
	check(sample(form, mesh, zero), expected);
	form.line_search_end();
	form.init(zero);
	check(sample(form, mesh, zero), expected);

	// A changed filter is also honored outside a cached-candidate interval.
	mesh.can_collide = [](size_t, size_t) { return false; };
	form.solution_changed(zero);
	check(sample(form, mesh, zero), empty);
}

TEST_CASE("Frozen contact derivatives agree with finite differences after rebuilding", "[contact_cache]")
{
	auto mesh = make_mesh();
	ReferenceForm form(mesh, 1., BarrierStiffnessMode::SemiImplicit);
	form.init(zero);
	form.refresh_semi_implicit_stiffness(zero, false);
	const auto center = sample(form, mesh, zero);
	REQUIRE(center.energy > 0);
	Eigen::VectorXd fd_gradient(6);
	Eigen::MatrixXd fd_hessian(6, 6);
	const double step = 1e-6;
	for (int j = 0; j < 6; ++j)
	{
		Eigen::VectorXd x = zero;
		x[j] += step;
		form.solution_changed(x);
		const auto plus = sample(form, mesh, x);
		x[j] -= 2 * step;
		form.solution_changed(x);
		const auto minus = sample(form, mesh, x);
		fd_gradient[j] = (plus.energy - minus.energy) / (2 * step);
		fd_hessian.col(j) = (plus.gradient - minus.gradient) / (2 * step);
	}
	// Central differences at a finite .2 gap: truncation and cancellation
	// justify a looser 1e-6 absolute + relative norm tolerance than equality.
	CHECK((fd_gradient - center.gradient).norm() <= 1e-6 + 1e-6 * center.gradient.norm());
	CHECK((fd_hessian - center.hessian).norm() <= 1e-6 + 1e-6 * center.hessian.norm());
}

TEST_CASE("Independent contact forms can rebuild concurrently", "[contact_cache][contact_cache_parallel]")
{
	// Each worker owns its mesh, broad phase, form, and Hessian callback.
	// No Catch assertions run on workers and no same-form mutation is shared.
	std::promise<void> start;
	const auto ready = start.get_future().share();
	auto worker = [ready]() {
		auto mesh = make_mesh();
		ReferenceForm form(mesh, 1., BarrierStiffnessMode::SemiImplicit);
		ready.wait();
		form.init(zero);
		form.refresh_semi_implicit_stiffness(zero, false);
		std::vector<Sample> samples;
		for (int i = 0; i < 16; ++i)
		{
			Eigen::VectorXd x = zero;
			x[5] = .01 * (i % 3);
			form.line_search_begin(zero, x);
			form.solution_changed(x);
			samples.push_back(sample(form, mesh, x));
			form.line_search_end();
		}
		return samples;
	};
	auto a = std::async(std::launch::async, worker);
	auto b = std::async(std::launch::async, worker);
	start.set_value();
	const auto samples_a = a.get(), samples_b = b.get();
	auto mesh = make_mesh();
	for (int i = 0; i < 16; ++i)
	{
		Eigen::VectorXd x = zero;
		x[5] = .01 * (i % 3);
		const auto expected = reference(mesh, 1., BarrierStiffnessMode::SemiImplicit, x);
		REQUIRE(expected.energy > 0);
		check(samples_a[i], expected);
		check(samples_b[i], expected);
	}
}

TEST_CASE("Physical diagnostic snapshots preserve contact state and frozen derivatives", "[physical_diagnostics]")
{
	const auto mesh = make_mesh();
	ReferenceForm form(mesh, 1., BarrierStiffnessMode::SemiImplicit);
	form.init(zero);
	form.refresh_semi_implicit_stiffness(zero, false);
	const auto before = sample(form, mesh, zero);
	const auto state = form.diagnostic_state();
	Eigen::VectorXd x = zero;
	x[4] = 1.1; // Different closest stencil; only the snapshot may memoize it.
	x[5] = .1;
	const auto snapshot = form.diagnostic_snapshot(x);
	CHECK(form.diagnostic_state() == state);
	check(sample(form, mesh, zero), before);
	check(sample(snapshot, mesh, x), reference(mesh, 1., BarrierStiffnessMode::SemiImplicit, x));
	Eigen::VectorXd fd(6);
	for (int j = 0; j < 6; ++j)
	{
		Eigen::VectorXd plus = x, minus = x;
		plus[j] += 1e-6;
		minus[j] -= 1e-6;
		fd[j] = (form.diagnostic_snapshot(plus).value(plus) - form.diagnostic_snapshot(minus).value(minus)) / 2e-6;
	}
	Eigen::VectorXd g;
	snapshot.first_derivative(x, g);
	CHECK((g - fd).norm() / (1 + g.norm()) < 1e-6);
	CHECK(form.diagnostic_state() == state);
	// On/off continuation: a subsequent production rebuild has the same result.
	form.solution_changed(x);
	check(sample(form, mesh, x), sample(snapshot, mesh, x));
	Eigen::VectorXd absent = zero;
	absent[5] = 2;
	const auto empty = form.diagnostic_snapshot(absent);
	CHECK(empty.diagnostic_state()["active_count"] == 0);
	CHECK(empty.diagnostic_state()["coefficient_range"]["value"].is_null());
	CHECK(empty.diagnostic_state()["candidate_count"]["value"].is_null());
}
