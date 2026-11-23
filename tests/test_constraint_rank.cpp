#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>

#include <polyfem/solver/NLProblem.hpp>
#include <polyfem/solver/forms/lagrangian/MatrixLagrangianForm.hpp>
#include <polyfem/utils/Logger.hpp>

#include <polysolve/linear/Solver.hpp>

using namespace polyfem;
using namespace polyfem::solver;

namespace
{
	class WeightedDistance : public Form
	{
	public:
		std::string name() const override { return "weighted-distance"; }

	protected:
		double value_unweighted(const Eigen::VectorXd &x) const override
		{
			const Eigen::Vector3d offset = x - center;
			return 0.5 * offset.dot(weights.cwiseProduct(offset));
		}

		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &grad) const override
		{
			grad = weights.cwiseProduct(x - center);
		}

		void second_derivative_unweighted(const Eigen::VectorXd &, StiffnessMatrix &hessian) const override
		{
			hessian = weights.asDiagonal().toDenseMatrix().sparseView();
		}

	private:
		const Eigen::Vector3d center{1, -2, 7};
		const Eigen::Vector3d weights{2, 1, 3};
	};

	class TimeVaryingZeroEquation : public MatrixLagrangianForm
	{
	public:
		TimeVaryingZeroEquation(const StiffnessMatrix &a)
			: MatrixLagrangianForm(a, Eigen::VectorXd::Zero(a.rows()))
		{
			set_initial_weight(1);
		}

		void update_quantities(const double t, const Eigen::VectorXd &) override { b_(0, 0) = t; }
	};

	Eigen::VectorXd quadratic_minimum(NLProblem &problem, const Eigen::VectorXd &start)
	{
		Eigen::VectorXd gradient;
		StiffnessMatrix tangent;
		problem.gradient(start, gradient);
		problem.hessian(start, tangent);
		return start + Eigen::MatrixXd(tangent).ldlt().solve(-gradient);
	}
} // namespace

TEST_CASE("Overdetermined sparse constraints retain the correct free subspace", "[constraints][constraint_rank]")
{
	Eigen::Matrix<double, 7, 3> equations;
	equations << 1, 1, 0,
		0, 0, 1,
		2, 2, 0,
		3, 3, -1,
		-1, -1, 2,
		0, 0, -2,
		0, 0, 0;
	Eigen::Matrix<double, 7, 1> prescribed;
	prescribed << 1, 2, 2, 1, 3, -4, 0;
	const StiffnessMatrix a = equations.sparseView();
	const auto constraint = std::make_shared<MatrixLagrangianForm>(a, prescribed);
	const std::shared_ptr<polysolve::linear::Solver> linear = polysolve::linear::Solver::create(json{{"solver", "Eigen::SparseLU"}}, logger());
	StiffnessMatrix mass(3, 3);
	mass.setIdentity();
	NLProblem problem(3, 0, {std::make_shared<WeightedDistance>()}, {constraint}, linear, 1, 1, mass, 1);
	REQUIRE(problem.full_size() - problem.reduced_size() == 2);

	// The first two equations are independent and imply the other five.
	// Substitute x_1 = 1 - x_0 and x_2 = 2. The derivative of the objective
	// along this line is 2(x_0 - 1) + (x_0 - 3) = 3 x_0 - 5.
	const Eigen::Vector3d feasible(3, -2, 2);
	const Eigen::Vector3d expected(5.0 / 3, -2.0 / 3, 2);
	const double tolerance = 1e-12; // Allow roundoff from QR with these small integer coefficients.
	const Eigen::VectorXd start = problem.full_to_reduced(feasible);
	CHECK((problem.reduced_to_full(start) - feasible).norm() < tolerance);
	const Eigen::VectorXd reduced = quadratic_minimum(problem, start);
	const Eigen::VectorXd full = problem.reduced_to_full(reduced);
	CHECK((equations * full - prescribed).norm() < tolerance);
	CHECK((full - expected).norm() < tolerance);
	Eigen::VectorXd gradient;
	problem.gradient(reduced, gradient);
	CHECK(gradient.norm() < tolerance);
}

TEST_CASE("Rank-zero sparse equations leave all DOFs free and reject inconsistent targets", "[constraints][constraint_rank]")
{
	StiffnessMatrix a(7, 3);
	SECTION("empty sparsity pattern") {}
	SECTION("explicitly stored zero") { a.insert(2, 1) = 0; }
	const auto constraint = std::make_shared<TimeVaryingZeroEquation>(a);
	const std::shared_ptr<polysolve::linear::Solver> linear = polysolve::linear::Solver::create(json{{"solver", "Eigen::SparseLU"}}, logger());
	StiffnessMatrix mass(3, 3);
	mass.setIdentity();
	NLProblem problem(3, 0, {std::make_shared<WeightedDistance>()}, {constraint}, linear, 1, 1, mass, 1);
	REQUIRE(problem.reduced_size() == 3);
	const Eigen::Vector3d start = Eigen::Vector3d::Zero();
	const Eigen::Vector3d expected(1, -2, 7);
	const Eigen::VectorXd full = quadratic_minimum(problem, start);
	CHECK((problem.reduced_to_full(full) - expected).norm() < 1e-12);
	CHECK((problem.full_to_reduced_grad(expected) - expected).norm() == 0);
	CHECK_THROWS_WITH(problem.update_quantities(1, full), "Rank-zero constraints require zero right-hand sides");
}
