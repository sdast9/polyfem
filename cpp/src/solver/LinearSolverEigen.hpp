#pragma once

////////////////////////////////////////////////////////////////////////////////
#include "LinearSolverEigen.h"
#include <iostream>
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Direct solvers
////////////////////////////////////////////////////////////////////////////////

// Get info on the last solve step
template<typename SparseSolver>
void poly_fem::LinearSolverEigenDirect<SparseSolver>::getInfo(json &params) const {
	switch (m_Solver.info()) {
	case Eigen::Success:        params["solver_info"] = "Success";        break;
	case Eigen::NumericalIssue: params["solver_info"] = "NumericalIssue"; break;
	case Eigen::NoConvergence:  params["solver_info"] = "NoConvergence";  break;
	case Eigen::InvalidInput:   params["solver_info"] = "InvalidInput";   break;
	default: assert(false);
	}
}

// Analyze sparsity pattern
template<typename SparseSolver>
void poly_fem::LinearSolverEigenDirect<SparseSolver>::analyzePattern(const SparseMatrixXd &A) {
	m_Solver.analyzePattern(A);
}

// Factorize system matrix
template<typename SparseSolver>
void poly_fem::LinearSolverEigenDirect<SparseSolver>::factorize(const SparseMatrixXd &A) {
	m_Solver.factorize(A);
	if (m_Solver.info() == Eigen::NumericalIssue) {
		std::cerr << "[LinearSolver] NumericalIssue encountered." << std::endl;
	}
}

// Solve the linear system
template<typename SparseSolver>
void poly_fem::LinearSolverEigenDirect<SparseSolver>::solve(
	const Ref<const VectorXd> b, Ref<VectorXd> x)
{
	x = m_Solver.solve(b);
}

////////////////////////////////////////////////////////////////////////////////
// Iterative solvers
////////////////////////////////////////////////////////////////////////////////

// Set solver parameters
template<typename SparseSolver>
void poly_fem::LinearSolverEigenIterative<SparseSolver>::setParameters(const json &params) {
	if (params.count("max_iter")) {
		m_Solver.setMaxIterations(params["max_iter"]);
	}
	if (params.count("tolerance")) {
		m_Solver.setTolerance(params["tolerance"]);
	}
}

// Get info on the last solve step
template<typename SparseSolver>
void poly_fem::LinearSolverEigenIterative<SparseSolver>::getInfo(json &params) const {
	params["solver_iter"] = m_Solver.iterations();
	params["solver_error"] = m_Solver.error();
}

// Analyze sparsity pattern
template<typename SparseSolver>
void poly_fem::LinearSolverEigenIterative<SparseSolver>::analyzePattern(const SparseMatrixXd &A) {
	m_Solver.analyzePattern(A);
}

// Factorize system matrix
template<typename SparseSolver>
void poly_fem::LinearSolverEigenIterative<SparseSolver>::factorize(const SparseMatrixXd &A) {
	m_Solver.factorize(A);
}

// Solve the linear system
template<typename SparseSolver>
void poly_fem::LinearSolverEigenIterative<SparseSolver>::solve(
	const Ref<const VectorXd> b, Ref<VectorXd> x)
{
	x = m_Solver.solveWithGuess(b, x);
}