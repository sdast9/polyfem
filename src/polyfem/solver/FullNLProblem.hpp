#pragma once

#include <polyfem/Common.hpp>
#include <polyfem/solver/forms/Form.hpp>
#include <polysolve/nonlinear/Problem.hpp>

#include <functional>
#include <limits>
#include <memory>
#include <vector>

namespace polyfem::solver
{
	/// @brief RB-04: one passive observation of a nonlinear solve in full
	///        coordinates, emitted from the PolySolve hooks the problem
	///        already receives. Observers report; they cannot change the
	///        solve, and an observer that throws is disabled with a warning.
	struct IterationObservation
	{
		enum class Kind
		{
			Proposal,      ///< line_search_begin: the trial sweep [x0, x1] handed to the contact broad phase
			StepBound,     ///< max_step_size: the fraction of [x0, x1] the forms (inversion check, CCD) allow
			Validity,      ///< is_step_valid: one line-search trial [x0, x1] and the forms' verdict
			LineSearchEnd, ///< line_search_end: the swept candidate cache is released
			Accepted       ///< post_step: an accepted iterate (PolySolve emits the start point before its first iteration)
		};
		/// A Newton iteration is observed as Validity* (finite-energy stage),
		/// Proposal, StepBound, Validity* (descent stage), LineSearchEnd,
		/// Accepted. ALSolver's feasibility checks before a subsolve appear as
		/// Proposal, Validity, LineSearchEnd with no StepBound. PolySolve's
		/// post_step reports the number of iterations completed before the
		/// call, so the start point and the first update both carry 0.
		Kind kind;
		const Eigen::VectorXd *x0 = nullptr;                          ///< Current iterate (Proposal, StepBound, Validity)
		const Eigen::VectorXd *x1 = nullptr;                          ///< Trial endpoint, or the accepted iterate (Accepted)
		const Eigen::VectorXd *grad = nullptr;                        ///< Objective gradient at the accepted iterate
		double step_bound = std::numeric_limits<double>::quiet_NaN(); ///< StepBound result
		bool valid = true;                                            ///< Validity verdict
		int iteration = -1;                                           ///< Accepted: iterations completed before this post_step
		const json *solver_info = nullptr;                            ///< Accepted: PolySolve solver info at that iterate
	};

	class FullNLProblem : public polysolve::nonlinear::Problem
	{
	public:
		FullNLProblem(const std::vector<std::shared_ptr<Form>> &forms, const bool is_residual = false);
		virtual ~FullNLProblem() = default;
		virtual void init(const TVector &x0) override;

		virtual double value(const TVector &x) override;
		virtual void gradient(const TVector &x, TVector &gradv) override;
		virtual void hessian(const TVector &x, THessian &hessian) override;

		virtual bool is_step_valid(const TVector &x0, const TVector &x1) override;
		virtual bool is_step_collision_free(const TVector &x0, const TVector &x1);
		virtual double max_step_size(const TVector &x0, const TVector &x1) override;

		virtual void line_search_begin(const TVector &x0, const TVector &x1) override;
		virtual void line_search_end() override;
		virtual void post_step(const polysolve::nonlinear::PostStepData &data) override;

		virtual void set_project_to_psd(bool val) override;
		bool is_residual() const override { return is_residual_; }

		virtual void solution_changed(const TVector &new_x) override;

		virtual void init_lagging(const TVector &x);
		virtual void update_lagging(const TVector &x, const int iter_num);
		int max_lagging_iterations() const;
		bool uses_lagging() const;

		std::vector<std::shared_ptr<Form>> &forms() { return forms_; }

		virtual bool stop(const TVector &x) override { return false; }

		void finish()
		{
			for (auto &form : forms_)
				form->finish();
		}

		virtual double normalize_forms();

		/// @brief RB-04: install (or clear, with nullptr) the passive
		///        iteration observer. Disabled observation costs one null check.
		void set_iteration_observer(std::function<void(const IterationObservation &)> observer)
		{
			iteration_observer_ = std::move(observer);
			iteration_observer_failed_ = false;
		}

	protected:
		std::vector<std::shared_ptr<Form>> forms_;
		const bool is_residual_;

		void observe(const IterationObservation &observation);
		std::function<void(const IterationObservation &)> iteration_observer_ = nullptr;
		bool iteration_observer_failed_ = false;
	};
} // namespace polyfem::solver
