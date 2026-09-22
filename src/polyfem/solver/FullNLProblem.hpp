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
			Proposal,      ///< line_search_begin: the trial sweep [x0, x1] about to be handed to the contact broad phase (observed before the forms build it, so a failing build still leaves its trial on record -- RB-05)
			Extension,     ///< line_search_extend: the same line search lengthens its sweep to [x0, x1] (a growing search, BFGS audit stage 3); observed before the forms rebuild it, and priced by the StepBound that follows
			StepBound,     ///< max_step_size: the fraction of [x0, x1] the forms (inversion check, CCD) allow
			Validity,      ///< is_step_valid: one line-search trial [x0, x1] and the forms' verdict
			LineSearchEnd, ///< line_search_end: the swept candidate cache is released
			Accepted       ///< post_step: an accepted iterate (PolySolve emits the start point before its first iteration)
		};
		/// A Newton iteration is observed as Validity* (finite-energy stage),
		/// Proposal, StepBound, Validity* (descent stage), LineSearchEnd,
		/// Accepted. ALSolver's feasibility checks before a subsolve appear as
		/// Proposal, Validity, LineSearchEnd with no StepBound (the RB-07
		/// snap-fraction probe, probe_step_bound, is deliberately unobserved
		/// so that a feasibility check never reads as a Newton trial).
		/// A growing line search (PolySolve's Wolfe) adds Extension, StepBound
		/// pairs inside the same iteration, before its LineSearchEnd.
		/// PolySolve's post_step reports the number of iterations completed
		/// before the call, so the start point and the first update both
		/// carry 0.
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
		/// @brief RB-07: the forms' step bound of [x0, x1] (inversion check,
		///        CCD) without the StepBound observation, for a diagnostic
		///        probe outside a line search -- the RB-04 observer reads a
		///        proposal that carries a step bound as a Newton trial. Needs
		///        line_search_begin(x0, x1) to be active like max_step_size.
		virtual double probe_step_bound(const TVector &x0, const TVector &x1);

		virtual void line_search_begin(const TVector &x0, const TVector &x1) override;
		/// The forms rebuild exactly as for line_search_begin; only the
		/// observation differs (Extension instead of Proposal).
		virtual void line_search_extend(const TVector &x0, const TVector &x1) override;
		virtual void line_search_end() override;
		virtual void post_step(const polysolve::nonlinear::PostStepData &data) override;

		virtual void set_project_to_psd(bool val) override;
		bool is_residual() const override { return is_residual_; }

		virtual void solution_changed(const TVector &new_x) override;

		/// @brief The forms' objective generations, summed
		///
		/// Changes exactly when one of the forms stops being the same
		/// function, which is what the nonlinear solver needs in order to
		/// discard a quasi-Newton pair that would span the change. Summing
		/// keeps it monotone and needs no coordination between the forms.
		uint64_t objective_generation() const override;

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

		/// @brief RB-06: the attempt-mutable state of every form of this
		///        problem (Form::save_state), captured together at a
		///        transaction boundary. Opaque to callers; restore it with
		///        restore_state on the same problem.
		struct SavedState
		{
			virtual ~SavedState() = default;
			std::vector<std::unique_ptr<FormState>> forms;
		};
		virtual std::unique_ptr<SavedState> save_state() const;
		/// @param x The full coordinates the state was captured at.
		virtual void restore_state(const SavedState &state, const TVector &x);

		/// @brief RB-06 test hook: called after the forms' post_step of every
		///        accepted iterate with the iteration count and the full
		///        coordinates; it may throw to abandon the solve at a
		///        deterministic point (solver/advanced/failure_injection).
		///        Never installed in production; nullptr clears it.
		void set_post_step_fault(std::function<void(int, const TVector &)> fault) { post_step_fault_ = std::move(fault); }

	protected:
		std::function<void(int, const TVector &)> post_step_fault_ = nullptr;
		std::vector<std::shared_ptr<Form>> forms_;
		const bool is_residual_;

		void observe(const IterationObservation &observation);
		std::function<void(const IterationObservation &)> iteration_observer_ = nullptr;
		bool iteration_observer_failed_ = false;
		/// Set while line_search_extend passes through the (possibly
		/// overridden, coordinate-mapping) line_search_begin chain.
		bool extending_line_search_ = false;
	};
} // namespace polyfem::solver
