#pragma once

#include <polyfem/solver/forms/Form.hpp>

namespace polyfem::solver
{
	/// @brief Form of the augmented lagrangian
	class AugmentedLagrangianForm : public Form
	{

	public:
		AugmentedLagrangianForm() {}

		virtual ~AugmentedLagrangianForm() {}

		virtual void update_lagrangian(const Eigen::VectorXd &x, const double k_al) = 0;

		virtual double compute_error(const Eigen::VectorXd &x) const = 0;

		inline void set_initial_weight(const double k_al) { k_al_ = k_al; }

		inline double lagrangian_weight() const { return k_al_; }
		/// @brief RB-06: the multipliers (read-only, for state fingerprints and tests).
		inline const Eigen::VectorXd &lagrange_multipliers() const { return lagr_mults_; }

		inline const StiffnessMatrix &constraint_matrix() const { return A_; }
		inline const Eigen::MatrixXd &constraint_value() const { return b_; }

		inline const StiffnessMatrix &constraint_projection_matrix() const { return A_proj_; }
		inline const Eigen::MatrixXd &constraint_projection_vector() const { return b_proj_; }

		inline bool has_projection() const { return A_proj_.rows() > 0; }

		virtual bool can_project() const { return false; }
		virtual void project_gradient(Eigen::VectorXd &grad) const { assert(false); }
		virtual void project_hessian(StiffnessMatrix &hessian) const { assert(false); }
		virtual void project_diag(Eigen::VectorXd &diag) const { assert(false); }

		/// @brief sets the scale for the form
		/// @param scale
		void set_scale(const double scale) override { k_scale_ = scale; }

		/// @brief RB-06: the penalty weight and the multipliers, which every
		///        AL pass of an attempt updates (update_lagrangian) and which
		///        persist across steps; the constraint targets are rebuilt by
		///        update_quantities between steps and are not attempt state.
		struct State : public FormState
		{
			double k_al = 0;
			double k_scale = 1;
			Eigen::VectorXd lagr_mults;
		};
		std::unique_ptr<FormState> save_state() const override
		{
			auto state = std::make_unique<State>();
			save_al_state(*state);
			return state;
		}
		void restore_state(const FormState &state, const Eigen::VectorXd &) override
		{
			restore_al_state(state_as<State>(state, "AugmentedLagrangianForm"));
		}

	protected:
		void save_al_state(State &state) const
		{
			save_base_state(state);
			state.k_al = k_al_;
			state.k_scale = k_scale_;
			state.lagr_mults = lagr_mults_;
		}
		void restore_al_state(const State &state)
		{
			restore_base_state(state);
			k_al_ = state.k_al;
			k_scale_ = state.k_scale;
			lagr_mults_ = state.lagr_mults;
		}

		inline double L_weight() const { return 1 / k_scale_; }
		inline double A_weight() const { return k_al_ / k_scale_; }

		double k_al_; ///< penalty parameter

		Eigen::VectorXd lagr_mults_; ///< vector of lagrange multipliers

		StiffnessMatrix A_; ///< Constraints matrix
		Eigen::MatrixXd b_; ///< Constraints value

		StiffnessMatrix A_proj_; ///< Constraints projection matrix
		Eigen::MatrixXd b_proj_; ///< Constraints projection value
	private:
		double k_scale_ = 1;
	};
} // namespace polyfem::solver
