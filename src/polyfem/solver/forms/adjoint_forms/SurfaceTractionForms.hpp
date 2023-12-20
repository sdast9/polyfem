#pragma once

#include "SpatialIntegralForms.hpp"

#include <polyfem/solver/forms/ContactForm.hpp>

namespace polyfem::solver
{
	class TractionNormForm : public SpatialIntegralForm
	{
	public:
		TractionNormForm(const std::vector<std::shared_ptr<VariableToSimulation>> &variable_to_simulations, const State &state, const json &args) : SpatialIntegralForm(variable_to_simulations, state, args)
		{
			set_integral_type(SpatialIntegralType::surface);

			auto tmp_ids = args["surface_selection"].get<std::vector<int>>();
			ids_ = std::set(tmp_ids.begin(), tmp_ids.end());

			if (args["power"] > 0)
				in_power_ = args["power"];
		}

	protected:
		IntegrableFunctional get_integral_functional() const override;

	private:
		int in_power_ = 2;
	};

	class ContactForceForm : public StaticForm
	{
	public:
		ContactForceForm(const std::vector<std::shared_ptr<VariableToSimulation>> &variable_to_simulations, const State &state, const json &args) : StaticForm(variable_to_simulations), state_(state)
		{
			auto tmp_ids = args["surface_selection"].get<std::vector<int>>();
			ids_ = std::set(tmp_ids.begin(), tmp_ids.end());

			build_active_nodes();
		}
		~ContactForceForm() = default;

		const State &get_state() { return state_; }

		double value_unweighted_step(const int time_step, const Eigen::VectorXd &x) const override;
		Eigen::VectorXd compute_adjoint_rhs_unweighted_step(const int time_step, const Eigen::VectorXd &x, const State &state) const override;
		Eigen::VectorXd compute_adjoint_rhs_unweighted_step_prev(const int time_step, const Eigen::VectorXd &x, const State &state) const override;
		virtual void compute_partial_gradient_unweighted_step(const int time_step, const Eigen::VectorXd &x, Eigen::VectorXd &gradv) const override;

		void build_active_nodes();

	protected:
		const State &state_;
		std::set<int> ids_;
		Eigen::VectorXi active_nodes_;
		StiffnessMatrix active_nodes_mat_;
		int dim_;

		double dhat_, epsv_, friction_coefficient_;
	};

	class ProxyContactForceForm : public StaticForm
	{
	public:
		ProxyContactForceForm(
			const std::vector<std::shared_ptr<VariableToSimulation>> &variable_to_simulations,
			const State &state,
			const double dhat,
			const bool quadratic_potential,
			const json &args);
		~ProxyContactForceForm() = default;

		double value_unweighted_step(const int time_step, const Eigen::VectorXd &x) const override;
		Eigen::VectorXd compute_adjoint_rhs_unweighted_step(const int time_step, const Eigen::VectorXd &x, const State &state) const override;
		void compute_partial_gradient_unweighted_step(const int time_step, const Eigen::VectorXd &x, Eigen::VectorXd &gradv) const override;
		void solution_changed(const Eigen::VectorXd &x) override
		{
			collision_set_indicator_.setZero();
			build_collision_mesh();
		}

	protected:
		void build_collision_mesh();
		const ipc::Collisions &get_or_compute_collision_set(const int time_step, const Eigen::MatrixXd &displaced_surface) const;

		const State &state_;
		std::set<int> boundary_ids_;
		std::map<int, std::set<int>> boundary_ids_to_dof_;
		Eigen::MatrixXi can_collide_cache_;

		mutable Eigen::VectorXi collision_set_indicator_;
		std::vector<std::shared_ptr<ipc::Collisions>> collision_sets_;

		ipc::CollisionMesh collision_mesh_;
		const double dhat_;
		const double dmin_ = 0;
		ipc::BroadPhaseMethod broad_phase_method_;

		ipc::BarrierPotential barrier_potential_;
	};

	// class ContactForceMatchForm : public StaticForm
	// {
	// public:
	// 	ContactForceMatchForm(const std::vector<std::shared_ptr<VariableToSimulation>> &variable_to_simulations, const State &state, const json &args) : StaticForm(variable_to_simulations), state_(state)
	// 	{
	// 		auto tmp_ids = args["surface_selection"].get<std::vector<int>>();
	// 		ids_ = std::set(tmp_ids.begin(), tmp_ids.end());

	// 		build_active_nodes(args["force_matching_function"].get<std::vector<std::string>>());
	// 	}
	// 	~ContactForceMatchForm() = default;

	// 	const State &get_state() { return state_; }

	// 	double value_unweighted_step(const int time_step, const Eigen::VectorXd &x) const override;
	// 	Eigen::VectorXd compute_adjoint_rhs_unweighted_step(const int time_step, const Eigen::VectorXd &x, const State &state) const override;
	// 	Eigen::VectorXd compute_adjoint_rhs_unweighted_step_prev(const int time_step, const Eigen::VectorXd &x, const State &state) const override;
	// 	virtual void compute_partial_gradient_unweighted_step(const int time_step, const Eigen::VectorXd &x, Eigen::VectorXd &gradv) const override;

	// 	void build_active_nodes(const std::vector<std::string> &closed_form_forces);

	// protected:
	// 	const State &state_;
	// 	std::set<int> ids_;
	// 	Eigen::VectorXi active_nodes_;
	// 	// Eigen::VectorXd node_area_scaling_;
	// 	StiffnessMatrix active_nodes_mat_;
	// 	int dim_;

	// 	Eigen::MatrixXd matched_forces_;

	// 	double dhat_, epsv_, friction_coefficient_;
	// };

	// class ContactForceForm : public ContactForceMatchForm
	// {
	// 	ContactForceForm(const std::vector<std::shared_ptr<VariableToSimulation>> &variable_to_simulations, const State &state, const json &args) : StaticForm(variable_to_simulations), state_(state)
	// 	{
	// 		auto tmp_ids = args["surface_selection"].get<std::vector<int>>();
	// 		ids_ = std::set(tmp_ids.begin(), tmp_ids.end());

	// 		build_active_nodes({"0", "0", "0"});
	// 	}
	// }
} // namespace polyfem::solver