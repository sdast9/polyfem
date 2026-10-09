#pragma once

#include <polyfem/varforms/VarForm.hpp>

#include <polyfem/assembler/Mass.hpp>

#include <utility>

namespace polyfem::mesh
{
	class Obstacle;
}

namespace polyfem::solver
{
	class ContactForm;
	class Form;
} // namespace polyfem::solver

namespace polyfem::time_integrator
{
	class ImplicitTimeIntegrator;
}

namespace polyfem::varform
{
	/// @brief Where an exported solution stands relative to the time
	///        integrator's history (RBR-01: explicit output time state).
	///
	/// The owner of the integrator states the phase from its control flow;
	/// it is never inferred from the solution's value. A held position equals
	/// the history head without being it (a quasistatic dwell returns the
	/// previous solution bit for bit), so the reviewed equality heuristic
	/// reported the previous step's kinematics for it.
	enum class OutputTimePhase
	{
		/// The history was initialized at, or advanced to, the exported
		/// solution: x_prev() is that solution and v_prev()/a_prev() are its
		/// kinematics. The initial output of every loop; the linear,
		/// incompressible and FSI-embedding loops (which advance before
		/// saving); any output after a step's advance, including the final
		/// export_data and the restored state after a rolled-back attempt.
		HistoryHead,
		/// The exported solution is the current step's endpoint, or an
		/// iterate of it (subsolve sequences, the step callback), and the
		/// history still ends at the previous step: the nonlinear loop saves
		/// before advancing. The integrator's own rule differences it
		/// against the history, whatever its value.
		CurrentStepBeforeAdvance,
	};

	/// @brief Velocity and acceleration of a saved solution (RB-04 output
	///        kinematics alignment, RBR-01 explicit phase). In the
	///        HistoryHead phase the stored v_prev()/a_prev() are returned;
	///        in the CurrentStepBeforeAdvance phase the solution is
	///        differenced with the integrator's compute_velocity /
	///        compute_acceleration at the current history, also when it
	///        equals the head. Zero before initialization or for a solution
	///        of another size than the history.
	std::pair<Eigen::VectorXd, Eigen::VectorXd> saved_solution_kinematics(
		const time_integrator::ImplicitTimeIntegrator &time_integrator,
		const Eigen::VectorXd &solution,
		const OutputTimePhase phase);

	class ElasticVarForm : public VarForm
	{
		friend class polyfem::test::VarFormTestAccess;

	public:
		void init(const std::string &formulation, const Units &units, const json &args, const std::string &out_path) override;

		void save_json(const Eigen::MatrixXd &solution, std::ostream &out) const override;
		void export_data(const Eigen::MatrixXd &solution) const override;
		io::OutputSpace output_space() const override;
		io::OutStatsData compute_errors(const Eigen::MatrixXd &solution) override;

	protected:
		void reset() override;
		void load_mesh(const mesh::Mesh &mesh, const json &args) override;
		void build_basis(mesh::Mesh &mesh, const bool iso_parametric, const json &args) override;
		void build_elastic_basis(mesh::Mesh &mesh, const bool iso_parametric, const json &args, const int fe_space_id);
		void assemble_rhs(const mesh::Mesh &mesh) override;
		void assemble_mass_mat(const mesh::Mesh &mesh, const json &args) override;
		void build_rhs_assembler() override;

		void initial_solution(
			Eigen::MatrixXd &solution,
			const InitialConditionOverride *override = nullptr,
			const std::string &state_prefix = "") const;
		void initial_velocity(
			Eigen::MatrixXd &velocity,
			const InitialConditionOverride *override = nullptr,
			const std::string &state_prefix = "") const;
		void initial_acceleration(
			Eigen::MatrixXd &acceleration,
			const InitialConditionOverride *override = nullptr,
			const std::string &state_prefix = "") const;
		QuadratureOrders elastic_boundary_samples() const;
		std::vector<int> elastic_primitive_to_node() const;
		std::vector<int> elastic_node_to_primitive() const;
		void build_mesh_matrices(Eigen::MatrixXd &V, Eigen::MatrixXi &F) const;
		void save_elastic_step_state(
			const double t0,
			const double dt,
			const int t,
			const time_integrator::ImplicitTimeIntegrator *time_integrator) const;
		std::vector<io::OutputField> elastic_output_fields(
			const io::OutputSample &sample,
			const Eigen::MatrixXd &solution,
			const io::OutputFieldOptions &options,
			const mesh::Obstacle *obstacle,
			const time_integrator::ImplicitTimeIntegrator *time_integrator,
			const std::vector<std::pair<std::string, std::shared_ptr<solver::Form>>> &named_forms,
			const solver::Form *elastic_form,
			const solver::ContactForm *contact_form = nullptr,
			const double force_scale = -1.0) const;
		void append_primary_output_fields(
			std::vector<io::OutputField> &fields,
			const io::OutputSample &sample,
			const Eigen::MatrixXd &solution,
			const io::OutputFieldOptions &options,
			const mesh::Obstacle *obstacle = nullptr) const;
		Eigen::MatrixXd displaced_output_normals(
			const io::OutputSample &sample,
			const Eigen::MatrixXd &solution) const;

		virtual int n_obstacle_vertices() const { return 0; }

		FESpace space_;
		VarFormBoundaryState boundary_;

		assembler::AssemblyValsCache ass_vals_cache_;
		assembler::AssemblyValsCache mass_ass_vals_cache_;
		assembler::AssemblyValsCache pure_mass_ass_vals_cache_;

		std::shared_ptr<assembler::RhsAssembler> rhs_assembler_;

		StiffnessMatrix mass_;
		StiffnessMatrix pure_mass_;

		double avg_mass_ = 0;
		Eigen::MatrixXd rhs_;

		std::shared_ptr<assembler::Assembler> primary_assembler_ = nullptr;
		std::shared_ptr<assembler::Mass> mass_assembler_ = nullptr;
		std::shared_ptr<assembler::HRZMass> pure_mass_assembler_ = nullptr;

		double t0 = 0;
		int time_steps = 0;
		double dt = 0;

		/// @brief The time phase of the solution the next output describes
		///        (RBR-01). The owner of the time integrator maintains it at
		///        every transition: HistoryHead when the integrator is
		///        initialized and after every history advance (the between-
		///        steps advance, the FSI embedding advance, the restored
		///        state of a rolled-back attempt); CurrentStepBeforeAdvance
		///        from the start of a step's solve attempt until its advance.
		///        elastic_output_fields reads it for the exported velocity
		///        and acceleration. Reset to HistoryHead with the formulation.
		OutputTimePhase output_time_phase_ = OutputTimePhase::HistoryHead;
	};
} // namespace polyfem::varform
