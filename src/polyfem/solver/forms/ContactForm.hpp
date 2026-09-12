#pragma once

#include <limits>
#include "Form.hpp"

#include <polyfem/Common.hpp>
#include <polyfem/utils/Types.hpp>

#include <ipc/collisions/normal/normal_collisions.hpp>
#include <ipc/collision_mesh.hpp>
#include <ipc/broad_phase/create_broad_phase.hpp>
#include <ipc/potentials/potential.hpp>

// map BroadPhaseMethod values to JSON as strings
namespace ipc
{
	// map ipc::BroadPhaseMethod values to JSON as strings
	NLOHMANN_JSON_SERIALIZE_ENUM(
		ipc::BroadPhaseMethod,
		{{ipc::BroadPhaseMethod::HASH_GRID, "hash_grid"}, // also default
		 {ipc::BroadPhaseMethod::HASH_GRID, "HG"},
		 {ipc::BroadPhaseMethod::BRUTE_FORCE, "brute_force"},
		 {ipc::BroadPhaseMethod::BRUTE_FORCE, "BF"},
		 {ipc::BroadPhaseMethod::SPATIAL_HASH, "spatial_hash"},
		 {ipc::BroadPhaseMethod::SPATIAL_HASH, "SH"},
		 {ipc::BroadPhaseMethod::LBVH, "bvh"},
		 {ipc::BroadPhaseMethod::LBVH, "BVH"},
		 {ipc::BroadPhaseMethod::LBVH, "LBVH"},
		 {ipc::BroadPhaseMethod::SWEEP_AND_TINIEST_QUEUE, "sweep_and_tiniest_queue"},
		 {ipc::BroadPhaseMethod::SWEEP_AND_TINIEST_QUEUE, "STQ"}})
} // namespace ipc

namespace polyfem::solver
{
	/// @brief RB-05: read the opt-in broad-phase resource limits from the
	///        `solver/contact/CCD` options (`resource_limits/max_cell_items`,
	///        `resource_limits/max_candidate_emissions`); absent or zero
	///        fields leave the corresponding bound disabled.
	ipc::BroadPhaseBudget broad_phase_budget_from_args(const json &ccd_args);

	/// @brief How the barrier stiffness is chosen and updated
	enum class BarrierStiffnessMode
	{
		Fixed,        ///< User-provided fixed stiffness
		Adaptive,     ///< Classic IPC adaptive stiffness [Li et al. 2020]
		SemiImplicit, ///< Per-contact elasticity-inclusive stiffness [Ando 2024]
	};

	/// @brief Form representing the contact potential and forces
	class ContactForm : public Form
	{
	public:
		/// @brief Construct a new Contact Form object
		/// @param collision_mesh Reference to the collision mesh
		/// @param dhat Barrier activation distance
		/// @param avg_mass Average mass of the mesh
		/// @param use_adaptive_barrier_stiffness If true, use an adaptive barrier stiffness
		/// @param is_time_dependent Is the simulation time dependent?
		/// @param broad_phase_method Broad phase method to use for distance and CCD evaluations
		/// @param ccd_tolerance Continuous collision detection tolerance
		/// @param ccd_max_iterations Continuous collision detection maximum iterations
		ContactForm(const ipc::CollisionMesh &collision_mesh,
					const double dhat,
					const double avg_mass,
					const bool use_adaptive_barrier_stiffness,
					const bool is_time_dependent,
					const bool enable_shape_derivatives,
					const ipc::BroadPhaseMethod broad_phase_method,
					const double ccd_tolerance,
					const int ccd_max_iterations);
		virtual ~ContactForm() = default;

		virtual std::string name() const override { return "contact"; }

		/// @brief Initialize the form
		/// @param x Current solution
		virtual void init(const Eigen::VectorXd &x) override;

	public:
		/// @brief Update time-dependent fields
		/// @param t Current time
		/// @param x Current solution at time t
		virtual void update_quantities(const double t, const Eigen::VectorXd &x) override;

		/// @brief Determine the maximum step size allowable between the current and next solution
		/// @param x0 Current solution (step size = 0)
		/// @param x1 Next solution (step size = 1)
		/// @return Maximum allowable step size
		virtual double max_step_size(const Eigen::VectorXd &x0, const Eigen::VectorXd &x1) const override;

		/// @brief Initialize variables used during the line search
		/// @param x0 Current solution
		/// @param x1 Next solution
		virtual void line_search_begin(const Eigen::VectorXd &x0, const Eigen::VectorXd &x1) override;

		/// @brief Clear variables used during the line search
		void line_search_end() override;

		/// @brief Update cached fields upon a change in the solution
		/// @param new_x New solution
		virtual void solution_changed(const Eigen::VectorXd &new_x) override;

		/// @brief Checks if the step is collision free
		/// @return True if the step is collision free else false
		virtual bool is_step_collision_free(const Eigen::VectorXd &x0, const Eigen::VectorXd &x1) const override;

		/// @brief Update the barrier stiffness based on the current elasticity energy
		/// @param x Current solution
		virtual void update_barrier_stiffness(const Eigen::VectorXd &x, const Eigen::MatrixXd &grad_energy) = 0;

		/// @brief Compute the displaced positions of the surface nodes
		Eigen::MatrixXd compute_displaced_surface(const Eigen::VectorXd &x) const;

		/// @brief Get the current barrier stiffness
		double barrier_stiffness() const { return barrier_stiffness_; }
		/// @brief Set the current barrier stiffness
		void set_barrier_stiffness(const double barrier_stiffness) { barrier_stiffness_ = barrier_stiffness; }
		/// @brief Get use_adaptive_barrier_stiffness
		bool use_adaptive_barrier_stiffness() const { return use_adaptive_barrier_stiffness_; }
		/// @brief Get use_convergent_formulation
		virtual bool use_convergent_formulation() const { return false; }

		bool enable_shape_derivatives() const { return enable_shape_derivatives_; }

		double weight() const override { return weight_ * barrier_stiffness_; }

		/// @brief If true, output debug files
		bool save_ccd_debug_meshes = false;

		double dhat() const { return dhat_; }

		std::shared_ptr<ipc::BroadPhase> get_broad_phase() const { return broad_phase_; }

		/// @brief RB-04: broad-phase candidate counts of the trial sweeps
		///        handed to CCD by line_search_begin since the last reset.
		///        The swept cache itself is cleared at line_search_end, so
		///        an endpoint record can only report these retained counts.
		///        RB-05 adds the broad phase's own intermediates when the
		///        method measures them (hash grid cell items; pair emissions
		///        are counted only while a budget is enabled).
		struct CandidateStatistics
		{
			size_t builds = 0;                   ///< completed line_search_begin builds since the reset
			size_t last = 0;                     ///< candidates of the most recent build
			size_t max = 0;                      ///< largest build since the reset
			bool intermediates_measured = false; ///< the broad phase reported its buffers
			size_t last_cell_items = 0;          ///< hash-grid (box, cell) items of the last build
			size_t max_cell_items = 0;           ///< largest item count since the reset
			size_t last_candidate_emissions = 0; ///< pre-filter pair emissions of the last build (budget enabled only)
			size_t max_candidate_emissions = 0;  ///< largest emission count since the reset
		};
		const CandidateStatistics &candidate_statistics() const { return candidate_statistics_; }
		void reset_candidate_statistics() { candidate_statistics_ = CandidateStatistics(); }

		/// @brief RB-05: opt-in bound on the broad phase's intermediate buffers,
		///        enforced by the toolkit before the corresponding allocation
		///        (ipc::BroadPhaseBudget). Zero fields disable the bound (the
		///        default, bit-identical to the unbudgeted path). A method that
		///        cannot enforce a budget is refused here, at configuration
		///        time, with a named error -- never silently ignored.
		void set_broad_phase_budget(const ipc::BroadPhaseBudget &budget);
		const ipc::BroadPhaseBudget &broad_phase_budget() const { return broad_phase_->budget; }

	protected:
		/// @brief Update the cached candidate set for the current solution
		/// @param displaced_surface Vertex positions displaced by the current solution
		virtual void update_collision_set(const Eigen::MatrixXd &displaced_surface) = 0;

		/// @brief Drop the swept candidate cache and leave the cached-candidate
		///        interval. The cache is valid only inside the line search that
		///        built it; init and update_quantities start a new solve/step
		///        and call this first, because PolySolve's line search calls no
		///        line_search_end while an exception unwinds through it (RB-05).
		void discard_swept_candidates();

		/// @brief update_collision_set at x, logging (and flushing) a named
		///        diagnostic before rethrowing any exception -- the toolkit
		///        throws silently and an escaping exception terminates the
		///        process without unwinding (RB-05).
		void rebuild_collision_set(const Eigen::VectorXd &x, const char *operation);

		virtual double barrier_support_size() const { return dhat_; }

		/// @brief Cap on the trial-step surface displacement handed to CCD,
		///        as a multiple of the barrier support size.
		/// @return Infinity by default, i.e. no cap: the trial interval is
		///         priced as given. Only the semi-implicit barrier, whose
		///         Newton trial steps can be orders larger than any
		///         acceptable step, opts into a finite cap -- see
		///         BarrierContactForm. Returning a finite value here caps
		///         the step even when CCD finds no collision at all, so it
		///         must not be enabled globally.
		virtual double trial_displacement_cap() const
		{
			return std::numeric_limits<double>::infinity();
		}

		/// @brief Collision mesh
		const ipc::CollisionMesh &collision_mesh_;

		/// @brief Barrier activation distance
		const double dhat_;

		/// @brief Minimum distance between elements
		const double dmin_ = 0;

		/// @brief If true, use an adaptive barrier stiffness
		const bool use_adaptive_barrier_stiffness_;
		/// @brief Barrier stiffness
		double barrier_stiffness_;
		/// @brief Maximum barrier stiffness to use when using adaptive barrier stiffness
		double max_barrier_stiffness_;

		/// @brief Average mass of the mesh (used for adaptive barrier stiffness)
		const double avg_mass_;

		/// @brief Is the simulation time dependent?
		const bool is_time_dependent_;

		/// @brief Enable shape derivatives computation
		const bool enable_shape_derivatives_;

		/// @brief Broad phase method to use for distance and CCD evaluations
		const ipc::BroadPhaseMethod broad_phase_method_;
		const std::shared_ptr<ipc::BroadPhase> broad_phase_;
		/// @brief Continuous collision detection specification object
		const ipc::TightInclusionCCD tight_inclusion_ccd_;

		/// @brief Previous minimum distance between all elements
		double prev_distance_;

		/// @brief If true, use the cached candidate set for the current solution
		bool use_cached_candidates_ = false;
		/// @brief Cached candidate set for the current solution
		ipc::Candidates candidates_;
		/// @brief Retained candidate counts of this solve's trial sweeps (RB-04)
		CandidateStatistics candidate_statistics_;
	};
} // namespace polyfem::solver
