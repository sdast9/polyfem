#include "FrictionForm.hpp"
#include "FrictionCheckpoint.hpp"
#include <polyfem/io/ForkCheckpoint.hpp>
#include "BarrierContactForm.hpp"
#include "GCPContactForm.hpp"
#include "ESPContactForm.hpp"

#include <polyfem/io/MatrixIO.hpp>
#include <polyfem/utils/Logger.hpp>
#include <polyfem/utils/Timer.hpp>
#include <polyfem/utils/MatrixUtils.hpp>

#include <ipc/broad_phase/create_broad_phase.hpp>
#include <ipc/collisions/normal/normal_collisions.hpp>
#include <ipc/gcp/gcp_collisions.hpp>

#include <Eigen/Core>

#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

namespace polyfem::solver
{
	FrictionForm::FrictionForm(
		const ipc::CollisionMesh &collision_mesh,
		const std::shared_ptr<time_integrator::ImplicitTimeIntegrator> time_integrator,
		const double epsv,
		const double mu,
		const ipc::BroadPhaseMethod broad_phase_method,
		const ContactForm &contact_form,
		const int n_lagging_iters)
		: collision_mesh_(collision_mesh),
		  time_integrator_(time_integrator),
		  epsv_(epsv),
		  mu_(mu),
		  broad_phase_method_(broad_phase_method),
		  n_lagging_iters_(n_lagging_iters < 0 ? std::numeric_limits<int>::max() : n_lagging_iters),
		  contact_form_(contact_form),
		  friction_potential_(epsv)
	{
		assert(epsv_ > 0);
	}

	Eigen::MatrixXd FrictionForm::compute_displaced_surface(const Eigen::VectorXd &x) const
	{
		return contact_form_.compute_displaced_surface(x);
	}

	Eigen::MatrixXd FrictionForm::compute_surface_velocities(const Eigen::VectorXd &x) const
	{
		// In the case of a static problem, the velocity is the displacement
		const Eigen::VectorXd v = time_integrator_ != nullptr ? time_integrator_->compute_velocity(x) : x;
		return collision_mesh_.map_displacements(utils::unflatten(v, collision_mesh_.dim()));
	}

	double FrictionForm::dv_dx() const
	{
		return time_integrator_ != nullptr ? time_integrator_->dv_dx() : 1;
	}

	bool FrictionForm::realized_lag() const
	{
		const auto barrier_contact = dynamic_cast<const BarrierContactForm *>(&contact_form_);
		return barrier_contact != nullptr && barrier_contact->uses_semi_implicit_stiffness() && barrier_contact->friction_lag_realized();
	}

	void FrictionForm::init_lagging(const Eigen::VectorXd &x)
	{
		// Realized mode: the lag built by update_quantities at these same
		// coordinates (before the between-steps refresh moved the trim) is
		// the one that carries the force that acted there; keep it.
		if (realized_lag() && lag_x_.size() == x.size() && lag_x_ == x)
			return;
		update_lagging(x, 0);
	}

	void FrictionForm::update_quantities(const double t, const Eigen::VectorXd &x)
	{
		if (realized_lag())
			update_lagging(x, 0);
	}

	double FrictionForm::trim_scale() const
	{
		const auto barrier_contact = dynamic_cast<const BarrierContactForm *>(&contact_form_);
		if (barrier_contact == nullptr || !barrier_contact->uses_semi_implicit_stiffness())
			return 1;
		if (barrier_contact->friction_lag_realized())
			return 1;
		if (!(lagged_trim_ > 0) || !std::isfinite(lagged_trim_))
			return 1;
		return contact_form_.barrier_stiffness() / lagged_trim_;
	}

	double FrictionForm::value_unweighted(const Eigen::VectorXd &x) const
	{
		return trim_scale() * friction_potential_(friction_collision_set_, collision_mesh_, compute_surface_velocities(x)) / dv_dx();
	}

	void FrictionForm::first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &gradv) const
	{
		const Eigen::VectorXd grad_friction = friction_potential_.gradient(
			friction_collision_set_, collision_mesh_, compute_surface_velocities(x));
		gradv = trim_scale() * collision_mesh_.to_full_dof(grad_friction);
	}

	void FrictionForm::second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &hessian) const
	{
		POLYFEM_SCOPED_TIMER("friction hessian");

		ipc::PSDProjectionMethod psd_projection_method;

		if (project_to_psd_)
		{
			psd_projection_method = ipc::PSDProjectionMethod::CLAMP;
		}
		else
		{
			psd_projection_method = ipc::PSDProjectionMethod::NONE;
		}

		hessian = (trim_scale() * dv_dx()) * friction_potential_.hessian( //
					  friction_collision_set_, collision_mesh_, compute_surface_velocities(x), psd_projection_method);

		hessian = collision_mesh_.to_full_dof(hessian);
	}

	std::unique_ptr<FormState> FrictionForm::save_state() const
	{
		auto state = std::make_unique<State>();
		save_base_state(*state);
		state->friction_collision_set = friction_collision_set_;
		state->lagged_trim = lagged_trim_;
		state->lag_x = lag_x_;
		return state;
	}

	void FrictionForm::restore_state(const FormState &state, const Eigen::VectorXd &)
	{
		const State &friction_state = state_as<State>(state, "FrictionForm");
		restore_base_state(friction_state);
		friction_collision_set_ = friction_state.friction_collision_set;
		lagged_trim_ = friction_state.lagged_trim;
		lag_x_ = friction_state.lag_x;
	}

	void FrictionForm::update_lagging(const Eigen::VectorXd &x, const int iter_num)
	{
		lag_x_ = x;
		const Eigen::MatrixXd displaced_surface = compute_displaced_surface(x);

		auto broad_phase = ipc::create_broad_phase(broad_phase_method_);
		if (const auto barrier_contact = dynamic_cast<const BarrierContactForm *>(&contact_form_))
		{
			ipc::NormalCollisions collision_set;
			collision_set.set_use_area_weighting(barrier_contact->use_area_weighting());
			// Copy the collision set type directly rather than going through
			// use_improved_max_operator(), which collapses it to a bool and so
			// would silently downgrade OGC to IPC here.
			collision_set.set_collision_set_type(barrier_contact->collision_set().collision_set_type());
			collision_set.set_skip_obstacles(barrier_contact->collision_set().skip_obstacles());
			collision_set.set_separate_signed_contributions(barrier_contact->collision_set().separate_signed_contributions());

			collision_set.set_enable_shape_derivatives(barrier_contact->enable_shape_derivatives());
			collision_set.build(
				collision_mesh_, displaced_surface, barrier_contact->dhat(), /*dmin=*/0, broad_phase.get());

			// Per-contact stiffness scales so the lagged friction normal
			// forces see trim * kappa_i instead of a single global stiffness.
			if (barrier_contact->uses_semi_implicit_stiffness())
				barrier_contact->assign_collision_stiffness(collision_set);

			ipc::BarrierPotential bp = barrier_contact->barrier_potential();
			bp.set_stiffness(barrier_contact->barrier_stiffness());
			lagged_trim_ = barrier_contact->barrier_stiffness();
			friction_collision_set_.build(
				collision_mesh_, displaced_surface, collision_set,
				bp, Eigen::VectorXd::Ones(collision_mesh_.num_vertices()) * mu_, Eigen::VectorXd::Ones(collision_mesh_.num_vertices()) * mu_);
		}
		else if (const auto gcp = dynamic_cast<const GCPContactForm *>(&contact_form_))
		{
			ipc::GCPCollisions collision_set;
			if (gcp->using_adaptive_dhat())
				collision_set.compute_adaptive_dhat(collision_mesh_, collision_mesh_.rest_positions(), gcp->get_params(), broad_phase.get());
			collision_set.build(
				collision_mesh_, displaced_surface, gcp->get_params(),
				gcp->using_adaptive_dhat(), broad_phase.get());

			friction_collision_set_.build(
				collision_mesh_, displaced_surface,
				collision_set, gcp->get_params(), contact_form_.barrier_stiffness(), Eigen::VectorXd::Ones(collision_mesh_.num_vertices()) * mu_, Eigen::VectorXd::Ones(collision_mesh_.num_vertices()) * mu_);
		}
		else if (const auto esp_contact = dynamic_cast<const ESPContactForm *>(&contact_form_))
		{
			ipc::ESPCollisions collision_set;
			collision_set.build(
				collision_mesh_, displaced_surface, esp_contact->get_params(),
				broad_phase.get());

			friction_collision_set_.build(
				collision_mesh_, displaced_surface, collision_set,
				esp_contact->get_params(), contact_form_.barrier_stiffness(),
				Eigen::VectorXd::Ones(collision_mesh_.num_vertices()) * mu_,
				Eigen::VectorXd::Ones(collision_mesh_.num_vertices()) * mu_,
				ipc::TangentialCollisions::default_blend_mu, esp_contact->barrier_potential().use_near_far());
		}
		else
		{
			throw std::runtime_error("Unknown contact form");
		}

		// The lagged normal forces and sliding bases are what the friction
		// potential is built from: rebuilding them is a new function. This
		// happens between minimizations, in the lagging loop.
		note_objective_change("friction lag update");
	}

	void FrictionForm::serialize_checkpoint(io::CheckpointWriter &writer, const std::string &group) const
	{
		if (mu_ != 0 && dynamic_cast<const ESPContactForm *>(&contact_form_))
			log_and_throw_error("Checkpoint continuation with ESP friction is not yet supported: quadrature contributions require additional history identities. Disable checkpoint output or use a supported contact formulation.");
		Eigen::MatrixXd scalars(1, 2);
		scalars << lagged_trim_, double(friction_collision_set_.size());
		writer.write_matrix(group + "/friction_scalars", scalars);
		writer.write_attribute(group, "layout_version", long(1));
		if (friction_collision_set_.size() == 0)
			return;
		Eigen::MatrixXd forces(friction_collision_set_.size(), 1);
		for (size_t i = 0; i < friction_collision_set_.size(); ++i)
			forces(i) = friction_collision_set_[i].normal_force_magnitude;
		writer.write_matrix(group + "/friction_normal_force", forces);
		const auto identities = friction_checkpoint::keys(friction_collision_set_);
		std::vector<long> ids;
		Eigen::MatrixXd planes(identities.size(), 4);
		for (size_t i = 0; i < identities.size(); ++i)
		{
			ids.insert(ids.end(), identities[i].ids.begin(), identities[i].ids.end());
			for (int j = 0; j < 4; ++j)
				planes(i, j) = identities[i].plane[j];
		}
		writer.write_long_vector(group + "/friction_identity", ids);
		writer.write_matrix(group + "/friction_planes", planes);
	}

	bool FrictionForm::deserialize_checkpoint(const io::CheckpointReader &reader, const std::string &group, const Eigen::VectorXd &x)
	{
		if (mu_ != 0 && dynamic_cast<const ESPContactForm *>(&contact_form_))
			log_and_throw_error("Checkpoint continuation with ESP friction is not yet supported: quadrature contributions require additional history identities. Disable checkpoint output or use a supported contact formulation.");
		if (!io::has_fork_checkpoint_group(reader, group))
			return false;
		const std::string path = reader.path().string();
		Eigen::MatrixXd scalars;
		if (!io::read_fork_checkpoint_matrix(reader, group, "friction_scalars", scalars))
			return false;
		if (scalars.rows() != 1 || scalars.cols() != 2)
			log_and_throw_error("Restart state {}: friction_scalars has {} entries, expected 2", path, scalars.size());
		if (!std::isfinite(scalars(0)))
			log_and_throw_error("Checkpoint {}: friction lagged trim must be finite", path);
		const size_t count = io::checked_checkpoint_integer<size_t>(scalars(1), "friction collision count");
		if (count != friction_collision_set_.size())
			log_and_throw_error("Checkpoint {}: friction collision count mismatch (saved {}, rebuilt {})", path, count, friction_collision_set_.size());
		ipc::TangentialCollisions restored = friction_collision_set_;
		if (count == 0)
		{
			for (const auto *name : {"friction_normal_force", "friction_identity", "friction_planes"})
				if (reader.exists(group + "/" + name))
					log_and_throw_error("Checkpoint {}: unexpected {} for empty friction history", path, name);
		}
		else
		{
			Eigen::MatrixXd forces, planes;
			io::read_fork_checkpoint_matrix(reader, group, "friction_normal_force", forces);
			io::read_fork_checkpoint_matrix(reader, group, "friction_planes", planes);
			if (size_t(forces.rows()) != count || forces.cols() != 1 || !forces.allFinite()
				|| size_t(planes.rows()) != count || planes.cols() != 4)
				log_and_throw_error("Checkpoint {}: invalid friction force/plane payload shape or values", path);
			const auto ids = reader.read_long_vector(group + "/friction_identity");
			if (ids.size() / 4 != count || ids.size() % 4 != 0)
				log_and_throw_error("Checkpoint {}: invalid friction identity payload size", path);
			std::vector<friction_checkpoint::Key> saved(count);
			for (size_t i = 0; i < count; ++i)
				for (int j = 0; j < 4; ++j)
				{
					saved[i].ids[j] = ids[4 * i + j];
					saved[i].plane[j] = planes(i, j);
				}
			const auto order = friction_checkpoint::match(saved, friction_checkpoint::keys(friction_collision_set_));
			restored = friction_checkpoint::reordered(friction_collision_set_, order);
			for (size_t i = 0; i < count; ++i)
				restored[i].normal_force_magnitude = forces(i);
		}
		friction_collision_set_ = std::move(restored);
		lagged_trim_ = scalars(0);
		lag_x_ = x;
		note_objective_change("restart state restored");
		return true;
	}
} // namespace polyfem::solver
