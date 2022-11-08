#include "Objective.hpp"
#include <polyfem/utils/MaybeParallelFor.hpp>
#include <polyfem/io/Evaluator.hpp>

using namespace polyfem::utils;

namespace polyfem::solver
{
    Eigen::VectorXd Objective::compute_adjoint_term(const State& state, const Parameter &param)
    {
        Eigen::VectorXd term;
        term.setZero(param.full_dim());

        if (param.contains_state(state))
        {
            assert(state.adjoint_solved());
            AdjointForm::compute_adjoint_term(state, param.name(), term);
        }
        
        return term;
    }

    SpatialIntegralObjective::SpatialIntegralObjective(const State &state, const std::shared_ptr<const ShapeParameter> shape_param, const json &args): state_(state), shape_param_(shape_param)
    {
        spatial_integral_type_ = AdjointForm::SpatialIntegralType::VOLUME;
        auto tmp_ids = args["volume_selection"].get<std::vector<int>>();
        interested_ids_ = std::set(tmp_ids.begin(), tmp_ids.end());
    }

    double SpatialIntegralObjective::value() const
    {
        assert(time_step_ < state_.diff_cached.size());
        return AdjointForm::integrate_objective(state_, get_integral_functional(), state_.diff_cached[time_step_].u, interested_ids_, spatial_integral_type_, time_step_);
    }

    Eigen::VectorXd SpatialIntegralObjective::compute_adjoint_rhs_step(const State& state) const
    {
        if (&state != &state_)
            return Eigen::VectorXd::Zero(state.ndof());
        
        assert(time_step_ < state_.diff_cached.size());
        
        Eigen::VectorXd rhs;
        AdjointForm::dJ_du_step(state, get_integral_functional(), state.diff_cached[time_step_].u, interested_ids_, spatial_integral_type_, time_step_, rhs);

        return rhs;
    }

    Eigen::VectorXd SpatialIntegralObjective::compute_partial_gradient(const Parameter &param) const
    {
        Eigen::VectorXd term;
        term.setZero(param.full_dim());
        if (&param == shape_param_.get())
        {
            assert(time_step_ < state_.diff_cached.size());
            AdjointForm::compute_shape_derivative_functional_term(state_, state_.diff_cached[time_step_].u, get_integral_functional(), interested_ids_, spatial_integral_type_, term, time_step_);
        }
        
        return term;
    }

    StressObjective::StressObjective(const State &state, const std::shared_ptr<const ShapeParameter> shape_param, const std::shared_ptr<const ElasticParameter> &elastic_param, const json &args, bool has_integral_sqrt): SpatialIntegralObjective(state, shape_param, args), elastic_param_(elastic_param)
    {
        formulation_ = state.formulation();
        in_power_ = args["power"];
        out_sqrt_ = has_integral_sqrt;
    }

    IntegrableFunctional StressObjective::get_integral_functional() const
    {
        IntegrableFunctional j;

        j.set_j([this](const Eigen::MatrixXd &local_pts, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &u, const Eigen::MatrixXd &grad_u, const Eigen::MatrixXd &lambda, const Eigen::MatrixXd &mu, const json &params, Eigen::MatrixXd &val) {
            val.setZero(grad_u.rows(), 1);
            Eigen::MatrixXd grad_u_q, stress;
            for (int q = 0; q < grad_u.rows(); q++)
            {
                if (this->formulation_ == "Laplacian")
                {
                    stress = grad_u.row(q);
                }
                else if (this->formulation_ == "LinearElasticity")
                {
                    vector2matrix(grad_u.row(q), grad_u_q);
                    stress = mu(q) * (grad_u_q + grad_u_q.transpose()) + lambda(q) * grad_u_q.trace() * Eigen::MatrixXd::Identity(grad_u_q.rows(), grad_u_q.cols());
                }
                else if (this->formulation_ == "NeoHookean")
                {
                    vector2matrix(grad_u.row(q), grad_u_q);
                    Eigen::MatrixXd def_grad = Eigen::MatrixXd::Identity(grad_u_q.rows(), grad_u_q.cols()) + grad_u_q;
                    Eigen::MatrixXd FmT = def_grad.inverse().transpose();
                    stress = mu(q) * (def_grad - FmT) + lambda(q) * std::log(def_grad.determinant()) * FmT;
                }
                else
                    log_and_throw_error("Unknown formulation!");
                val(q) = pow(stress.squaredNorm(), this->in_power_ / 2.);
            }
        });

        j.set_dj_dgradu([this](const Eigen::MatrixXd &local_pts, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &u, const Eigen::MatrixXd &grad_u, const Eigen::MatrixXd &lambda, const Eigen::MatrixXd &mu, const json &params, Eigen::MatrixXd &val) {
            val.setZero(grad_u.rows(), grad_u.cols());
            const int dim = sqrt(grad_u.cols());
            const int actual_dim = (this->formulation_ == "Laplacian") ? 1 : dim;
            Eigen::MatrixXd grad_u_q, stress, stress_dstress;
            for (int q = 0; q < grad_u.rows(); q++)
            {
                if (this->formulation_ == "Laplacian")
                {
                    stress = grad_u.row(q);
                    stress_dstress = 2 * stress;
                }
                else if (this->formulation_ == "LinearElasticity")
                {
                    vector2matrix(grad_u.row(q), grad_u_q);
                    stress = mu(q) * (grad_u_q + grad_u_q.transpose()) + lambda(q) * grad_u_q.trace() * Eigen::MatrixXd::Identity(grad_u_q.rows(), grad_u_q.cols());
                    stress_dstress = mu(q) * (stress + stress.transpose()) + lambda(q) * stress.trace() * Eigen::MatrixXd::Identity(stress.rows(), stress.cols());
                }
                else if (this->formulation_ == "NeoHookean")
                {
                    vector2matrix(grad_u.row(q), grad_u_q);
                    Eigen::MatrixXd def_grad = Eigen::MatrixXd::Identity(grad_u_q.rows(), grad_u_q.cols()) + grad_u_q;
                    Eigen::MatrixXd FmT = def_grad.inverse().transpose();
                    stress = mu(q) * (def_grad - FmT) + lambda(q) * std::log(def_grad.determinant()) * FmT;
                    stress_dstress = mu(q) * stress + FmT * stress.transpose() * FmT * (mu(q) - lambda(q) * std::log(def_grad.determinant())) + (lambda(q) * (FmT.array() * stress.array()).sum()) * FmT;
                }
                else
                    logger().error("Unknown formulation!");

                const double coef = this->in_power_ * pow(stress.squaredNorm(), this->in_power_ / 2. - 1.);
                for (int i = 0; i < actual_dim; i++)
                    for (int l = 0; l < dim; l++)
                        val(q, i * dim + l) = coef * stress_dstress(i, l);
            }
        });

        return j;
    }

    double StressObjective::value() const
    {
        double val = SpatialIntegralObjective::value();
        if (out_sqrt_)
            return pow(val, 1. / in_power_);
        else
            return val;
    }

    Eigen::VectorXd StressObjective::compute_adjoint_rhs_step(const State& state) const
    {
        Eigen::VectorXd rhs = SpatialIntegralObjective::compute_adjoint_rhs_step(state);

        if (out_sqrt_)
        {
            double val = SpatialIntegralObjective::value();
            if (std::abs(val) < 1e-12)
                logger().warn("stress integral too small, may result in NAN grad!");
            return (pow(val, 1. / in_power_ - 1) / in_power_) * rhs;
        }
        else
            return rhs;
    }

    Eigen::VectorXd StressObjective::compute_partial_gradient(const Parameter &param) const
    {
        Eigen::VectorXd term;
        term.setZero(param.full_dim());
        if (&param == elastic_param_.get())
        {
            // TODO: differentiate stress wrt. lame param
            log_and_throw_error("Not implemented!");
        }
        else if (&param == shape_param_.get())
        {
            term = SpatialIntegralObjective::compute_partial_gradient(param);
        }
        
        if (out_sqrt_)
        {
            double val = SpatialIntegralObjective::value();
            if (std::abs(val) < 1e-12)
                logger().warn("stress integral too small, may result in NAN grad!");
            return (pow(val, 1. / in_power_ - 1) / in_power_) * term;
        }
        else
            return term;
    }

    SumObjective::SumObjective(const json &args)
    {
        if (args.is_array())
        {

        }
        else
        {

        }
    }

    Eigen::MatrixXd SumObjective::compute_adjoint_rhs(const State& state) const
    {
        Eigen::VectorXd rhs;
        rhs.setZero(state.ndof());
        for (const auto &obj : objs)
        {
            rhs += obj.compute_adjoint_rhs(state);
        }
        return rhs;
    }

    Eigen::VectorXd SumObjective::compute_partial_gradient(const Parameter &param) const
    {
        Eigen::VectorXd grad;
        grad.setZero(param.full_dim());
        for (const auto &obj : objs)
        {
            grad += obj.compute_partial_gradient(param);
        }
        return grad;
    }

    double SumObjective::value() const
    {
        double val = 0;
        for (const auto &obj : objs)
        {
            val += obj.value();
        }
        return val;
    }

    void BoundarySmoothingObjective::init(const std::shared_ptr<const ShapeParameter> shape_param)
    {
        shape_param_ = shape_param;

        shape_param_->get_full_mesh(V, F);

        const int dim = V.cols();
        const int n_verts = V.rows();

        Eigen::MatrixXi boundary_edges = shape_param_->get_boundary_edges();
        active_mask = shape_param_->get_active_vertex_mask();
        boundary_nodes = shape_param_->get_boundary_nodes();

        adj.setZero();
        adj.resize(n_verts, n_verts);
        std::vector<Eigen::Triplet<bool>> T_adj;
        for (int e = 0; e < boundary_edges.rows(); e++)
        {
            T_adj.emplace_back(boundary_edges(e, 0), boundary_edges(e, 1), true);
            T_adj.emplace_back(boundary_edges(e, 1), boundary_edges(e, 0), true);
        }
        adj.setFromTriplets(T_adj.begin(), T_adj.end());

        std::vector<int> degrees(n_verts, 0);
        for (int k = 0; k < adj.outerSize(); ++k)
            for (Eigen::SparseMatrix<bool, Eigen::RowMajor>::InnerIterator it(adj, k); it; ++it)
                degrees[k]++;
                
        L.setZero();
        L.resize(n_verts, n_verts);
        if (!args_["scale_invariant"])
        {
            std::vector<Eigen::Triplet<double>> T_L;
            for (int k = 0; k < adj.outerSize(); ++k)
            {
                if (degrees[k] == 0 || !active_mask[k])
                    continue;
                T_L.emplace_back(k, k, degrees[k]);
                for (Eigen::SparseMatrix<bool, Eigen::RowMajor>::InnerIterator it(adj, k); it; ++it)
                {
                    assert(it.row() == k);
                    T_L.emplace_back(it.row(), it.col(), -1);
                }
            }
            L.setFromTriplets(T_L.begin(), T_L.end());
            L.prune([](int i, int j, double val) { return abs(val) > 1e-12; });
        }
    }

    BoundarySmoothingObjective::BoundarySmoothingObjective(const std::shared_ptr<const ShapeParameter> shape_param, const json &args): args_(args)
    {
        init(shape_param);
    }

    double BoundarySmoothingObjective::value() const
    {
        const int dim = V.cols();
        const int n_verts = V.rows();
        const int power = args_["power"];

        double val = 0;
        if (args_["scale_invariant"])
        {
			for (int b : boundary_nodes)
			{
				if (!active_mask[b])
					continue;
				polyfem::RowVectorNd s;
				s.setZero(V.cols());
				double sum_norm = 0;
				for (Eigen::SparseMatrix<bool, Eigen::RowMajor>::InnerIterator it(adj, b); it; ++it)
				{
					assert(it.col() != b);
					s += V.row(b) - V.row(it.col());
					sum_norm += (V.row(b) - V.row(it.col())).norm();
				}
				s = s / sum_norm;
				val += pow(s.norm(), power);
			}
        }
        else
            val = (L * V).eval().squaredNorm();

        return val;
    }

    Eigen::MatrixXd BoundarySmoothingObjective::compute_adjoint_rhs(const State& state) const
    {
        return Eigen::VectorXd::Zero(state.ndof());
    }

    Eigen::VectorXd BoundarySmoothingObjective::compute_partial_gradient(const Parameter &param) const
    {
        const int dim = V.cols();
        const int n_verts = V.rows();
        const int power = args_["power"];

        if (args_["scale_invariant"])
        {
            Eigen::VectorXd grad;
			grad.setZero(V.size());
			for (int b : boundary_nodes)
			{
				if (!active_mask[b])
					continue;
				polyfem::RowVectorNd s;
				s.setZero(dim);
				double sum_norm = 0;
				auto sum_normalized = s;
				int valence = 0;
				for (Eigen::SparseMatrix<bool, Eigen::RowMajor>::InnerIterator it(adj, b); it; ++it)
				{
					assert(it.col() != b);
					auto x = V.row(b) - V.row(it.col());
					s += x;
					sum_norm += x.norm();
					sum_normalized += x.normalized();
					valence += 1;
				}
				s = s / sum_norm;

				for (int d = 0; d < dim; d++)
				{
					grad(b * dim + d) += (s(d) * valence - s.squaredNorm() * sum_normalized(d)) * power * pow(s.norm(), power - 2.) / sum_norm;
				}

				for (Eigen::SparseMatrix<bool, Eigen::RowMajor>::InnerIterator it(adj, b); it; ++it)
				{
					for (int d = 0; d < dim; d++)
					{
						grad(it.col() * dim + d) -= (s(d) + s.squaredNorm() * (V(it.col(), d) - V(b, d)) / (V.row(b) - V.row(it.col())).norm()) * power * pow(s.norm(), power - 2.) / sum_norm;
					}
				}
			}
            return grad;
        }
        else
            return 2 * (L.transpose() * (L * V));
    }

    VolumeObjective::VolumeObjective(const std::shared_ptr<const ShapeParameter> shape_param, const json &args): shape_param_(shape_param)
    {
        if (!shape_param)
            log_and_throw_error("Volume Objective needs non-empty shape parameter!");
        auto tmp_ids = args["volume_selection"].get<std::vector<int>>();
        interested_ids_ = std::set(tmp_ids.begin(), tmp_ids.end());
    }

    double VolumeObjective::value() const
    {
		IntegrableFunctional j;
		j.set_j([](const Eigen::MatrixXd &local_pts, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &u, const Eigen::MatrixXd &grad_u, const Eigen::MatrixXd &lambda, const Eigen::MatrixXd &mu, const json &params, Eigen::MatrixXd &val) {
			val.setOnes(u.rows(), 1);
		});

        const State &state = shape_param_->get_state();

        return AdjointForm::integrate_objective(state, j, Eigen::MatrixXd::Zero(state.ndof(), 1), interested_ids_, AdjointForm::SpatialIntegralType::VOLUME, 0);
    }

    Eigen::MatrixXd VolumeObjective::compute_adjoint_rhs(const State& state) const
    {
        return Eigen::VectorXd::Zero(state.ndof()); // Important: it's state, not state_
    }

    Eigen::VectorXd VolumeObjective::compute_partial_gradient(const Parameter &param) const
    {
        if (&param == shape_param_.get())
        {
            IntegrableFunctional j;
            j.set_j([](const Eigen::MatrixXd &local_pts, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &u, const Eigen::MatrixXd &grad_u, const Eigen::MatrixXd &lambda, const Eigen::MatrixXd &mu, const json &params, Eigen::MatrixXd &val) {
                val.setOnes(u.rows(), 1);
            });

            const State &state = shape_param_->get_state();
            Eigen::VectorXd term;
            AdjointForm::compute_shape_derivative_functional_term(state, Eigen::MatrixXd::Zero(state.ndof(), 1), j, interested_ids_, AdjointForm::SpatialIntegralType::VOLUME, term, 0);
            return term;
        }
        else
            return Eigen::VectorXd::Zero(param.full_dim());
    }

    VolumePaneltyObjective::VolumePaneltyObjective(const std::shared_ptr<const ShapeParameter> shape_param, const json &args)
    {
        if (args["soft_bound"].get<std::vector<double>>().size() == 2)
            bound = args["soft_bound"];
        else
            bound << 0, std::numeric_limits<double>::max();

        obj = std::make_shared<VolumeObjective>(shape_param, args);
    }

    double VolumePaneltyObjective::value() const
    {
        double vol = obj->value();

        if (vol < bound[0])
            return pow(vol - bound[0], 2);
        else if (vol > bound[1])
            return pow(vol - bound[1], 2);
        else
            return 0;
    }
    Eigen::MatrixXd VolumePaneltyObjective::compute_adjoint_rhs(const State& state) const
    {
        return Eigen::MatrixXd::Zero(state.diff_cached[0].u.size(), 1);
    }
    Eigen::VectorXd VolumePaneltyObjective::compute_partial_gradient(const Parameter &param) const
    {
        double vol = obj->value();
        Eigen::VectorXd grad = obj->compute_partial_gradient(param);

        if (vol < bound[0])
            return (2 * (vol - bound[0])) * grad;
        else if (vol > bound[1])
            return (2 * (vol - bound[1])) * grad;
        else
            return Eigen::VectorXd::Zero(grad.size(), 1);
    }

    PositionObjective::PositionObjective(const State &state, const std::shared_ptr<const ShapeParameter> shape_param, const json &args): SpatialIntegralObjective(state, shape_param, args)
    {
    }

    IntegrableFunctional PositionObjective::get_integral_functional() const
    {
		IntegrableFunctional j;

		j.set_j([this](const Eigen::MatrixXd &local_pts, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &u, const Eigen::MatrixXd &grad_u, const Eigen::MatrixXd &lambda, const Eigen::MatrixXd &mu, const json &params, Eigen::MatrixXd &val) {
			val = u.col(this->dim_) + pts.col(this->dim_);
		});

		j.set_dj_du([this](const Eigen::MatrixXd &local_pts, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &u, const Eigen::MatrixXd &grad_u, const Eigen::MatrixXd &lambda, const Eigen::MatrixXd &mu, const json &params, Eigen::MatrixXd &val) {
			val.setZero(u.rows(), u.cols());
			val.col(this->dim_).setOnes();
		});

		j.set_dj_dx([this](const Eigen::MatrixXd &local_pts, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &u, const Eigen::MatrixXd &grad_u, const Eigen::MatrixXd &lambda, const Eigen::MatrixXd &mu, const json &params, Eigen::MatrixXd &val) {
			val.setZero(pts.rows(), pts.cols());
			val.col(this->dim_).setOnes();
		});

        return j;
    }

    Eigen::MatrixXd StaticObjective::compute_adjoint_rhs(const State& state) const
    {
        Eigen::MatrixXd term(state.ndof(), state.diff_cached.size());
        term.col(time_step_) = compute_adjoint_rhs_step(state);

        return term;
    }

    BarycenterTargetObjective::BarycenterTargetObjective(const State &state, const std::shared_ptr<const ShapeParameter> shape_param, const json &args, const Eigen::MatrixXd &target)
    {
        dim_ = state.mesh->dimension();
        target_ = target;

        objv = std::make_shared<VolumeObjective>(shape_param, args);
        objp.resize(dim_);
        for (int d = 0; d < dim_; d++)
        {
            objp[d] = std::make_shared<PositionObjective>(state, shape_param, args);
            objp[d]->set_dim(d);
        }
    }
    
    Eigen::VectorXd BarycenterTargetObjective::get_target() const
    {
        assert(target_.cols() == dim_);
        if (target_.rows() > 1)
            return target_.row(get_time_step());
        else
            return target_;
    }

    void BarycenterTargetObjective::set_time_step(int time_step)
    {
        StaticObjective::set_time_step(time_step);
        for (auto &obj : objp)
            obj->set_time_step(time_step);
    }

    double BarycenterTargetObjective::value() const
    {
        return (get_barycenter() - get_target()).squaredNorm();
    }
    Eigen::VectorXd BarycenterTargetObjective::compute_partial_gradient(const Parameter &param) const
    {
        Eigen::VectorXd term;
        term.setZero(param.full_dim());

        Eigen::VectorXd target = get_target();
        
        const double volume = objv->value();
        Eigen::VectorXd center(dim_);
        for (int d = 0; d < dim_; d++)
            center(d) = objp[d]->value() / volume;

        double coeffv = 0;
        for (int d = 0; d < dim_; d++)
            coeffv += 2 * (center(d) - target(d)) * (-center(d) / volume);
        
        term += coeffv * objv->compute_partial_gradient(param);

        for (int d = 0; d < dim_; d++)
            term += (2.0 / volume * (center(d) - target(d))) * objp[d]->compute_partial_gradient(param);
        
        return term;
    }
    Eigen::VectorXd BarycenterTargetObjective::compute_adjoint_rhs_step(const State& state) const
    {
        Eigen::VectorXd term;
        term.setZero(state.ndof());

        Eigen::VectorXd target = get_target();
        
        const double volume = objv->value();
        Eigen::VectorXd center(dim_);
        for (int d = 0; d < dim_; d++)
            center(d) = objp[d]->value() / volume;

        for (int d = 0; d < dim_; d++)
            term += (2.0 / volume * (center(d) - target(d))) * objp[d]->compute_adjoint_rhs_step(state);
        
        return term;
    }

    Eigen::VectorXd BarycenterTargetObjective::get_barycenter() const
    {
        const double volume = objv->value();
        Eigen::VectorXd center(dim_);
        for (int d = 0; d < dim_; d++)
            center(d) = objp[d]->value() / volume;

        return center;
    }

    TransientObjective::TransientObjective(const int time_steps, const double dt, const std::string &transient_integral_type, const std::shared_ptr<StaticObjective> &obj)
    {
        time_steps_ = time_steps;
        dt_ = dt;
        transient_integral_type_ = transient_integral_type;
        obj_ = obj;
    }

    std::vector<double> TransientObjective::get_transient_quadrature_weights() const
    {
        std::vector<double> weights;
        weights.assign(time_steps_ + 1, dt_);
        if (transient_integral_type_ == "uniform")
        {
            weights[0] = 0;
        }
        else if (transient_integral_type_ == "trapezoidal")
        {
            weights[0] = dt_ / 2.;
            weights[weights.size() - 1] = dt_ / 2.;
        }
        else if (transient_integral_type_ == "simpson")
        {
            weights[0] = dt_ / 3.;
            weights[weights.size() - 1] = dt_ / 3.;
            for (int i = 1; i < weights.size() - 1; i++)
            {
                if (i % 2)
                    weights[i] = dt_ * 4. / 3.;
                else
                    weights[i] = dt_ * 2. / 4.;
            }
        }
        else if (transient_integral_type_ == "final")
        {
            weights.assign(time_steps_ + 1, 0);
            weights[time_steps_] = 1;
        }
        else if (transient_integral_type_.find("step_") != std::string::npos)
        {
            weights.assign(time_steps_ + 1, 0);
            int step = std::stoi(transient_integral_type_.substr(5));
            assert(step > 0 && step < weights.size());
            weights[step] = 1;
        }
        else
            assert(false);

        return weights;
    }

    double TransientObjective::value() const
    {
        double value = 0;
		std::vector<double> weights = get_transient_quadrature_weights();
		for (int i = 0; i <= time_steps_; i++)
		{
            obj_->set_time_step(i);
            value += weights[i] * obj_->value();
		}
        return value;
    }

    Eigen::MatrixXd TransientObjective::compute_adjoint_rhs(const State& state) const
    {
        Eigen::MatrixXd terms;
		terms.setZero(state.ndof(), time_steps_ + 1);

		std::vector<double> weights = get_transient_quadrature_weights();
		for (int i = 0; i <= time_steps_; i++)
		{
            obj_->set_time_step(i);
            terms.col(i) = weights[i] * obj_->compute_adjoint_rhs_step(state);
		}

        return terms;
    }

    Eigen::VectorXd TransientObjective::compute_partial_gradient(const Parameter &param) const
    {
        Eigen::VectorXd term;
        term.setZero(param.full_dim());

        std::vector<double> weights = get_transient_quadrature_weights();
        for (int i = 0; i <= time_steps_; i++)
        {
            obj_->set_time_step(i);
            term += weights[i] * obj_->compute_partial_gradient(param);
        }

        return term;
    }

    ComplianceObjective::ComplianceObjective(const State &state, const std::shared_ptr<const ShapeParameter> shape_param, const std::shared_ptr<const ElasticParameter> &elastic_param, const std::shared_ptr<const TopologyOptimizationParameter> topo_param, const json &args): SpatialIntegralObjective(state, shape_param, args), elastic_param_(elastic_param), topo_param_(topo_param)
    {
        formulation_ = state.formulation();
    }

    IntegrableFunctional ComplianceObjective::get_integral_functional() const
    {
        IntegrableFunctional j;

		j.set_j([this](const Eigen::MatrixXd &local_pts, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &u, const Eigen::MatrixXd &grad_u, const Eigen::MatrixXd &lambda, const Eigen::MatrixXd &mu, const json &params, Eigen::MatrixXd &val) {
			val.setZero(grad_u.rows(), 1);
			for (int q = 0; q < grad_u.rows(); q++)
			{
				Eigen::MatrixXd grad_u_q, stress;
				vector2matrix(grad_u.row(q), grad_u_q);
				if (this->formulation_ == "LinearElasticity")
				{
					stress = mu(q) * (grad_u_q + grad_u_q.transpose()) + lambda(q) * grad_u_q.trace() * Eigen::MatrixXd::Identity(grad_u_q.rows(), grad_u_q.cols());
				}
				else
					logger().error("Unknown formulation!");
				val(q) = (stress.array() * grad_u_q.array()).sum();
			}
		});

		j.set_dj_dgradu([this](const Eigen::MatrixXd &local_pts, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &u, const Eigen::MatrixXd &grad_u, const Eigen::MatrixXd &lambda, const Eigen::MatrixXd &mu, const json &params, Eigen::MatrixXd &val) {
			val.setZero(grad_u.rows(), grad_u.cols());
			const int dim = sqrt(grad_u.cols());
			for (int q = 0; q < grad_u.rows(); q++)
			{
				Eigen::MatrixXd grad_u_q, stress;
				vector2matrix(grad_u.row(q), grad_u_q);
				if (this->formulation_ == "LinearElasticity")
				{
					stress = mu(q) * (grad_u_q + grad_u_q.transpose()) + lambda(q) * grad_u_q.trace() * Eigen::MatrixXd::Identity(grad_u_q.rows(), grad_u_q.cols());
				}
				else
					logger().error("Unknown formulation!");

				for (int i = 0; i < dim; i++)
					for (int l = 0; l < dim; l++)
						val(q, i * dim + l) = 2 * stress(i, l);
			}
		});

        return j;
    }

    Eigen::VectorXd ComplianceObjective::compute_partial_gradient(const Parameter &param) const
    {
        Eigen::VectorXd term;
        term.setZero(param.full_dim());
        if (&param == elastic_param_.get())
        {
            // TODO: differentiate wrt. lame param
            log_and_throw_error("Not implemented!");
        }
        else if (&param == shape_param_.get())
            term = compute_partial_gradient(param);
        else if (&param == topo_param_.get())
        {
            const auto &bases = state_.bases;
            const auto &gbases = state_.geom_bases();
            const int dim = state_.mesh->dimension();
            
			const LameParameters &params = state_.assembler.lame_params();
			const auto &density_mat = params.density_mat_;
			const double density_power = params.density_power_;
			for (int e = 0; e < bases.size(); e++)
			{
				assembler::ElementAssemblyValues vals;
				state_.ass_vals_cache.compute(e, state_.mesh->is_volume(), bases[e], gbases[e], vals);

				const quadrature::Quadrature &quadrature = vals.quadrature;

				for (int q = 0; q < quadrature.weights.size(); q++)
				{
					double lambda, mu;
					params.lambda_mu(quadrature.points.row(q), vals.val.row(q), e, lambda, mu, false);

					Eigen::MatrixXd grad_u_q(dim, dim);
					grad_u_q.setZero();
					for (const auto &v : vals.basis_values)
						for (int d = 0; d < dim; d++)
						{
							double coeff = 0;
							for (const auto &g : v.global)
								coeff += state_.diff_cached[time_step_].u(g.index * dim + d) * g.val;
							grad_u_q.row(d) += v.grad_t_m.row(q) * coeff;
						}

					Eigen::MatrixXd stress;
					if (state_.formulation() == "LinearElasticity")
						stress = mu * (grad_u_q + grad_u_q.transpose()) + lambda * grad_u_q.trace() * Eigen::MatrixXd::Identity(grad_u_q.rows(), grad_u_q.cols());
					else
						logger().error("Unknown formulation!");

					term(e) += density_power * pow(density_mat(e), density_power - 1) * (stress.array() * grad_u_q.array()).sum() * quadrature.weights(q) * vals.det(q);
				}
			}
        }
        
        return term;
    }

    IntegrableFunctional TargetObjective::get_integral_functional() const
    {
        assert (target_state_);
        assert (target_state_->diff_cached.size() > 0);

        IntegrableFunctional j;

        auto j_func = [this](const Eigen::MatrixXd &local_pts, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &u, const Eigen::MatrixXd &grad_u, const Eigen::MatrixXd &lambda, const Eigen::MatrixXd &mu, const json &params, Eigen::MatrixXd &val) {
            val.setZero(u.rows(), 1);
            const int e = params["elem"];
            int e_ref;
            if (auto search = e_to_ref_e_.find(e); search != e_to_ref_e_.end())
                e_ref = search->second;
            else
                e_ref = e;
            const auto &gbase_ref = target_state_->geom_bases()[e_ref];

            Eigen::MatrixXd pts_ref;
            gbase_ref.eval_geom_mapping(local_pts, pts_ref);

            Eigen::MatrixXd u_ref, grad_u_ref;
            const Eigen::MatrixXd &sol_ref = target_state_->problem->is_time_dependent() ? target_state_->diff_cached[params["step"].get<int>()].u : target_state_->diff_cached[0].u;
            io::Evaluator::interpolate_at_local_vals(*(target_state_->mesh), target_state_->problem->is_scalar(), target_state_->bases, target_state_->geom_bases(), e_ref, local_pts, sol_ref, u_ref, grad_u_ref);

            for (int q = 0; q < u.rows(); q++)
            {
                val(q) = ((u_ref.row(q) + pts_ref.row(q)) - (u.row(q) + pts.row(q))).squaredNorm();
            }
        };

        auto djdu_func = [this](const Eigen::MatrixXd &local_pts, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &u, const Eigen::MatrixXd &grad_u, const Eigen::MatrixXd &lambda, const Eigen::MatrixXd &mu, const json &params, Eigen::MatrixXd &val) {
            val.setZero(u.rows(), u.cols());
            const int e = params["elem"];
            int e_ref;
            if (auto search = e_to_ref_e_.find(e); search != e_to_ref_e_.end())
                e_ref = search->second;
            else
                e_ref = e;
            const auto &gbase_ref = target_state_->geom_bases()[e_ref];

            Eigen::MatrixXd pts_ref;
            gbase_ref.eval_geom_mapping(local_pts, pts_ref);

            Eigen::MatrixXd u_ref, grad_u_ref;
            const Eigen::MatrixXd &sol_ref = target_state_->problem->is_time_dependent() ? target_state_->diff_cached[params["step"].get<int>()].u : target_state_->diff_cached[0].u;
            io::Evaluator::interpolate_at_local_vals(*(target_state_->mesh), target_state_->problem->is_scalar(), target_state_->bases, target_state_->geom_bases(), e_ref, local_pts, sol_ref, u_ref, grad_u_ref);

            for (int q = 0; q < u.rows(); q++)
            {
                auto x = (u.row(q) + pts.row(q)) - (u_ref.row(q) + pts_ref.row(q));
                val.row(q) = 2 * x;
            }
        };

        j.set_j(j_func);
        j.set_dj_du(djdu_func);
        j.set_dj_dx(djdu_func); // only used for shape derivative

        return j;
    }

    void TargetObjective::set_reference(const std::shared_ptr<const State> &target_state, const std::set<int> &reference_cached_body_ids)
	{
		target_state_ = target_state;

		std::map<int, std::vector<int>> ref_interested_body_id_to_e;
		int ref_count = 0;
		for (int e = 0; e < target_state_->bases.size(); ++e)
		{
			int body_id = target_state_->mesh->get_body_id(e);
			if (reference_cached_body_ids.size() > 0 && reference_cached_body_ids.count(body_id) == 0)
				continue;
			if (ref_interested_body_id_to_e.find(body_id) != ref_interested_body_id_to_e.end())
				ref_interested_body_id_to_e[body_id].push_back(e);
			else
				ref_interested_body_id_to_e[body_id] = {e};
			ref_count++;
		}

		std::map<int, std::vector<int>> interested_body_id_to_e;
		int count = 0;
		for (int e = 0; e < state_.bases.size(); ++e)
		{
			int body_id = state_.mesh->get_body_id(e);
			if (reference_cached_body_ids.size() > 0 && reference_cached_body_ids.count(body_id) == 0)
				continue;
			if (interested_body_id_to_e.find(body_id) != interested_body_id_to_e.end())
				interested_body_id_to_e[body_id].push_back(e);
			else
				interested_body_id_to_e[body_id] = {e};
			count++;
		}

		if (count != ref_count)
			logger().error("Number of interested elements in the reference and optimization examples do not match!");
		else
			logger().trace("Found {} matching elements.", count);

		for (const auto &kv : interested_body_id_to_e)
		{
			for (int i = 0; i < kv.second.size(); ++i)
			{
				e_to_ref_e_[kv.second[i]] = ref_interested_body_id_to_e[kv.first][i];
			}
		}
	}
}