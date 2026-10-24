#include "GrowthNeoHookean.hpp"

namespace polyfem::assembler
{
	void GrowthNeoHookean::add_multimaterial(int index, const json &params, const Units &units, const std::string &root_path)
	{
		if (size() != 3)
			log_and_throw_error("GrowthNeoHookean requires a 3D volume mesh");
		params_.add_multimaterial(index, params, true, units.stress(), root_path);
		json p = params;
		for (const auto *key : {"g11", "g22", "g33"})
			if (!p.contains(key))
				p[key] = 1.0;
		for (const auto *key : {"g12", "g13", "g23"})
			if (!p.contains(key))
				p[key] = 0.0;
		for (auto *g : {&g11_, &g22_, &g33_, &g12_, &g13_, &g23_})
			g->add_multimaterial(index, p, "", root_path);
	}

	Eigen::Matrix3d GrowthNeoHookean::natural_configuration(const RowVectorNd &p, double t, int el_id) const
	{
		Eigen::Matrix3d G;
		G << g11_(p, t, el_id), g12_(p, t, el_id), g13_(p, t, el_id),
			g12_(p, t, el_id), g22_(p, t, el_id), g23_(p, t, el_id),
			g13_(p, t, el_id), g23_(p, t, el_id), g33_(p, t, el_id);
		// Sylvester's criterion; reject invalid state instead of repairing it silently.
		if (!G.allFinite() || G(0, 0) <= 0 || G.topLeftCorner<2, 2>().determinant() <= 0 || G.determinant() <= 0)
			log_and_throw_error("GrowthNeoHookean natural configuration must be finite and positive definite (element {})", el_id);
		return G;
	}

	std::map<std::string, Assembler::ParamFunc> GrowthNeoHookean::parameters() const
	{
		std::map<std::string, ParamFunc> out;
		for (const std::string key : {"lambda", "mu", "E", "nu"})
			out[key] = [this, key](const RowVectorNd &uv, const RowVectorNd &p, double t, int e) {
				double lambda, mu;
				params_.lambda_mu(uv, p, t, e, lambda, mu);
				if (key == "lambda")
					return lambda;
				if (key == "mu")
					return mu;
				if (key == "E")
					return mu * (3 * lambda + 2 * mu) / (lambda + mu);
				return lambda / (2 * (lambda + mu));
			};
		for (int i = 0; i < 3; ++i)
			for (int j = i; j < 3; ++j)
				out["g" + std::to_string(i + 1) + std::to_string(j + 1)] =
					[this, i, j](const RowVectorNd &, const RowVectorNd &p, double t, int e) {
						return natural_configuration(p, t, e)(i, j);
					};
		return out;
	}
} // namespace polyfem::assembler
