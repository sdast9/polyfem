#include "OgdenElasticity.hpp"

#include <polyfem/autogen/auto_eigs.hpp>
#include <polyfem/utils/Logger.hpp>

namespace polyfem::assembler
{
	UnconstrainedOgdenElasticity::UnconstrainedOgdenElasticity()
		: alphas_("alphas"), mus_("mus"), Ds_("Ds")
	{
	}

	void UnconstrainedOgdenElasticity::add_multimaterial(const int index, const json &params, const Units &units, const std::string &root_path)
	{
		alphas_.add_multimaterial(index, params, "", root_path);
		mus_.add_multimaterial(index, params, units.stress(), root_path);
		Ds_.add_multimaterial(index, params, units.stress(), root_path);
		// RB-11: the energy loops over the alphas and reads mus[N] for each; a
		// mismatch silently dropped terms or read out of range (the assert is dead
		// in release builds). Checked only when this law is configured: under
		// MultiModels every law sees every body's parameters.
		const bool configured = params.contains("alphas") || params.contains("mus") || params.contains("Ds");
		if (configured && alphas_.size() != mus_.size())
			log_and_throw_error(
				"UnconstrainedOgden: 'alphas' has {} term(s) but 'mus' has {}; every Ogden term needs one alpha and one mu",
				alphas_.size(), mus_.size());
		if (configured && Ds_.size() == 0)
			log_and_throw_error("UnconstrainedOgden: 'Ds' needs at least one volumetric coefficient");
	}

	std::map<std::string, Assembler::ParamFunc> UnconstrainedOgdenElasticity::parameters() const
	{
		std::map<std::string, ParamFunc> res;
		const auto &alphas = this->alphas();
		const auto &mus = this->mus();
		const auto &Ds = this->Ds();

		for (int i = 0; i < alphas.size(); ++i)
			res[fmt::format("alpha_{}", i)] = [&alphas, i](const RowVectorNd &, const RowVectorNd &p, double t, int e) {
				return alphas[i](p, t, e);
			};

		for (int i = 0; i < mus.size(); ++i)
			res[fmt::format("mu_{}", i)] = [&mus, i](const RowVectorNd &, const RowVectorNd &p, double t, int e) {
				return mus[i](p, t, e);
			};

		for (int i = 0; i < Ds.size(); ++i)
			res[fmt::format("D_{}", i)] = [&Ds, i](const RowVectorNd &, const RowVectorNd &p, double t, int e) {
				return Ds[i](p, t, e);
			};
		return res;
	}

	// =========================================================================

	IncompressibleOgdenElasticity::IncompressibleOgdenElasticity()
		: coefficients_("c"), expoenents_("m"), bulk_modulus_("k")
	{
	}

	void IncompressibleOgdenElasticity::add_multimaterial(const int index, const json &params, const Units &units, const std::string &root_path)
	{
		coefficients_.add_multimaterial(index, params, units.stress(), root_path);
		expoenents_.add_multimaterial(index, params, "", root_path);
		bulk_modulus_.add_multimaterial(index, params, units.stress(), root_path);
		if ((params.contains("c") || params.contains("m")) && coefficients_.size() != expoenents_.size())
			log_and_throw_error(
				"IncompressibleOgden: 'c' has {} term(s) but 'm' has {}; every Ogden term needs one coefficient and one exponent",
				coefficients_.size(), expoenents_.size());
	}

	std::map<std::string, Assembler::ParamFunc> IncompressibleOgdenElasticity::parameters() const
	{
		std::map<std::string, ParamFunc> res;

		const auto &coefficients = this->coefficients();
		const auto &expoenents = this->expoenents();
		const auto &k = this->bulk_modulus();

		for (int i = 0; i < coefficients.size(); ++i)
			res[fmt::format("c_{}", i)] = [&coefficients, i](const RowVectorNd &, const RowVectorNd &p, double t, int e) {
				return coefficients[i](p, t, e);
			};

		for (int i = 0; i < expoenents.size(); ++i)
			res[fmt::format("m_{}", i)] = [&expoenents, i](const RowVectorNd &, const RowVectorNd &p, double t, int e) {
				return expoenents[i](p, t, e);
			};

		res["k"] = [&k](const RowVectorNd &, const RowVectorNd &p, double t, int e) {
			return k(p, t, e);
		};

		return res;
	}
} // namespace polyfem::assembler
