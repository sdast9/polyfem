#pragma once

#include <polyfem/assembler/GenericElastic.hpp>
#include <polyfem/assembler/MatParams.hpp>

namespace polyfem::assembler
{
	// Frozen natural configuration for an external growth/remodeling integrator.
	// Energy is per ORIGINAL volume: det(G) W(F G^-1). G is SPD in material axes.
	// Use MaterialSum with an ordinary law for an implant that does not grow.
	class GrowthNeoHookean : public GenericElastic<GrowthNeoHookean>
	{
	public:
		std::string name() const override { return "GrowthNeoHookean"; }
		bool allow_inversion() const override { return false; }
		std::map<std::string, ParamFunc> parameters() const override;
		void add_multimaterial(int index, const json &params, const Units &units, const std::string &root_path) override;
		Eigen::Matrix3d natural_configuration(const RowVectorNd &p, double t, int el_id) const;

		template <typename T>
		T elastic_energy(const RowVectorNd &p, double t, int el_id, const DefGradMatrix<T> &F) const
		{
			const Eigen::Matrix3d G = natural_configuration(p, t, el_id);
			const DefGradMatrix<T> Fe = F * G.inverse().template cast<T>();
			double lambda, mu;
			params_.lambda_mu(p, p, t, el_id, lambda, mu);
			const T logJ = log(polyfem::utils::determinant(Fe));
			return G.determinant() * (mu / 2 * ((Fe * Fe.transpose()).trace() - 3 - 2 * logJ) + lambda / 2 * logJ * logJ);
		}

	private:
		LameParameters params_;
		GenericMatParam g11_{"g11"}, g22_{"g22"}, g33_{"g33"};
		GenericMatParam g12_{"g12"}, g13_{"g13"}, g23_{"g23"};
	};
} // namespace polyfem::assembler
