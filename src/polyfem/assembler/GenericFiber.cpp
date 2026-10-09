#include "GenericFiber.hpp"

#include <polyfem/assembler/HGOFiber.hpp>
#include <polyfem/assembler/ActiveFiber.hpp>
#include <polyfem/assembler/HGODispersion.hpp>

namespace polyfem::assembler
{
	template <typename FiberModel>
	GenericFiber<FiberModel>::GenericFiber()
	{
	}

	template <typename FiberModel>
	void GenericFiber<FiberModel>::add_multimaterial(const int index, const json &params, const Units &units, const std::string &root_path)
	{
		if (params.contains("fiber_direction"))
			fiber_direction_.add_multimaterial(index, params["fiber_direction"], units.length(), root_path, params);
	}

	template <typename FiberModel>
	void GenericFiber<FiberModel>::set_size(const int size)
	{
		GenericElastic<FiberModel>::set_size(size);

		fiber_direction_.resize(size);
	}

	template <typename FiberModel>
	std::map<std::string, Assembler::ParamFunc> GenericFiber<FiberModel>::parameters() const
	{
		std::map<std::string, Assembler::ParamFunc> res;

		const auto &fiber_direction = this->fiber_direction_;

		// The functor returns a size x 1 direction or a size x size structure
		// tensor; read it at its own size (a fixed Vector3d asserted on the 2D
		// direction in Debug builds and read past it in Release, RB-12).
		const auto component = [&fiber_direction](const RowVectorNd &p, const double t, const int e, const int k) -> double {
			const auto dir = fiber_direction(p, p, t, e);
			if (dir.cols() == 1)
				return k < dir.rows() ? dir(k, 0) : 0.0;
			// the tensor form: the weight of axis k
			return k < dir.rows() && k < dir.cols() ? dir(k, k) : 0.0;
		};

		res["fiber_direction_x"] = [component](const RowVectorNd &, const RowVectorNd &p, double t, int e) {
			return component(p, t, e, 0);
		};

		res["fiber_direction_y"] = [component](const RowVectorNd &, const RowVectorNd &p, double t, int e) {
			return component(p, t, e, 1);
		};

		if (this->size() == 3)
		{
			res["fiber_direction_z"] = [component](const RowVectorNd &, const RowVectorNd &p, double t, int e) {
				return component(p, t, e, 2);
			};
		}

		return res;
	}

	template class GenericFiber<HGOFiber>;
	template class GenericFiber<ActiveFiber>;
	template class GenericFiber<HGODispersion>;
} // namespace polyfem::assembler