#pragma once
#include <polyfem/io/Checkpoint.hpp>
#include <polyfem/utils/Logger.hpp>
#include <cmath>
#include <limits>

namespace polyfem::io
{
	// Missing owner groups denote upstream-only checkpoints; present groups
	// must have the supported layout rather than being silently ignored.
	inline bool has_fork_checkpoint_group(const CheckpointReader &reader, const std::string &group)
	{
		if (!reader.exists(group))
			return false;
		const long version = reader.read_integer_attribute(group, "layout_version");
		if (version != 1)
			log_and_throw_error("Checkpoint {}: unsupported fork layout version {} at {} (expected 1)", reader.path().string(), version, group);
		return true;
	}

	inline bool read_fork_checkpoint_matrix(const CheckpointReader &reader, const std::string &group, const std::string &name, Eigen::MatrixXd &matrix)
	{
		if (!reader.exists(group + "/" + name))
			log_and_throw_error("Checkpoint {}: missing required fork dataset {}/{}", reader.path().string(), group, name);
		matrix = reader.read_matrix(group + "/" + name);
		return true;
	}

	inline void validate_checkpoint_multipliers(const Eigen::MatrixXd &values, bool reorder, const std::string &field)
	{
		if (!values.allFinite())
			log_and_throw_error("Checkpoint {}: augmented-Lagrangian multipliers must be finite", field);
		// Exact zero is permutation invariant. Never discard nonzero history
		// or use a tolerance to make an unsupported reorder appear safe.
		if (reorder && (values.array() != 0).any())
			log_and_throw_error("Checkpoint {}: input.checkpoint.reorder is not supported with nonzero augmented-Lagrangian history; resume without reordering.", field);
	}

	template <typename Integer>
	inline Integer checked_checkpoint_integer(double value, const std::string &field, bool nonnegative = true)
	{
		if (!std::isfinite(value) || std::floor(value) != value
			|| (nonnegative && value < 0)
			|| static_cast<long double>(value) < static_cast<long double>(std::numeric_limits<Integer>::lowest())
			|| static_cast<long double>(value) > static_cast<long double>(std::numeric_limits<Integer>::max()))
			log_and_throw_error("Checkpoint fork field {} has invalid integer value {}", field, value);
		return static_cast<Integer>(value);
	}
} // namespace polyfem::io
