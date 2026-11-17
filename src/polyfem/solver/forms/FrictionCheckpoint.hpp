#pragma once
#include <polyfem/utils/Logger.hpp>
#include <ipc/collisions/tangential/tangential_collisions.hpp>
#include <array>
#include <map>
#include <set>
#include <tuple>
#include <cmath>

namespace polyfem::solver::friction_checkpoint
{
	// Preserve original orientation and signed duplicate contributions.
	struct Key
	{
		std::array<long, 4> ids{}; // type, primitive 0, primitive 1, weight sign
		std::array<double, 4> plane{};
		bool operator<(const Key &other) const { return std::tie(ids, plane) < std::tie(other.ids, other.plane); }
	};
	inline long sign(double weight)
	{
		if (!std::isfinite(weight))
			log_and_throw_error("Checkpoint friction weight must be finite");
		return (weight > 0) - (weight < 0);
	}
	inline std::vector<Key> keys(const ipc::TangentialCollisions &collisions)
	{
		std::vector<Key> result;
		for (const auto &c : collisions.vv_collisions)
			result.push_back({{0, long(c.vertex0_id), long(c.vertex1_id), sign(c.weight)}, {}});
		for (const auto &c : collisions.ev_collisions)
			result.push_back({{1, long(c.edge_id), long(c.vertex_id), sign(c.weight)}, {}});
		for (const auto &c : collisions.ee_collisions)
			result.push_back({{2, long(c.edge0_id), long(c.edge1_id), sign(c.weight)}, {}});
		for (const auto &c : collisions.fv_collisions)
			result.push_back({{3, long(c.face_id), long(c.vertex_id), sign(c.weight)}, {}});
		for (const auto &c : collisions.pv_collisions)
		{
			Key key{{4, long(c.vertex_id), 0, sign(c.weight)}, {}};
			for (int i = 0; i < 4; ++i)
				key.plane[i] = c.plane.coeffs()(i);
			result.push_back(key);
		}
		return result;
	}
	// Returns rebuilt indices in saved order; never changes live collisions.
	inline std::vector<size_t> match(const std::vector<Key> &saved, const std::vector<Key> &rebuilt)
	{
		if (saved.size() != rebuilt.size())
			log_and_throw_error("Checkpoint friction collision identity count mismatch");
		std::map<Key, size_t> positions;
		const auto validate = [](const Key &key) {
			if (key.ids[0] < 0 || key.ids[0] > 4 || key.ids[1] < 0 || key.ids[2] < 0 || (key.ids[3] < -1 || key.ids[3] > 1))
				log_and_throw_error("Checkpoint friction collision identity is invalid");
			for (double coefficient : key.plane)
				if (!std::isfinite(coefficient))
					log_and_throw_error("Checkpoint friction plane identity must be finite");
		};
		for (size_t i = 0; i < rebuilt.size(); ++i)
		{
			validate(rebuilt[i]);
			if (!positions.emplace(rebuilt[i], i).second)
				log_and_throw_error("Checkpoint friction rebuilt collision identity is ambiguous");
		}
		std::vector<size_t> order;
		std::set<Key> seen;
		long previous_type = -1;
		for (const auto &key : saved)
		{
			validate(key);
			if (key.ids[0] < previous_type || !seen.insert(key).second)
				log_and_throw_error("Checkpoint friction saved collision identity is duplicated or out of typed order");
			previous_type = key.ids[0];
			const auto found = positions.find(key);
			if (found == positions.end())
				log_and_throw_error("Checkpoint friction collision identity/orientation mismatch");
			order.push_back(found->second);
		}
		return order;
	}
	inline ipc::TangentialCollisions reordered(const ipc::TangentialCollisions &rebuilt, const std::vector<size_t> &order)
	{
		ipc::TangentialCollisions result = rebuilt;
		size_t offset = 0;
		const auto reorder = [&](const auto &source, auto &destination) {
			for (size_t i = 0; i < source.size(); ++i)
			{
				const size_t index = order.at(offset + i);
				if (index < offset || index - offset >= source.size())
					log_and_throw_error("Checkpoint friction collision type mismatch");
				destination[i] = source[index - offset];
			}
			offset += source.size();
		};
		reorder(rebuilt.vv_collisions, result.vv_collisions);
		reorder(rebuilt.ev_collisions, result.ev_collisions);
		reorder(rebuilt.ee_collisions, result.ee_collisions);
		reorder(rebuilt.fv_collisions, result.fv_collisions);
		reorder(rebuilt.pv_collisions, result.pv_collisions);
		return result;
	}
} // namespace polyfem::solver::friction_checkpoint
