#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace polyfem::utils
{
	/// Lazy input snapshot: the first successful read of each path/field wins.
	/// Keep one instance per simulation, including remeshing/material rebuilds.
	/// A new input initialization must create a new instance. No timestamp checks
	/// or process-wide invalidation: readers of old snapshots remain valid.
	class MaterialFileCache
	{
	public:
		using Fibers = std::vector<Eigen::Vector3d>;
		std::shared_ptr<const Eigen::MatrixXd> find_matrix(const std::string &path)
		{
			std::lock_guard<std::mutex> lock(mutex_);
			const auto it = matrices_.find(canonical_path(path));
			return it == matrices_.end() ? nullptr : it->second;
		}
		template <typename Loader>
		std::shared_ptr<const Eigen::MatrixXd> matrix(const std::string &path, Loader load)
		{
			std::lock_guard<std::mutex> lock(mutex_);
			const auto key = canonical_path(path);
			auto it = matrices_.find(key);
			if (it != matrices_.end())
				return it->second;
			auto value = std::make_shared<const Eigen::MatrixXd>(load());
			matrices_.emplace(key, value);
			return value;
		}
		template <typename Loader>
		std::shared_ptr<const Fibers> fibers(const std::string &path, const std::string &field, Loader load)
		{
			std::lock_guard<std::mutex> lock(mutex_);
			const auto key = std::make_pair(canonical_path(path), field);
			auto it = fibers_.find(key);
			if (it != fibers_.end())
				return it->second;
			auto value = std::make_shared<const Fibers>(load());
			fibers_.emplace(key, value);
			return value;
		}

	private:
		static std::string canonical_path(const std::string &path)
		{
			return std::filesystem::absolute(path).lexically_normal().string();
		}
		std::mutex mutex_;
		std::map<std::string, std::shared_ptr<const Eigen::MatrixXd>> matrices_;
		std::map<std::pair<std::string, std::string>, std::shared_ptr<const Fibers>> fibers_;
	};

	/// Bind an explicitly owned snapshot to synchronous material initialization.
	/// Nested scopes restore the previous binding, including on exceptions.
	/// Worker threads must each establish a scope with the same snapshot; the
	/// cache itself serializes first loads. Threads never inherit this binding.
	/// Without a scope, a standalone load is an independent fresh input read.
	class MaterialFileCacheScope
	{
	public:
		explicit MaterialFileCacheScope(std::shared_ptr<MaterialFileCache> snapshot)
			: previous_(current_)
		{
			current_ = snapshot ? std::move(snapshot) : std::make_shared<MaterialFileCache>();
		}
		~MaterialFileCacheScope() { current_ = std::move(previous_); }
		MaterialFileCacheScope(const MaterialFileCacheScope &) = delete;
		MaterialFileCacheScope &operator=(const MaterialFileCacheScope &) = delete;
		static std::shared_ptr<MaterialFileCache> current()
		{
			return current_ ? current_ : std::make_shared<MaterialFileCache>();
		}

	private:
		std::shared_ptr<MaterialFileCache> previous_;
		inline static thread_local std::shared_ptr<MaterialFileCache> current_;
	};
} // namespace polyfem::utils
