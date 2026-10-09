#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace polyfem::utils
{
	/// @brief SHA-256 (FIPS 180-4) for the run manifest's content hashes
	///        (RB-12): the executable, the effective input and every file
	///        the input refers to. A self-contained implementation so that
	///        the identity of a run never depends on an optional system
	///        library; the known-answer tests are in tests/test_run_manifest.cpp.
	class Sha256
	{
	public:
		Sha256();

		/// @brief Absorb bytes; may be called repeatedly.
		void update(const void *data, size_t size);
		void update(const std::string &data) { update(data.data(), data.size()); }

		/// @brief Finish and return the digest as 64 lowercase hex characters.
		///        The object must not be updated afterwards.
		std::string hex_digest();

		/// @brief Digest of a string.
		static std::string of(const std::string &data);

		/// @brief Digest of a file's bytes, streamed. Throws std::runtime_error
		///        (naming the path) when the file cannot be read.
		static std::string of_file(const std::string &path);

	private:
		void transform(const uint8_t block[64]);

		std::array<uint32_t, 8> state_;
		std::array<uint8_t, 64> buffer_;
		size_t buffered_ = 0;
		uint64_t total_bytes_ = 0;
		bool finished_ = false;
	};
} // namespace polyfem::utils
