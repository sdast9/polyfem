#include "Sha256.hpp"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace polyfem::utils
{
	namespace
	{
		constexpr uint32_t K[64] = {
			0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
			0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
			0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
			0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
			0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
			0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
			0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
			0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

		inline uint32_t rotr(const uint32_t x, const int n) { return (x >> n) | (x << (32 - n)); }
	} // namespace

	Sha256::Sha256()
		: state_{{0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19}}
	{
		buffer_.fill(0);
	}

	void Sha256::transform(const uint8_t block[64])
	{
		uint32_t w[64];
		for (int i = 0; i < 16; ++i)
			w[i] = (uint32_t(block[4 * i]) << 24) | (uint32_t(block[4 * i + 1]) << 16) | (uint32_t(block[4 * i + 2]) << 8) | uint32_t(block[4 * i + 3]);
		for (int i = 16; i < 64; ++i)
		{
			const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
			const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
			w[i] = w[i - 16] + s0 + w[i - 7] + s1;
		}

		uint32_t a = state_[0], b = state_[1], c = state_[2], d = state_[3];
		uint32_t e = state_[4], f = state_[5], g = state_[6], h = state_[7];
		for (int i = 0; i < 64; ++i)
		{
			const uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
			const uint32_t ch = (e & f) ^ (~e & g);
			const uint32_t t1 = h + S1 + ch + K[i] + w[i];
			const uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
			const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
			const uint32_t t2 = S0 + maj;
			h = g;
			g = f;
			f = e;
			e = d + t1;
			d = c;
			c = b;
			b = a;
			a = t1 + t2;
		}
		state_[0] += a;
		state_[1] += b;
		state_[2] += c;
		state_[3] += d;
		state_[4] += e;
		state_[5] += f;
		state_[6] += g;
		state_[7] += h;
	}

	void Sha256::update(const void *data, size_t size)
	{
		assert(!finished_ && "Sha256 updated after its digest was taken");
		const uint8_t *bytes = static_cast<const uint8_t *>(data);
		total_bytes_ += size;
		while (size > 0)
		{
			const size_t take = std::min(size, buffer_.size() - buffered_);
			std::memcpy(buffer_.data() + buffered_, bytes, take);
			buffered_ += take;
			bytes += take;
			size -= take;
			if (buffered_ == buffer_.size())
			{
				transform(buffer_.data());
				buffered_ = 0;
			}
		}
	}

	std::string Sha256::hex_digest()
	{
		assert(!finished_ && "Sha256 digest taken twice");
		// Padding: 0x80, zeros up to 56 bytes of the final block, then the
		// message length in bits, big-endian. update() never leaves a full
		// buffer behind, so there is room for the 0x80 byte.
		const uint64_t bit_length = total_bytes_ * 8;
		buffer_[buffered_++] = 0x80;
		if (buffered_ > 56)
		{
			while (buffered_ < 64)
				buffer_[buffered_++] = 0;
			transform(buffer_.data());
			buffered_ = 0;
		}
		while (buffered_ < 56)
			buffer_[buffered_++] = 0;
		for (int i = 0; i < 8; ++i)
			buffer_[56 + i] = uint8_t(bit_length >> (56 - 8 * i));
		transform(buffer_.data());
		buffered_ = 0;
		finished_ = true;

		static const char *digits = "0123456789abcdef";
		std::string hex;
		hex.reserve(64);
		for (const uint32_t word : state_)
			for (int shift = 28; shift >= 0; shift -= 4)
				hex.push_back(digits[(word >> shift) & 0xf]);
		return hex;
	}

	std::string Sha256::of(const std::string &data)
	{
		Sha256 hash;
		hash.update(data);
		return hash.hex_digest();
	}

	std::string Sha256::of_file(const std::string &path)
	{
		std::ifstream file(path, std::ios::binary);
		if (!file.is_open())
			throw std::runtime_error("Could not open " + path + " for hashing");
		Sha256 hash;
		char chunk[1 << 16];
		while (file)
		{
			file.read(chunk, sizeof(chunk));
			hash.update(chunk, size_t(file.gcount()));
		}
		if (file.bad())
			throw std::runtime_error("Could not read " + path + " for hashing");
		return hash.hex_digest();
	}
} // namespace polyfem::utils
