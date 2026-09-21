#pragma once

// Reading the Float64 point-data arrays PolyFEM writes to its VTU files
// (uncompressed inline base64 with UInt64 byte-count headers; only that
// layout is accepted). Shared by the scene tests that check saved fields
// (RBR-01 output kinematics, the fully prescribed body).
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace polyfem::test
{
	inline std::vector<unsigned char> base64_decode(const std::string &text)
	{
		std::vector<unsigned char> bytes;
		unsigned int buffer = 0;
		int bits = 0;
		for (const char c : text)
		{
			int value;
			if (c >= 'A' && c <= 'Z')
				value = c - 'A';
			else if (c >= 'a' && c <= 'z')
				value = c - 'a' + 26;
			else if (c >= '0' && c <= '9')
				value = c - '0' + 52;
			else if (c == '+')
				value = 62;
			else if (c == '/')
				value = 63;
			else
				continue; // '=' padding, whitespace
			buffer = (buffer << 6) | value;
			bits += 6;
			if (bits >= 8)
			{
				bits -= 8;
				bytes.push_back(static_cast<unsigned char>((buffer >> bits) & 0xFF));
			}
		}
		return bytes;
	}

	// A named Float64 point-data array of a VTU as rows x components.
	inline Eigen::MatrixXd read_vtu_field(const std::filesystem::path &path, const std::string &name)
	{
		std::ifstream file(path);
		REQUIRE(file.is_open());
		std::stringstream buffer;
		buffer << file.rdbuf();
		const std::string vtu = buffer.str();
		REQUIRE(vtu.find("header_type=\"UInt64\"") != std::string::npos);
		REQUIRE(vtu.find("compressor=") == std::string::npos);

		const std::string tag = "Name=\"" + name + "\"";
		const size_t name_pos = vtu.find(tag);
		REQUIRE(name_pos != std::string::npos);
		const size_t open = vtu.rfind("<DataArray", name_pos);
		const size_t header_end = vtu.find('>', name_pos);
		const size_t close = vtu.find("</DataArray>", header_end);
		REQUIRE(open != std::string::npos);
		REQUIRE(header_end != std::string::npos);
		REQUIRE(close != std::string::npos);
		const std::string header = vtu.substr(open, header_end - open);
		REQUIRE(header.find("type=\"Float64\"") != std::string::npos);
		REQUIRE(header.find("format=\"binary\"") != std::string::npos);
		int components = 1;
		const size_t nc = header.find("NumberOfComponents=\"");
		if (nc != std::string::npos)
			components = std::stoi(header.substr(nc + 20));

		const std::vector<unsigned char> bytes = base64_decode(vtu.substr(header_end + 1, close - header_end - 1));
		REQUIRE(bytes.size() >= 8);
		std::uint64_t byte_count = 0;
		std::memcpy(&byte_count, bytes.data(), 8);
		REQUIRE(byte_count == bytes.size() - 8);
		REQUIRE(byte_count % (8 * components) == 0);
		const Eigen::Index rows = byte_count / (8 * components);
		Eigen::MatrixXd values(rows, components);
		for (Eigen::Index i = 0; i < rows; ++i)
			for (int c = 0; c < components; ++c)
				std::memcpy(&values(i, c), bytes.data() + 8 + (i * components + c) * 8, 8);
		return values;
	}
} // namespace polyfem::test
