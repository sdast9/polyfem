#pragma once

// Read uncompressed VTU arrays with UInt64 headers from inline Base64
// or raw appended storage. This serves the scene tests for saved fields.
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

	struct VtuDataArray
	{
		std::string header;
		std::vector<unsigned char> payload;
	};

	inline VtuDataArray read_vtu_array(const std::filesystem::path &path, const std::string &name)
	{
		CAPTURE(path, name);
		std::ifstream file(path, std::ios::binary);
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
		REQUIRE(open != std::string::npos);
		REQUIRE(header_end != std::string::npos);
		VtuDataArray array;
		array.header = vtu.substr(open, header_end - open);
		if (array.header.find("format=\"appended\"") != std::string::npos)
		{
			const std::string offset_key = "offset=\"";
			const size_t offset_pos = array.header.find(offset_key);
			REQUIRE(offset_pos != std::string::npos);
			const uint64_t offset = std::stoull(array.header.substr(offset_pos + offset_key.size()));
			const std::string marker = "<AppendedData encoding=\"raw\">\n_";
			const size_t found = vtu.find(marker, header_end);
			const size_t footer = vtu.rfind("</AppendedData>");
			REQUIRE(found != std::string::npos);
			REQUIRE(footer != std::string::npos);
			const size_t start = found + marker.size();
			REQUIRE(footer >= start);
			REQUIRE(offset <= footer - start);
			const size_t block = start + offset;
			REQUIRE(sizeof(uint64_t) <= footer - block);
			uint64_t count = 0;
			std::memcpy(&count, vtu.data() + block, sizeof(count));
			REQUIRE(count <= footer - block - sizeof(count));
			array.payload.assign(vtu.begin() + block + sizeof(count), vtu.begin() + block + sizeof(count) + count);
		}
		else
		{
			REQUIRE(array.header.find("format=\"binary\"") != std::string::npos);
			const size_t close = vtu.find("</DataArray>", header_end);
			REQUIRE(close != std::string::npos);
			const auto bytes = base64_decode(vtu.substr(header_end + 1, close - header_end - 1));
			REQUIRE(bytes.size() >= sizeof(uint64_t));
			uint64_t count = 0;
			std::memcpy(&count, bytes.data(), sizeof(count));
			REQUIRE(count == bytes.size() - sizeof(count));
			array.payload.assign(bytes.begin() + sizeof(count), bytes.end());
		}
		return array;
	}

	// A named Float64 point-data array of a VTU as rows x components.
	inline Eigen::MatrixXd read_vtu_field(const std::filesystem::path &path, const std::string &name)
	{
		const auto array = read_vtu_array(path, name);
		REQUIRE(array.header.find("type=\"Float64\"") != std::string::npos);
		int components = 1;
		const size_t nc = array.header.find("NumberOfComponents=\"");
		if (nc != std::string::npos)
			components = std::stoi(array.header.substr(nc + 20));
		REQUIRE(components > 0);
		REQUIRE(array.payload.size() % (sizeof(double) * components) == 0);
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> values(
			array.payload.size() / (sizeof(double) * components), components);
		if (!array.payload.empty())
			std::memcpy(values.data(), array.payload.data(), array.payload.size());
		return values;
	}
} // namespace polyfem::test
