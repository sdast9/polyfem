#include "MshReader.hpp"

#include <polyfem/utils/Logger.hpp>
#include <polyfem/utils/StringUtils.hpp>

#include <mshio/mshio.h>

#include <fstream>
#include <limits>
#include <set>
#include <string>
#include <iostream>
#include <vector>

#include <filesystem> // filesystem

namespace polyfem::io
{
	namespace
	{
		int num_corner_nodes(const int type)
		{
			if (type == 1 || type == 8 || type == 26 || type == 27 || type == 28) // line
				return 2;
			if (type == 2 || type == 9 || type == 21 || type == 23 || type == 25) // triangle
				return 3;
			if (type == 3 || type == 10) // quad
				return 4;
			return -1;
		}
	} // namespace

	template <typename Entity>
	void map_entity_tag_to_physical_tag(const std::vector<Entity> &entities, std::unordered_map<int, int> &entity_tag_to_physical_tag)
	{
		for (int i = 0; i < entities.size(); i++)
		{
			entity_tag_to_physical_tag[entities[i].tag] =
				entities[i].physical_group_tags.size() > 0
					? entities[i].physical_group_tags.front()
					: 0;
		}
	}

	bool MshReader::load(const std::string &path, Eigen::MatrixXd &vertices, Eigen::MatrixXi &cells, std::vector<std::vector<int>> &elements, std::vector<std::vector<double>> &weights, std::vector<int> &body_ids)
	{
		std::vector<std::string> node_data_name;
		std::vector<std::vector<double>> node_data;
		std::vector<std::vector<int>> boundary_elements;
		std::vector<int> boundary_ids;

		return load(path, vertices, cells, elements, weights, body_ids, boundary_elements, boundary_ids, node_data_name, node_data);
	}

	bool MshReader::load(const std::string &path, Eigen::MatrixXd &vertices, Eigen::MatrixXi &cells, std::vector<std::vector<int>> &elements, std::vector<std::vector<double>> &weights, std::vector<int> &body_ids, std::vector<std::vector<int>> &boundary_elements, std::vector<int> &boundary_ids)
	{
		std::vector<std::string> node_data_name;
		std::vector<std::vector<double>> node_data;

		return load(path, vertices, cells, elements, weights, body_ids, boundary_elements, boundary_ids, node_data_name, node_data);
	}

	bool MshReader::load(const std::string &path, Eigen::MatrixXd &vertices, Eigen::MatrixXi &cells, std::vector<std::vector<int>> &elements, std::vector<std::vector<double>> &weights, std::vector<int> &body_ids, std::vector<std::string> &node_data_name, std::vector<std::vector<double>> &node_data)
	{
		std::vector<std::vector<int>> boundary_elements;
		std::vector<int> boundary_ids;

		return load(path, vertices, cells, elements, weights, body_ids, boundary_elements, boundary_ids, node_data_name, node_data);
	}

	bool MshReader::load(const std::string &path, Eigen::MatrixXd &vertices, Eigen::MatrixXi &cells, std::vector<std::vector<int>> &elements, std::vector<std::vector<double>> &weights, std::vector<int> &body_ids, std::vector<std::vector<int>> &boundary_elements, std::vector<int> &boundary_ids, std::vector<std::string> &node_data_name, std::vector<std::vector<double>> &node_data)
	{
		if (!std::filesystem::exists(path))
		{
			logger().error("Msh file does not exist: {}", path);
			return false;
		}

		// A malformed file stops the run with mshio's reason (the line, the
		// section, the offending token and the value it should have been: the
		// checked ASCII reader of cmake/recipes/patches/mshio-checked-ascii-read.patch).
		// The unchecked reader looped forever on a value it could not convert.
		mshio::MshSpec spec;
		try
		{
			spec = mshio::load_msh(path);
		}
		catch (const std::exception &err)
		{
			log_and_throw_error("MSH file {}: {}", path, err.what());
		}
		catch (...)
		{
			log_and_throw_error("MSH file {}: unknown error while reading it", path);
		}

		const auto &nodes = spec.nodes;
		const auto &els = spec.elements;

		// These were asserts, compiled out of release builds: a file without
		// surface or volume elements reached Eigen as a negative column count
		// and stopped as "memory allocation failed".
		int dim = -1;
		for (const auto &e : els.entity_blocks)
		{
			dim = std::max(dim, e.entity_dim);
		}
		if (dim != 2 && dim != 3)
			log_and_throw_error(
				"MSH file {}: the file has no surface or volume elements{}", path,
				dim < 0 ? std::string() : fmt::format(" (its elements have dimension {} at most)", dim));

		// Node tags index tag_to_index. A tag of 0, a tag above the largest
		// tag the file declares (a binary file's header is not cross-checked)
		// or one beyond int was written out of bounds; a tag used twice left a
		// vertex row unset.
		if (nodes.max_node_tag >= static_cast<size_t>(std::numeric_limits<int>::max()))
			log_and_throw_error("MSH file {}: node tag {} is too large (PolyFEM numbers nodes with int)", path, nodes.max_node_tag);
		size_t n_tagged = 0;
		for (const auto &n : nodes.entity_blocks)
			n_tagged += n.tags.size();
		if (n_tagged != nodes.num_nodes)
			log_and_throw_error("MSH file {}: $Nodes declares {} nodes, but its blocks hold {}", path, nodes.num_nodes, n_tagged);

		const int n_vertices = nodes.num_nodes;
		const int max_tag = nodes.max_node_tag;

		vertices.resize(n_vertices, dim);
		std::vector<int> tag_to_index = std::vector<int>(max_tag + 1, -1);
		if (n_vertices != max_tag)
			logger().warn("MSH file contains more node tags than nodes, condensing nodes which will break input node ordering.");

		int index = 0;
		for (size_t b = 0; b < nodes.entity_blocks.size(); ++b)
		{
			const auto &n = nodes.entity_blocks[b];
			// A node stores x y z. A parametric block (MSH 4.1 written with
			// Gmsh's Mesh.SaveParametric) follows them with the node's
			// coordinates on the block's entity: u on a curve, u v on a
			// surface, u v w in a volume. Only x y z are positions; a fixed
			// stride of 3 read the parametric values as the positions of the
			// block's later nodes, and the file loaded with them silently or
			// stopped as a flipped element. mshio range-checks the entity
			// dimension of ASCII files only.
			if (n.parametric == 1 && (n.entity_dim < 0 || n.entity_dim > 3))
				log_and_throw_error("MSH file {}: parametric node block {} has entity dimension {} (0 to 3 expected)", path, b + 1, n.entity_dim);
			const size_t stride = 3 + static_cast<size_t>(n.parametric == 1 ? n.entity_dim : 0);
			if (n.data.size() != n.num_nodes_in_block * stride)
				log_and_throw_error("MSH file {}: node block {} holds {} values for {} nodes, not {} per node", path, b + 1, n.data.size(), n.num_nodes_in_block, stride);

			for (size_t j = 0; j < n.num_nodes_in_block; ++j)
			{
				const size_t tag = n.tags[j];
				if (tag < 1 || tag > static_cast<size_t>(max_tag))
					log_and_throw_error("MSH file {}: node tag {} lies outside the tag range 1 to {} of the file", path, tag, max_tag);
				if (tag_to_index[tag] >= 0)
					log_and_throw_error("MSH file {}: node tag {} is used by two nodes", path, tag);

				const int node_id = n_vertices != max_tag ? (index++) : (tag - 1);

				const size_t i = j * stride; // x of node j
				if (dim == 2)
					vertices.row(node_id) << n.data[i], n.data[i + 1];
				if (dim == 3)
					vertices.row(node_id) << n.data[i], n.data[i + 1], n.data[i + 2];

				tag_to_index[tag] = node_id;
			}
		}

		// RB-11: a node tag that no node carries (or tag 0, which Gmsh never
		// assigns) used to pass an assert compiled out of release builds and
		// reach the mesh builder as index -1 or garbage.
		const auto node_index_of = [&](const long long tag, int &index) {
			if (tag < 0 || tag >= static_cast<long long>(tag_to_index.size()))
				return false;
			index = tag_to_index[tag];
			return index >= 0 && index < n_vertices;
		};

		int cells_cols = -1;
		int num_els = 0;
		for (const auto &e : els.entity_blocks)
		{
			if (e.entity_dim != dim)
				continue;
			const int type = e.element_type;
			// https://shipengcheng1230.github.io/GmshTools.jl/stable/element_types/
			if (type == 2 || type == 9 || type == 21 || type == 23 || type == 25) // tri
			{
				cells_cols = std::max(cells_cols, 3);
				num_els += e.num_elements_in_block;
			}
			else if (type == 3 || type == 10) // quad
			{
				cells_cols = std::max(cells_cols, 4);
				num_els += e.num_elements_in_block;
			}
			else if (type == 4 || type == 11 || type == 29 || type == 30 || type == 31) // tet
			{
				cells_cols = std::max(cells_cols, 4);
				num_els += e.num_elements_in_block;
			}
			else if (type == 5 || type == 12) // hex
			{
				cells_cols = std::max(cells_cols, 8);
				num_els += e.num_elements_in_block;
			}
			else if (type == 6) // prism
			{
				cells_cols = std::max(cells_cols, 6);
				num_els += e.num_elements_in_block;
			}
			else if (type == 7) // pyramid
			{
				cells_cols = std::max(cells_cols, 5);
				num_els += e.num_elements_in_block;
			}
		}
		if (cells_cols < 0)
		{
			std::set<int> types;
			for (const auto &e : els.entity_blocks)
				if (e.entity_dim == dim)
					types.insert(e.element_type);
			std::string type_list;
			for (const int type : types)
				type_list += (type_list.empty() ? "" : ", ") + std::to_string(type);
			log_and_throw_error("MSH file {}: none of its {}D elements has a type PolyFEM reads (Gmsh element types {})", path, dim, type_list);
		}

		std::unordered_map<int, int> entity_tag_to_physical_tag;
		std::unordered_map<int, int> boundary_entity_tag_to_physical_tag;
		if (dim == 2)
		{
			map_entity_tag_to_physical_tag(spec.entities.surfaces, entity_tag_to_physical_tag);
			map_entity_tag_to_physical_tag(spec.entities.curves, boundary_entity_tag_to_physical_tag);
		}
		else
		{
			map_entity_tag_to_physical_tag(spec.entities.volumes, entity_tag_to_physical_tag);
			map_entity_tag_to_physical_tag(spec.entities.surfaces, boundary_entity_tag_to_physical_tag);
		}

		boundary_elements.clear();
		boundary_ids.clear();
		for (const auto &e : els.entity_blocks)
		{
			if (e.entity_dim != dim - 1)
				continue;

			const auto physical_tag = boundary_entity_tag_to_physical_tag.find(e.entity_tag);
			if (physical_tag == boundary_entity_tag_to_physical_tag.end() || physical_tag->second == 0)
				continue;

			const int n_corners = num_corner_nodes(e.element_type);
			if (n_corners < 0)
			{
				logger().warn("Ignoring unsupported tagged codimension-one Gmsh element type {}.", e.element_type);
				continue;
			}

			const size_t n_nodes = mshio::nodes_per_element(e.element_type);
			for (int i = 0; i < e.data.size(); i += n_nodes + 1)
			{
				std::vector<int> corners(n_corners);
				for (int j = 0; j < n_corners; ++j)
				{
					const int node_tag = e.data[i + j + 1];
					if (!node_index_of(node_tag, corners[j]))
					{
						log_and_throw_error("MSH file {}: tagged side element {} references node tag {}, which is not a node of the file", path, e.data[i], node_tag);
					}
				}
				boundary_elements.emplace_back(std::move(corners));
				boundary_ids.push_back(physical_tag->second);
			}
		}

		cells.resize(num_els, cells_cols);
		cells.setConstant(-1);
		body_ids.resize(num_els);
		elements.resize(num_els);
		weights.resize(num_els);
		int cell_index = 0;
		for (const auto &e : els.entity_blocks)
		{
			if (e.entity_dim != dim)
				continue;
			const int type = e.element_type;
			if (type == 2 || type == 9 || type == 21 || type == 23 || type == 25 || type == 3 || type == 10 || type == 4 || type == 11 || type == 29 || type == 30 || type == 31 || type == 5 || type == 12 || type == 6 || type == 7)
			{
				const size_t n_nodes = mshio::nodes_per_element(type);
				int local_cells_cols = -1;

				if (type == 2 || type == 9 || type == 21 || type == 23 || type == 25) // tri
					local_cells_cols = 3;
				else if (type == 3 || type == 10) // quad
					local_cells_cols = 4;
				else if (type == 4 || type == 11 || type == 29 || type == 30 || type == 31) // tet
					local_cells_cols = 4;
				else if (type == 5 || type == 12) // hex
					local_cells_cols = 8;
				else if (type == 6) // prism
					local_cells_cols = 6;
				else if (type == 7) // pyramid
					local_cells_cols = 5;

				for (int i = 0; i < e.data.size(); i += (n_nodes + 1))
				{
					int index = 0;
					for (int j = i + 1; j <= i + local_cells_cols; ++j)
					{
						int v_index = -1;
						if (!node_index_of(e.data[j], v_index))
						{
							log_and_throw_error("MSH file {}: element {} references node tag {}, which is not a node of the file", path, e.data[i], e.data[j]);
						}
						cells(cell_index, index++) = v_index;
					}

					for (int j = i + 1; j < i + 1 + n_nodes; ++j)
					{
						int v_index = -1;
						if (!node_index_of(e.data[j], v_index))
						{
							log_and_throw_error("MSH file {}: element {} references node tag {}, which is not a node of the file", path, e.data[i], e.data[j]);
						}
						elements[cell_index].push_back(v_index);
					}

					const auto &it = entity_tag_to_physical_tag.find(e.entity_tag);
					body_ids[cell_index] =
						it != entity_tag_to_physical_tag.end() ? it->second : 0;

					++cell_index;
				}
			}
		}

		node_data.resize(spec.node_data.size());
		int i = 0;
		for (const auto &data : spec.node_data)
		{
			for (const auto &str : data.header.string_tags)
				node_data_name.push_back(str);

			for (const auto &entry : data.entries)
				for (const auto &d : entry.data)
					node_data[i].push_back(d);

			i++;
		}

		// std::ifstream infile(path.c_str());

		// std::string line;

		// int phase = -1;
		// int line_number = -1;
		// bool size_read = false;

		// int n_triangles = 0;
		// int n_tets = 0;

		// std::vector<std::vector<double>> all_elements;

		// while (std::getline(infile, line))
		// {
		// 	line = StringUtils::trim(line);
		// 	++line_number;

		// 	if (line.empty())
		// 		continue;

		// 	if (line[0] == '$')
		// 	{
		// 		if (line.substr(1, 3) == "End")
		// 			phase = -1;
		// 		else
		// 		{
		// 			const auto header = line.substr(1);

		// 			if (header.find("MeshFormat") == 0)
		// 				phase = 0;
		// 			else if (header.find("Nodes") == 0)
		// 				phase = 1;
		// 			else if (header.find("Elements") == 0)
		// 				phase = 2;
		// 			else
		// 			{
		// 				logger().debug("{}: [Warning] ignoring {}", line_number, header);
		// 				phase = -1;
		// 			}
		// 		}

		// 		size_read = false;

		// 		continue;
		// 	}

		// 	if (phase == -1)
		// 		continue;

		// 	std::istringstream iss(line);
		// 	//header
		// 	if (phase == 0)
		// 	{
		// 		double version_number;
		// 		int file_type;
		// 		int data_size;

		// 		iss >> version_number >> file_type >> data_size;

		// 		assert(version_number == 2.2);
		// 		assert(file_type == 0);
		// 		assert(data_size == 8);
		// 	}
		// 	//coordiantes
		// 	else if (phase == 1)
		// 	{
		// 		if (!size_read)
		// 		{
		// 			int n_vertices;
		// 			iss >> n_vertices;
		// 			vertices.resize(n_vertices, 3);
		// 			size_read = true;
		// 		}
		// 		else
		// 		{
		// 			int node_number;
		// 			double x_coord, y_coord, z_coord;

		// 			iss >> node_number >> x_coord >> y_coord >> z_coord;
		// 			//node_numbers starts with 1
		// 			vertices.row(node_number - 1) << x_coord, y_coord, z_coord;
		// 		}
		// 	}
		// 	//elements
		// 	else if (phase == 2)
		// 	{
		// 		if (!size_read)
		// 		{
		// 			int number_of_elements;
		// 			iss >> number_of_elements;
		// 			all_elements.resize(number_of_elements);
		// 			size_read = true;
		// 		}
		// 		else
		// 		{
		// 			int elm_number, elm_type, number_of_tags;

		// 			iss >> elm_number >> elm_type >> number_of_tags;

		// 			//9-node third order incomplete triangle
		// 			assert(elm_type != 20);

		// 			//12-node fourth order incomplete triangle
		// 			assert(elm_type != 22);

		// 			//15-node fifth order incomplete triangle
		// 			assert(elm_type != 24);

		// 			//21-node fifth order complete triangle
		// 			assert(elm_type != 25);

		// 			//56-node fifth order tetrahedron
		// 			assert(elm_type != 31);

		// 			//60 is the new rational element
		// 			if (elm_type == 2 || elm_type == 9 || elm_type == 21 || elm_type == 23 || elm_type == 60)
		// 				++n_triangles;
		// 			else if (elm_type == 4 || elm_type == 11 || elm_type == 29 || elm_type == 30)
		// 				++n_tets;

		// 			//skipping tags
		// 			for (int i = 0; i < number_of_tags; ++i)
		// 			{
		// 				int tmp;
		// 				iss >> tmp;
		// 			}

		// 			auto &node_list = all_elements[elm_number - 1];
		// 			node_list.push_back(elm_type);

		// 			while (iss.good())
		// 			{
		// 				double tmp;
		// 				iss >> tmp;
		// 				node_list.push_back(tmp);
		// 			}
		// 		}
		// 	}
		// 	else
		// 	{
		// 		assert(false);
		// 	}
		// }

		// int index = 0;
		// if (n_tets == 0)
		// {
		// 	elements.resize(n_triangles);
		// 	weights.resize(n_triangles);
		// 	cells.resize(n_triangles, 3);

		// 	for (const auto &els : all_elements)
		// 	{
		// 		const int elm_type = els[0];
		// 		if (elm_type != 2 && elm_type != 9 && elm_type != 21 && elm_type != 23 && elm_type != 60)
		// 			continue;

		// 		auto &el = elements[index];
		// 		auto &wh = weights[index];
		// 		for (size_t i = 1; i < (elm_type == 60 ? 7 : els.size()); ++i)
		// 			el.push_back(int(els[i]) - 1);
		// 		if (elm_type == 60)
		// 		{
		// 			for (size_t i = 7; i < els.size(); ++i)
		// 				wh.push_back(els[i]);

		// 			assert(wh.size() == el.size());
		// 			assert(wh.size() == 6);
		// 		}

		// 		cells.row(index) << el[0], el[1], el[2];

		// 		++index;
		// 	}
		// }
		// else
		// {
		// 	elements.resize(n_tets);
		// 	weights.resize(n_tets);
		// 	cells.resize(n_tets, 4);

		// 	for (const auto &els : all_elements)
		// 	{
		// 		const int elm_type = els[0];
		// 		if (elm_type != 4 && elm_type != 11 && elm_type != 29 && elm_type != 30)
		// 			continue;

		// 		auto &el = elements[index];
		// 		auto &wh = weights[index];
		// 		for (size_t i = 1; i < els.size(); ++i)
		// 			el.push_back(els[i] - 1);

		// 		cells.row(index) << el[0], el[1], el[2], el[3];
		// 		++index;
		// 	}
		// }

		return true;
	}
} // namespace polyfem::io
