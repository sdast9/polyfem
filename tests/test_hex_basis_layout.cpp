// RB-23: Q3 hexahedral basis node bookkeeping.
//
// Every basis function's stored node position must be the image of its
// reference node (autogen::q_nodes_3d) under the element's geometric map, and
// elements sharing a global node must map it to the same point. Before RB-23
// the hexahedral builder listed the interior nodes of the vertical edges
// e5-e7 against the autogen direction and enumerated every face's and the
// cell's interior nodes in the mesh's own frame instead of the layout's, so at
// Q3 basis j was stored at another node's position (124 of 256 on the 4-hex
// column) and the global space was discontinuous across faces (the 12 shared
// face-interior nodes disagreed). Q1/Q2 own at most one node per edge, face
// and cell and were never affected.
//
// These tests use the bases the way the solver does, by interpolation, so a
// bookkeeping error shows up as a lost polynomial reproduction, a jump across
// a shared face or a wrong convergence rate, independently of any collision
// code. The direct node-position checks live in test_hex_collision_surface.cpp
// ([hex_nodes]).

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <polyfem/State.hpp>
#include <polyfem/autogen/auto_q_bases.hpp>
#include <polyfem/mesh/mesh3D/Mesh3D.hpp>

#include "VarFormTestAccess.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <string>

using namespace polyfem;
using Catch::Matchers::ContainsSubstring;

namespace
{
	struct Built
	{
		std::shared_ptr<State> state;
		test::VarFormDebugData debug;
	};

	// Builds the FE space (no contact, no solve) of a mesh at the given order;
	// `order` may also be a path to a per-element order file.
	Built build(const std::string &mesh_path, const json &order, const std::string &basis_type = "Lagrange")
	{
		json in_args;
		in_args["/geometry/0/mesh"_json_pointer] = mesh_path;
		in_args["/materials/type"_json_pointer] = "NeoHookean";
		in_args["/materials/E"_json_pointer] = 1e5;
		in_args["/materials/nu"_json_pointer] = 0.3;
		in_args["/materials/rho"_json_pointer] = 1e3;
		in_args["/space/discr_order"_json_pointer] = order;
		if (basis_type != "Lagrange")
			in_args["/space/basis_type"_json_pointer] = basis_type;
		in_args["/contact/enabled"_json_pointer] = false;
		in_args["/time/time_steps"_json_pointer] = 1;
		in_args["/time/tend"_json_pointer] = 1;
		in_args["/output/log/level"_json_pointer] = "error";

		Built b;
		b.state = std::make_shared<State>();
		b.state->init(in_args, true);
		b.state->set_max_threads(1);
		b.state->load_mesh();
		test::VarFormTestAccess::prepare(*b.state->variational_formulation);
		b.debug = test::VarFormTestAccess::debug_data(*b.state->variational_formulation);
		REQUIRE(b.debug.bases != nullptr);
		REQUIRE(b.debug.geometry_bases != nullptr);
		return b;
	}

	std::string column_mesh()
	{
		return std::string(POLYFEM_DATA_DIR) + "/quad_test/hex.HYBRID";
	}

	// n x n x n hexahedral grid of the unit cube, every vertex moved by the
	// smooth non-affine map (x + a y z, y + a x z, z + a x y) so that no
	// element is a parallelepiped and the geometric map is genuinely
	// trilinear; written as gmsh 2.2 ASCII to a scratch file.
	std::string write_skewed_hex_grid(const int n, const double a)
	{
		const std::filesystem::path path = std::filesystem::temp_directory_path() / ("rb23-skewed-hex-" + std::to_string(n) + ".msh");
		std::ofstream out(path);
		REQUIRE(out.is_open());
		const int nv = n + 1;
		const auto vid = [nv](const int i, const int j, const int k) { return 1 + i + nv * (j + nv * k); };
		out << "$MeshFormat\n2.2 0 8\n$EndMeshFormat\n$Nodes\n"
			<< nv * nv * nv << "\n";
		for (int k = 0; k < nv; ++k)
			for (int j = 0; j < nv; ++j)
				for (int i = 0; i < nv; ++i)
				{
					const double x = double(i) / n, y = double(j) / n, z = double(k) / n;
					out << vid(i, j, k) << " " << x + a * y * z << " " << y + a * x * z << " " << z + a * x * y << "\n";
				}
		out << "$EndNodes\n$Elements\n"
			<< n * n * n << "\n";
		int e = 1;
		for (int k = 0; k < n; ++k)
			for (int j = 0; j < n; ++j)
				for (int i = 0; i < n; ++i)
					out << e++ << " 5 2 0 0 "
						<< vid(i, j, k) << " " << vid(i + 1, j, k) << " " << vid(i + 1, j + 1, k) << " " << vid(i, j + 1, k) << " "
						<< vid(i, j, k + 1) << " " << vid(i + 1, j, k + 1) << " " << vid(i + 1, j + 1, k + 1) << " " << vid(i, j + 1, k + 1) << "\n";
		out << "$EndElements\n";
		return path.string();
	}

	// The invariant itself, on any hexahedral mesh: stored position = image of
	// the reference node; one position per global node.
	void check_node_bookkeeping(const Built &b, const int order)
	{
		Eigen::MatrixXd ref_nodes;
		autogen::q_nodes_3d(order, ref_nodes);
		std::map<int, Eigen::RowVector3d> seen;
		int checked = 0;
		for (size_t e = 0; e < b.debug.bases->size(); ++e)
		{
			const basis::ElementBases &bases = (*b.debug.bases)[e];
			REQUIRE(bases.bases.size() == size_t(ref_nodes.rows()));
			Eigen::MatrixXd mapped;
			(*b.debug.geometry_bases)[e].eval_geom_mapping(ref_nodes, mapped);
			for (size_t j = 0; j < bases.bases.size(); ++j)
			{
				REQUIRE(bases.bases[j].global().size() == 1);
				const auto &g = bases.bases[j].global().front();
				INFO("element " << e << " local node " << j);
				CHECK((mapped.row(j) - g.node).norm() < 1e-10);
				const auto it = seen.find(g.index);
				if (it == seen.end())
					seen[g.index] = g.node;
				else
					CHECK((it->second - g.node).norm() < 1e-10);
				++checked;
			}
		}
		CHECK(checked == int(ref_nodes.rows()) * int(b.debug.bases->size()));
		CHECK(int(seen.size()) == b.debug.n_bases);
	}

	// Nodal values of f at every global node (the positions the bases store).
	template <typename F>
	Eigen::VectorXd interpolate_nodal(const Built &b, F f)
	{
		Eigen::VectorXd u = Eigen::VectorXd::Constant(b.debug.n_bases, std::numeric_limits<double>::quiet_NaN());
		for (const basis::ElementBases &bases : *b.debug.bases)
			for (const basis::Basis &basis : bases.bases)
			{
				REQUIRE(basis.global().size() == 1);
				const auto &g = basis.global().front();
				u[g.index] = f(g.node);
			}
		REQUIRE(u.allFinite());
		return u;
	}

	// Maximum |u_h - f| over random reference points of every element,
	// evaluated through the element's own bases and geometric map.
	template <typename F>
	double max_interpolation_error(const Built &b, const Eigen::VectorXd &u, F f, const int n_samples, const unsigned seed)
	{
		std::mt19937 rng(seed);
		std::uniform_real_distribution<double> unif(0., 1.);
		Eigen::MatrixXd uv(n_samples, 3);
		for (int s = 0; s < n_samples; ++s)
			uv.row(s) << unif(rng), unif(rng), unif(rng);

		double err = 0;
		for (size_t e = 0; e < b.debug.bases->size(); ++e)
		{
			const basis::ElementBases &bases = (*b.debug.bases)[e];
			std::vector<assembler::AssemblyValues> vals;
			bases.evaluate_bases(uv, vals);
			Eigen::MatrixXd mapped;
			(*b.debug.geometry_bases)[e].eval_geom_mapping(uv, mapped);
			Eigen::VectorXd uh = Eigen::VectorXd::Zero(n_samples);
			for (size_t j = 0; j < bases.bases.size(); ++j)
				uh += vals[j].val * u[bases.bases[j].global().front().index];
			for (int s = 0; s < n_samples; ++s)
				err = std::max(err, std::abs(uh[s] - f(Eigen::RowVector3d(mapped.row(s)))));
		}
		return err;
	}

	// A random polynomial with every monomial x^a y^b z^d of degree a, b, d <= q
	// (tensor = true: the full Q_q, reproduced on axis-aligned elements) or of
	// total degree a + b + d <= q (P_q, reproduced on any trilinear element),
	// in the scaled coordinates x / scale (kept O(1) on the 20 x 20 x 100 column).
	struct RandomPolynomial
	{
		int q;
		bool tensor;
		Eigen::RowVector3d scale;
		std::vector<double> c; // c[(a * (q+1) + b) * (q+1) + d] for x^a y^b z^d

		RandomPolynomial(const int order, const bool tensor_product, const Eigen::RowVector3d &s, const unsigned seed)
			: q(order), tensor(tensor_product), scale(s), c((order + 1) * (order + 1) * (order + 1), 0.)
		{
			std::mt19937 rng(seed);
			std::uniform_real_distribution<double> unif(-1., 1.);
			for (int a = 0; a <= q; ++a)
				for (int bb = 0; bb <= q; ++bb)
					for (int d = 0; d <= q; ++d)
						if (tensor || a + bb + d <= q)
							c[(a * (q + 1) + bb) * (q + 1) + d] = unif(rng);
		}

		double operator()(const Eigen::RowVector3d &p) const
		{
			const double x = p[0] / scale[0], y = p[1] / scale[1], z = p[2] / scale[2];
			double val = 0;
			for (int a = 0; a <= q; ++a)
				for (int bb = 0; bb <= q; ++bb)
					for (int d = 0; d <= q; ++d)
						val += c[(a * (q + 1) + bb) * (q + 1) + d] * std::pow(x, a) * std::pow(y, bb) * std::pow(z, d);
			return val;
		}
	};

	// Inverse of the trilinear geometric map by Newton iteration (the grids
	// here are mildly skewed; the column is affine).
	Eigen::RowVector3d reference_coordinates(const basis::ElementBases &gbases, const Eigen::RowVector3d &x)
	{
		Eigen::MatrixXd uv = Eigen::MatrixXd::Constant(1, 3, 0.5);
		for (int it = 0; it < 30; ++it)
		{
			Eigen::MatrixXd mapped;
			std::vector<Eigen::MatrixXd> grads;
			gbases.eval_geom_mapping(uv, mapped);
			gbases.eval_geom_mapping_grads(uv, grads);
			const Eigen::RowVector3d r = mapped.row(0) - x;
			if (r.norm() < 1e-13)
				break;
			// grads[0](i, j) = d x_j / d uv_i in polyfem's convention, hence the transpose
			const Eigen::Vector3d step = grads[0].transpose().colPivHouseholderQr().solve(r.transpose());
			uv.row(0) -= step.transpose();
		}
		Eigen::MatrixXd mapped;
		gbases.eval_geom_mapping(uv, mapped);
		REQUIRE((mapped.row(0) - x).norm() < 1e-10);
		return uv.row(0);
	}

	// Values of the interpolant of u at physical points x from element e.
	Eigen::VectorXd evaluate_at(const Built &b, const int e, const Eigen::VectorXd &u, const Eigen::MatrixXd &x)
	{
		const basis::ElementBases &bases = (*b.debug.bases)[e];
		const basis::ElementBases &gbases = (*b.debug.geometry_bases)[e];
		Eigen::MatrixXd uv(x.rows(), 3);
		for (int s = 0; s < x.rows(); ++s)
			uv.row(s) = reference_coordinates(gbases, x.row(s));
		std::vector<assembler::AssemblyValues> vals;
		bases.evaluate_bases(uv, vals);
		Eigen::VectorXd uh = Eigen::VectorXd::Zero(x.rows());
		for (size_t j = 0; j < bases.bases.size(); ++j)
			uh += vals[j].val * u[bases.bases[j].global().front().index];
		return uh;
	}

	// Every interior face, with its two elements and physical sample points
	// strictly inside the face (from the first element's reference frame).
	struct SharedFace
	{
		int e0, e1;
		Eigen::MatrixXd points;
	};

	std::vector<SharedFace> shared_faces(const Built &b, const int n_samples, const unsigned seed)
	{
		const auto &mesh = dynamic_cast<const mesh::Mesh3D &>(*b.debug.mesh);
		std::mt19937 rng(seed);
		std::uniform_real_distribution<double> unif(0.05, 0.95);
		std::vector<SharedFace> faces;
		for (int f = 0; f < mesh.n_faces(); ++f)
		{
			if (mesh.is_boundary_face(f))
				continue;
			// the two cells around the face
			int e0 = -1, e1 = -1;
			for (int c = 0; c < mesh.n_cells() && e1 < 0; ++c)
				for (int lf = 0; lf < mesh.n_cell_faces(c); ++lf)
					if (mesh.get_index_from_element(c, lf, 0).face == f)
					{
						(e0 < 0 ? e0 : e1) = c;
						break;
					}
			REQUIRE(e0 >= 0);
			REQUIRE(e1 >= 0);
			// bilinear samples of the face from its four vertices
			Eigen::Matrix<double, 4, 3> v;
			auto index = mesh.get_index_from_element(e0, 0, 0);
			for (int lf = 0; lf < mesh.n_cell_faces(e0); ++lf)
			{
				index = mesh.get_index_from_element(e0, lf, 0);
				if (index.face == f)
					break;
			}
			REQUIRE(index.face == f);
			for (int k = 0; k < 4; ++k)
			{
				v.row(k) = mesh.point(index.vertex);
				index = mesh.next_around_face(index);
			}
			SharedFace sf{e0, e1, Eigen::MatrixXd(n_samples, 3)};
			for (int s = 0; s < n_samples; ++s)
			{
				const double a = unif(rng), c = unif(rng);
				sf.points.row(s) = (1 - a) * (1 - c) * v.row(0) + a * (1 - c) * v.row(1) + a * c * v.row(2) + (1 - a) * c * v.row(3);
			}
			faces.push_back(sf);
		}
		return faces;
	}
} // namespace

TEST_CASE("Hexahedral bases reproduce tensor-product polynomials of their order", "[rb23][hex_basis]")
{
	// Q_q contains every x^a y^b z^c with a, b, c <= q: the nodal interpolant
	// of such a polynomial must be exact. A basis stored at the wrong node
	// position gets the wrong nodal value and loses the reproduction (Q3 on
	// the column before RB-23: errors of order 1).
	const int order = GENERATE(1, 2, 3);
	CAPTURE(order);

	SECTION("public 4-hex column (axis-aligned elements): the full Q_q")
	{
		const Built b = build(column_mesh(), order);
		check_node_bookkeeping(b, order);
		const RandomPolynomial p(order, true, Eigen::RowVector3d(20, 20, 100), 7 + order);
		const Eigen::VectorXd u = interpolate_nodal(b, p);
		CHECK(max_interpolation_error(b, u, p, 40, 11) < 1e-10);
	}
	SECTION("2 x 2 x 2 skewed grid (trilinear elements): total degree q")
	{
		// Under a non-affine trilinear map the space still contains every
		// polynomial of total degree q (a product of k trilinear factors is
		// in Q_k), but not the full tensor space.
		const Built b = build(write_skewed_hex_grid(2, 0.15), order);
		check_node_bookkeeping(b, order);
		const RandomPolynomial p(order, false, Eigen::RowVector3d(1, 1, 1), 17 + order);
		const Eigen::VectorXd u = interpolate_nodal(b, p);
		CHECK(max_interpolation_error(b, u, p, 40, 13) < 1e-10);
	}
}

TEST_CASE("Hexahedral bases are continuous across shared faces", "[rb23][hex_basis]")
{
	// Random nodal values: the interpolant seen from the two elements of every
	// interior face must agree at interior points of the face. Before RB-23
	// the Q3 face-interior nodes were placed differently by the two elements
	// and the traces disagreed (a nonconforming space).
	const int order = GENERATE(2, 3);
	CAPTURE(order);
	const std::string mesh = GENERATE(as<std::string>{}, "column", "skewed");
	CAPTURE(mesh);
	const Built b = build(mesh == "column" ? column_mesh() : write_skewed_hex_grid(2, 0.15), order);

	std::mt19937 rng(23);
	std::uniform_real_distribution<double> unif(-1., 1.);
	Eigen::VectorXd u(b.debug.n_bases);
	for (int i = 0; i < u.size(); ++i)
		u[i] = unif(rng);

	const auto faces = shared_faces(b, 12, 29);
	REQUIRE(faces.size() == (mesh == "column" ? 3 : 12));
	double jump = 0;
	for (const SharedFace &f : faces)
	{
		const Eigen::VectorXd a = evaluate_at(b, f.e0, u, f.points);
		const Eigen::VectorXd c = evaluate_at(b, f.e1, u, f.points);
		jump = std::max(jump, (a - c).cwiseAbs().maxCoeff());
	}
	CHECK(jump < 1e-10);
}

TEST_CASE("Hexahedral interpolation converges at order q + 1", "[rb23][hex_basis]")
{
	// sup-norm nodal interpolation error of a smooth function on the skewed
	// unit-cube grids at h = 1/2 and h = 1/4: O(h^(q+1)). A permuted node
	// layout at Q3 destroys the rate (it is not even consistent).
	const int order = GENERATE(1, 2, 3);
	CAPTURE(order);
	const auto f = [](const Eigen::RowVector3d &x) { return std::sin(1.3 * x[0] + 0.4) * std::cos(0.9 * x[1] - 0.2) * std::exp(0.5 * x[2]); };

	std::array<double, 2> err{};
	for (int level = 0; level < 2; ++level)
	{
		const Built b = build(write_skewed_hex_grid(2 << level, 0.15), order);
		const Eigen::VectorXd u = interpolate_nodal(b, f);
		err[level] = max_interpolation_error(b, u, f, 40, 31 + level);
	}
	// measured 2026-09-20 (RB-23 record): 1.90 / 2.90 / 3.87 for Q1 / Q2 / Q3,
	// err[1] = 3.2e-2 / 4.4e-4 / 1.2e-5
	const double rate = std::log2(err[0] / err[1]);
	CAPTURE(err[0], err[1], rate);
	CHECK(rate > order + 0.5);
	CHECK(err[1] < 5e-2 / std::pow(8., order - 1));
}

TEST_CASE("Mixed per-element hexahedral orders are refused with a named error", "[rb23][hex_basis][input_validation]")
{
	// A hexahedron of higher order than a neighbour gets placeholder node ids
	// for the shared edges/faces; the interface stitching that resolves them
	// exists for simplices only (the hex branch is a TODO). Before RB-23 the
	// release build initialised the bases from node_position(< 0): a
	// bad_alloc on a Q1/Q2 mix, a NaN solve on a Q2/Q3 mix.
	const std::string orders = GENERATE(as<std::string>{}, "1\n2\n1\n1\n", "2\n3\n2\n2\n", "3\n3\n1\n1\n");
	CAPTURE(orders);
	const std::filesystem::path path = std::filesystem::temp_directory_path() / "rb23-mixed-hex-orders.txt";
	{
		std::ofstream out(path);
		REQUIRE(out.is_open());
		out << orders;
	}

	State state;
	json in_args;
	in_args["/geometry/0/mesh"_json_pointer] = column_mesh();
	in_args["/materials/type"_json_pointer] = "NeoHookean";
	in_args["/materials/E"_json_pointer] = 1e5;
	in_args["/materials/nu"_json_pointer] = 0.3;
	in_args["/materials/rho"_json_pointer] = 1e3;
	in_args["/space/discr_order"_json_pointer] = path.string();
	in_args["/contact/enabled"_json_pointer] = false;
	in_args["/time/time_steps"_json_pointer] = 1;
	in_args["/time/tend"_json_pointer] = 1;
	in_args["/output/log/level"_json_pointer] = "error";
	state.init(in_args, true);
	state.set_max_threads(1);
	state.load_mesh();
	REQUIRE_THROWS_WITH(
		test::VarFormTestAccess::prepare(*state.variational_formulation),
		ContainsSubstring("Mixed per-element hexahedral orders are not supported") && ContainsSubstring("hexahedron 1 (order"));

	// uniform orders from a per-element file are still accepted
	{
		std::ofstream out(path);
		out << "3\n3\n3\n3\n";
	}
	const Built b = build(column_mesh(), path.string());
	CHECK(b.debug.n_bases == 4 * 64 - 3 * 16);
}

namespace
{
	void expect_basis_error(const std::string &mesh, const int order, const std::string &basis_type, const std::string &text)
	{
		State state;
		json in_args;
		in_args["/geometry/0/mesh"_json_pointer] = mesh;
		in_args["/materials/type"_json_pointer] = "NeoHookean";
		in_args["/materials/E"_json_pointer] = 1e5;
		in_args["/materials/nu"_json_pointer] = 0.3;
		in_args["/materials/rho"_json_pointer] = 1e3;
		in_args["/space/discr_order"_json_pointer] = order;
		in_args["/space/basis_type"_json_pointer] = basis_type;
		in_args["/contact/enabled"_json_pointer] = false;
		in_args["/time/time_steps"_json_pointer] = 1;
		in_args["/time/tend"_json_pointer] = 1;
		in_args["/output/log/level"_json_pointer] = "error";
		state.init(in_args, true);
		state.set_max_threads(1);
		state.load_mesh();
		REQUIRE_THROWS_WITH(test::VarFormTestAccess::prepare(*state.variational_formulation), ContainsSubstring(text));
	}
} // namespace

TEST_CASE("Tensor-product orders without a basis table are refused with a named error", "[rb23][hex_basis][input_validation]")
{
	// autogen carries Q0-Q3 and serendipity Q2 only (MAX_Q_BASES = 3). A Q4+
	// hexahedron or quadrilateral used to segfault on the empty node table
	// (exit 139); serendipity at order 3 ran 32 node ids against the 20-node
	// table with exit 0 (a silent garbage solve), at order 1 it died with a
	// misleading "element is flipped".
	const std::string quad = std::string(POLYFEM_DATA_DIR) + "/quad_test/quad.obj";
	SECTION("hexahedra")
	{
		expect_basis_error(column_mesh(), 4, "Lagrange", "Q4 hexahedral bases are not available");
		expect_basis_error(column_mesh(), 5, "Lagrange", "Q5 hexahedral bases are not available");
		expect_basis_error(column_mesh(), 1, "Serendipity", "Serendipity hexahedral bases exist for discr_order 2 only");
		expect_basis_error(column_mesh(), 3, "Serendipity", "Serendipity hexahedral bases exist for discr_order 2 only");
	}
	SECTION("quadrilaterals (the same guard in LagrangeBasis2d)")
	{
		expect_basis_error(quad, 4, "Lagrange", "Q4 quadrilateral bases are not available");
		expect_basis_error(quad, 3, "Serendipity", "Serendipity quadrilateral bases exist for discr_order 2 only");
	}
	SECTION("the supported orders still build")
	{
		CHECK(build(column_mesh(), 3).debug.n_bases == 4 * 64 - 3 * 16);
		CHECK(build(column_mesh(), 2, "Serendipity").debug.n_bases == 20 + 36);
	}
}
