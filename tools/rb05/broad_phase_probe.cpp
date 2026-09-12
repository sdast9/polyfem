// RB-05 broad-phase resource probe.
//
// Builds the swept candidate set of one trial step on a synthetic two-sheet
// mesh with the effective IPC toolkit and reports the intermediate buffers
// the default hash grid allocates before any count is logged: the
// (box, cell) items, the pair emissions before deduplication, the final
// candidates, wall time and the process maximum resident set. An exact
// pre-allocation estimate of the item and emission counts is computed
// independently from the boxes and compared with the instrumented grid.
//
// Bounded by construction: a configuration whose estimated items exceed
// --max-items is reported with its estimate only and never built.
//
// Usage: broad_phase_probe --n 20 --movers 1 --disp 5 [--uniform]
//        [--dhat 1e-3] [--gap .05] [--method hash_grid|brute_force|bvh|spatial_hash|sweep_and_prune]
//        [--max-items 2e8] [--estimate-only] [--rlimit-data-mb 512] [--json out.json]
//        [--budget-cell-items N] [--budget-emissions N]   (toolkit BroadPhaseBudget, 0 = unlimited)
#include <ipc/broad_phase/aabb.hpp>
#include <ipc/broad_phase/create_broad_phase.hpp>
#include <ipc/broad_phase/hash_grid.hpp>
#include <ipc/candidates/candidates.hpp>
#include <ipc/collision_mesh.hpp>

#include <igl/edges.h>
#include <igl/median.h>
#include <nlohmann/json.hpp>

#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <new>
#include <string>
#include <vector>

using json = nlohmann::json;

namespace
{
	long max_rss_bytes()
	{
		rusage usage{};
		getrusage(RUSAGE_SELF, &usage);
		return usage.ru_maxrss; // bytes on macOS, kilobytes on Linux
	}

	// Exposes the item vectors the toolkit keeps protected.
	class InstrumentedHashGrid : public ipc::HashGrid
	{
	public:
		json items() const
		{
			return {{"vertex", vertex_items.size()}, {"edge", edge_items.size()}, {"face", face_items.size()}, {"total", vertex_items.size() + edge_items.size() + face_items.size()}, {"bytes", sizeof(ipc::HashItem) * (vertex_items.size() + edge_items.size() + face_items.size())}};
		}
		// Pair emissions of the enumeration loops before the collision
		// filter, the AABB test and the unique pass: for one sorted item set
		// every pair inside a key run, for two sets every cross pair.
		static size_t self_emissions(const std::vector<ipc::HashItem> &items)
		{
			size_t total = 0;
			for (size_t i = 0; i < items.size();)
			{
				size_t j = i;
				while (j < items.size() && items[j].key == items[i].key)
					++j;
				const size_t n = j - i;
				total += n * (n - 1) / 2;
				i = j;
			}
			return total;
		}
		static size_t cross_emissions(const std::vector<ipc::HashItem> &a, const std::vector<ipc::HashItem> &b)
		{
			size_t total = 0, i = 0, j = 0;
			while (i < a.size() && j < b.size())
			{
				if (a[i].key < b[j].key)
					++i;
				else if (b[j].key < a[i].key)
					++j;
				else
				{
					const long key = a[i].key;
					size_t na = 0, nb = 0;
					while (i < a.size() && a[i].key == key)
						++i, ++na;
					while (j < b.size() && b[j].key == key)
						++j, ++nb;
					total += na * nb;
				}
			}
			return total;
		}
		json emissions() const
		{
			if (dim == 2)
				return {{"edge_vertex", cross_emissions(edge_items, vertex_items)}};
			return {{"edge_edge", self_emissions(edge_items)}, {"face_vertex", cross_emissions(face_items, vertex_items)}};
		}
		json grid() const
		{
			return {{"cell_size", cell_size()}, {"size", {grid_size()[0], grid_size()[1], grid_size()[2]}}, {"cells", double(grid_size()[0]) * grid_size()[1] * grid_size()[2]}};
		}
	};

	// Independent replication of HashGrid::build's cell size, grid and
	// per-box cell coverage, computed from the boxes alone (no items).
	struct Estimate
	{
		double cell_size = 0;
		Eigen::Array3i grid = Eigen::Array3i::Ones();
		size_t vertex_items = 0, edge_items = 0, face_items = 0;
		size_t total() const { return vertex_items + edge_items + face_items; }
	};

	Estimate estimate_hash_grid(const Eigen::MatrixXd &V0, const Eigen::MatrixXd &V1, const Eigen::MatrixXi &E, const Eigen::MatrixXi &F, const double inflation)
	{
		const int dim = V0.cols();
		ipc::AABBs vb, eb, fb;
		ipc::build_vertex_boxes(V0, V1, vb, inflation);
		ipc::build_edge_boxes(vb, E, eb);
		ipc::build_face_boxes(vb, F, fb);

		Eigen::Array3d domain_min = Eigen::Array3d::Constant(std::numeric_limits<double>::max());
		Eigen::Array3d domain_max = Eigen::Array3d::Constant(std::numeric_limits<double>::lowest());
		for (const auto &b : vb)
		{
			domain_min = domain_min.min(b.min);
			domain_max = domain_max.max(b.max);
		}
		const ipc::AABBs &sizing = E.rows() > 0 ? eb : vb;
		Eigen::VectorXd extents(sizing.size());
		for (size_t i = 0; i < sizing.size(); ++i)
			extents(i) = (sizing[i].max - sizing[i].min).maxCoeff();
		Estimate est;
		igl::median(extents, est.cell_size);
		if (est.cell_size <= 0)
			est.cell_size = std::numeric_limits<double>::max();
		est.grid = ((domain_max - domain_min) / est.cell_size).ceil().cast<int>().max(1);

		auto cells = [&](const ipc::AABB &b) -> size_t {
			Eigen::Array3i lo = ((b.min - domain_min) / est.cell_size).cast<int>();
			Eigen::Array3i hi = ((b.max - domain_min) / est.cell_size).cast<int>();
			lo = lo.max(0).min(est.grid - 1);
			hi = hi.max(0).min(est.grid - 1);
			size_t n = size_t(hi.x() - lo.x() + 1) * size_t(hi.y() - lo.y() + 1);
			if (dim == 3)
				n *= size_t(hi.z() - lo.z() + 1);
			return n;
		};
		for (const auto &b : vb)
			est.vertex_items += cells(b);
		for (const auto &b : eb)
			est.edge_items += cells(b);
		for (const auto &b : fb)
			est.face_items += cells(b);
		return est;
	}

	// Two N×N triangulated sheets over [0,1]², the lower one at z = 0 and the
	// upper one at z = gap. Trial displacement: the first `movers` vertices of
	// the upper sheet (or all of them with --uniform) move by disp along
	// (1,1,1)/√3, the diagonal that maximizes swept-box cell coverage.
	void make_sheets(const int n, const double gap, Eigen::MatrixXd &V, Eigen::MatrixXi &F)
	{
		const int per_sheet = n * n;
		V.resize(2 * per_sheet, 3);
		F.resize(2 * 2 * (n - 1) * (n - 1), 3);
		int f = 0;
		for (int sheet = 0; sheet < 2; ++sheet)
		{
			const int offset = sheet * per_sheet;
			for (int i = 0; i < n; ++i)
				for (int j = 0; j < n; ++j)
					V.row(offset + i * n + j) << double(i) / (n - 1), double(j) / (n - 1), sheet * gap;
			for (int i = 0; i + 1 < n; ++i)
				for (int j = 0; j + 1 < n; ++j)
				{
					const int a = offset + i * n + j, b = a + 1, c = a + n, d = c + 1;
					F.row(f++) << a, b, d;
					F.row(f++) << a, d, c;
				}
		}
	}

	double arg_double(const std::vector<std::string> &args, const std::string &key, const double fallback)
	{
		for (size_t i = 0; i + 1 < args.size(); ++i)
			if (args[i] == key)
				return std::stod(args[i + 1]);
		return fallback;
	}
	std::string arg_string(const std::vector<std::string> &args, const std::string &key, const std::string &fallback)
	{
		for (size_t i = 0; i + 1 < args.size(); ++i)
			if (args[i] == key)
				return args[i + 1];
		return fallback;
	}
	bool arg_flag(const std::vector<std::string> &args, const std::string &key)
	{
		return std::find(args.begin(), args.end(), key) != args.end();
	}
} // namespace

int main(int argc, char **argv)
{
	const std::vector<std::string> args(argv + 1, argv + argc);
	const int n = int(arg_double(args, "--n", 20));
	const int movers = int(arg_double(args, "--movers", 1));
	const double disp = arg_double(args, "--disp", 0.);
	const double dhat = arg_double(args, "--dhat", 1e-3);
	const double gap = arg_double(args, "--gap", .05);
	const double max_items = arg_double(args, "--max-items", 2e8);
	const double rlimit_mb = arg_double(args, "--rlimit-data-mb", 0.);
	const double budget_cell_items = arg_double(args, "--budget-cell-items", 0.);
	const double budget_emissions = arg_double(args, "--budget-emissions", 0.);
	const bool uniform = arg_flag(args, "--uniform");
	const bool spread = arg_flag(args, "--spread"); // movers evenly spread over the upper sheet instead of adjacent
	const bool estimate_only = arg_flag(args, "--estimate-only");
	const std::string method = arg_string(args, "--method", "hash_grid");
	const std::string out = arg_string(args, "--json", "");

	Eigen::MatrixXd V0;
	Eigen::MatrixXi F, E;
	make_sheets(n, gap, V0, F);
	igl::edges(F, E);
	Eigen::MatrixXd V1 = V0;
	const Eigen::RowVector3d d = disp * Eigen::RowVector3d(1, 1, 1) / std::sqrt(3.);
	const int per_sheet = n * n;
	const int moving = uniform ? per_sheet : std::min(movers, per_sheet);
	for (int i = 0; i < moving; ++i)
		V1.row(per_sheet + (spread ? (i * per_sheet) / moving : i)) += d;

	json report = {
		{"n", n}, {"vertices", V0.rows()}, {"edges", E.rows()}, {"faces", F.rows()}, {"movers", moving}, {"uniform", uniform}, {"spread", spread}, {"disp", disp}, {"dhat", dhat}, {"inflation_radius", dhat / 2}, {"gap", gap}, {"method", method}, {"trial_linf", (V1 - V0).lpNorm<Eigen::Infinity>()}, {"budget", {{"max_cell_items", size_t(budget_cell_items)}, {"max_candidate_emissions", size_t(budget_emissions)}}}};

	const Estimate est = estimate_hash_grid(V0, V1, E, F, dhat / 2);
	report["estimate"] = {{"cell_size", est.cell_size}, {"grid", {est.grid[0], est.grid[1], est.grid[2]}}, {"cells", double(est.grid[0]) * est.grid[1] * est.grid[2]}, {"vertex_items", est.vertex_items}, {"edge_items", est.edge_items}, {"face_items", est.face_items}, {"total_items", est.total()}, {"item_bytes", est.total() * sizeof(ipc::HashItem)}};
	report["built"] = false;

	if (estimate_only || double(est.total()) > max_items)
	{
		report["skipped"] = estimate_only ? "estimate only" : "estimated items exceed --max-items; not built (bounded probe)";
	}
	else
	{
		if (rlimit_mb > 0)
		{
			rlimit limit{};
			limit.rlim_cur = limit.rlim_max = rlim_t(rlimit_mb * 1024 * 1024);
			report["rlimit_data_set"] = setrlimit(RLIMIT_DATA, &limit) == 0;
		}
		ipc::CollisionMesh mesh(V0, E, F);
		std::shared_ptr<ipc::BroadPhase> bp;
		InstrumentedHashGrid *grid = nullptr;
		if (method == "hash_grid")
		{
			auto g = std::make_shared<InstrumentedHashGrid>();
			grid = g.get();
			bp = g;
		}
		else if (method == "brute_force")
			bp = ipc::create_broad_phase(ipc::BroadPhaseMethod::BRUTE_FORCE);
		else if (method == "bvh")
			bp = ipc::create_broad_phase(ipc::BroadPhaseMethod::LBVH);
		else if (method == "spatial_hash")
			bp = ipc::create_broad_phase(ipc::BroadPhaseMethod::SPATIAL_HASH);
		else if (method == "sweep_and_prune")
			bp = ipc::create_broad_phase(ipc::BroadPhaseMethod::SWEEP_AND_PRUNE);
		else
		{
			std::cerr << "unknown method " << method << "\n";
			return 2;
		}

		bp->budget.max_cell_items = size_t(budget_cell_items);
		bp->budget.max_candidate_emissions = size_t(budget_emissions);

		ipc::Candidates candidates;
		const long rss_before = max_rss_bytes();
		const auto t0 = std::chrono::steady_clock::now();
		try
		{
			candidates.build(mesh, V0, V1, dhat / 2, bp.get());
			report["built"] = true;
		}
		catch (const std::bad_alloc &e)
		{
			report["exception"] = {{"type", "std::bad_alloc"}, {"what", e.what()}};
		}
		catch (const ipc::BroadPhaseBudgetExceeded &e)
		{
			report["exception"] = {{"type", "ipc::BroadPhaseBudgetExceeded"}, {"what", e.what()}, {"quantity", e.quantity}, {"requested", e.requested}, {"limit", e.limit}};
			report["candidates_after_failure"] = candidates.size(); // must be 0: no partial set
		}
		catch (const std::exception &e)
		{
			report["exception"] = {{"type", typeid(e).name()}, {"what", e.what()}};
		}
		const auto t1 = std::chrono::steady_clock::now();
		report["build_seconds"] = std::chrono::duration<double>(t1 - t0).count();
		report["max_rss_bytes_before"] = rss_before;
		report["max_rss_bytes_after"] = max_rss_bytes();
		report["candidates"] = {{"total", candidates.size()}, {"edge_vertex", candidates.ev_candidates.size()}, {"edge_edge", candidates.ee_candidates.size()}, {"face_vertex", candidates.fv_candidates.size()}, {"bytes", sizeof(ipc::EdgeEdgeCandidate) * candidates.ee_candidates.size() + sizeof(ipc::FaceVertexCandidate) * candidates.fv_candidates.size() + sizeof(ipc::EdgeVertexCandidate) * candidates.ev_candidates.size()}};
		const auto &stats = bp->build_statistics();
		report["toolkit_statistics"] = {{"measured", stats.measured}, {"cell_items", stats.cell_items}, {"candidate_emissions", stats.candidate_emissions}, {"cell_size", stats.cell_size}, {"grid", {stats.grid_size[0], stats.grid_size[1], stats.grid_size[2]}}};
		if (grid != nullptr)
		{
			// The grid still holds the items of the last build (the toolkit
			// clears them on the next build), so the counts are observable.
			report["measured"] = {{"grid", grid->grid()}, {"items", grid->items()}, {"emissions", grid->emissions()}};
			report["estimate_matches_items"] = grid->items()["vertex"] == est.vertex_items && grid->items()["edge"] == est.edge_items && grid->items()["face"] == est.face_items;
			report["estimate_matches_grid"] = grid->grid()["size"] == json({est.grid[0], est.grid[1], est.grid[2]}) && grid->grid()["cell_size"].get<double>() == est.cell_size;
			// The toolkit's own emission count (budget enabled) against the
			// probe's independent run-length count over the same items.
			if (report["built"].get<bool>() && bp->budget.enabled())
			{
				size_t independent = 0;
				const json independent_emissions = grid->emissions(); // keep the object alive while iterating
				for (const auto &[key, value] : independent_emissions.items())
					independent += value.get<size_t>();
				report["toolkit_emissions_match_independent"] = stats.candidate_emissions == independent;
			}
		}
	}

	const std::string text = report.dump(2);
	if (!out.empty())
		std::ofstream(out) << text << "\n";
	std::cout << text << std::endl;
	return report["built"].get<bool>() || report.contains("skipped") ? 0 : 1;
}
