#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>

#include <h5pp/h5pp.h>

#include <paraviewo/HDF5VTUWriter.hpp>

#include <filesystem>

TEST_CASE("HDF5", "[hdf5]")
{
	using MatrixXl = Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>;

	const std::string hdf5_file = std::string(POLYFEM_DATA_DIR) + "/test.hdf5";
	h5pp::File file(hdf5_file, h5pp::FileAccess::READONLY);
	std::string json_string = file.readDataset<std::string>("json");

	nlohmann::json in_args = nlohmann::json::parse(json_string);
	in_args["root_path"] = hdf5_file;

	std::vector<std::string> names = file.findGroups("", "/meshes");
	CHECK(names.size() == 2);
	CHECK(names[0] == "hdf5_0");
	CHECK(names[1] == "hdf5_1");
	std::vector<Eigen::MatrixXi> cells(names.size());
	std::vector<Eigen::MatrixXd> vertices(names.size());

	for (int i = 0; i < names.size(); ++i)
	{
		const std::string &name = names[i];
		cells[i] = file.readDataset<MatrixXl>("/meshes/" + name + "/c").cast<int>();
		vertices[i] = file.readDataset<Eigen::MatrixXd>("/meshes/" + name + "/v");
	}
}
TEST_CASE("HDF5 VTU fields are chunked in dense row blocks", "[hdf5]")
{
	// h5pp's default chunk guess is square (256 x 256): on an N x 3 field
	// every compressed chunk held 85x more padding than data, so a written
	// 2.4M-point frame took minutes and reading it back ~80 s. Fields and
	// points must be chunked by rows spanning every column.
	const int n = 100000;
	Eigen::MatrixXd points = Eigen::MatrixXd::Random(n, 3);
	Eigen::MatrixXi tets(n / 4, 4);
	for (int i = 0; i < tets.rows(); ++i)
		tets.row(i) << 4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3;
	const Eigen::MatrixXd vector_field = Eigen::MatrixXd::Random(n, 3);
	const Eigen::MatrixXd scalar_field = Eigen::MatrixXd::Random(n, 1);

	const std::string path =
		(std::filesystem::temp_directory_path() / "polyfem_hdf5_row_chunks.hdf").string();
	{
		paraviewo::HDF5VTUWriter writer;
		writer.add_field("vector", vector_field);
		writer.add_field("scalar", scalar_field);
		REQUIRE(writer.write_mesh(path, points, tets, paraviewo::CellType::Tetrahedron));
	}

	h5pp::File file(path, h5pp::FileAccess::READONLY);
	for (const std::string dset : {"/VTKHDF/Points", "/VTKHDF/PointData/vector"})
	{
		const auto info = file.getDatasetInfo(dset);
		REQUIRE(info.dsetChunk.has_value());
		const std::vector<hsize_t> &chunk = info.dsetChunk.value();
		REQUIRE(chunk.size() == 2);
		CHECK(chunk[1] == 3);
		CHECK(chunk[0] >= 4096);
		CHECK(chunk[0] <= static_cast<hsize_t>(n));
	}
	CHECK(file.readDataset<Eigen::MatrixXd>("/VTKHDF/PointData/vector").isApprox(vector_field));
	CHECK(file.readDataset<Eigen::MatrixXd>("/VTKHDF/Points").isApprox(points));
	CHECK(file.readDataset<Eigen::VectorXd>("/VTKHDF/PointData/scalar").isApprox(scalar_field.col(0)));
	std::filesystem::remove(path);
}
