set(SOURCES
	BuildInfo.hpp
	DiagnosticSchemas.hpp
	Checkpoint.cpp
	Checkpoint.hpp
	ForkCheckpoint.hpp
	HDF5RowChunks.hpp
	Evaluator.cpp
	Evaluator.hpp
	InputLoader.hpp
	MatrixIO.cpp
	MatrixIO.hpp
	MshReader.cpp
	MshReader.hpp
	MshWriter.cpp
	MshWriter.hpp
	OBJReader.cpp
	OBJReader.hpp
	OBJWriter.cpp
	OBJWriter.hpp
	OutData.cpp
	OutData.hpp
	OutputData.hpp
	OutputWriter.cpp
	OutputWriter.hpp
	OutStatsData.hpp
	RunManifest.cpp
	RunManifest.hpp
	ResourceIO.cpp
	ResourceIO.hpp
	SolverCSVWriter.cpp
	SolverCSVWriter.hpp
	YamlToJson.cpp
	YamlToJson.hpp
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "Source Files" FILES ${SOURCES})
target_sources(polyfem PRIVATE ${SOURCES})
