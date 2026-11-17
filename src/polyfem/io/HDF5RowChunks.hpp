#pragma once
#include <h5pp/h5pp.h>
#include <algorithm>
#include <optional>
#include <vector>
namespace polyfem::io
{
	/// Chunk dimensions for a dataset h5pp would store chunked: blocks of
	/// whole rows of about 1 MiB. h5pp's own guess is a square (256 x 256
	/// for any 2D array over 512 kB), which is allocated in full even for
	/// an N x 1 column: a restart state of 652,440 DOFs stored 1.34 GB per
	/// 5.2 MB vector, 4.0 GB per file. Smaller data keeps h5pp's compact
	/// or contiguous layout (std::nullopt); an existing dataset keeps its
	/// own layout.
	template <typename Mat>
	std::optional<std::vector<hsize_t>> row_block_chunks(const Mat &mat)
	{
		std::vector<hsize_t> dims = h5pp::util::getDimensions(mat);
		const hsize_t bytes = hsize_t(mat.size()) * sizeof(typename Mat::Scalar);
		if (dims.empty() || bytes < h5pp::constants::maxSizeContiguous)
			return std::nullopt;

		hsize_t row_bytes = sizeof(typename Mat::Scalar);
		for (size_t i = 1; i < dims.size(); ++i)
			row_bytes *= dims[i];
		dims[0] = std::clamp<hsize_t>((hsize_t(1) << 20) / row_bytes, 1, dims[0]);
		return dims;
	}
} // namespace polyfem::io
