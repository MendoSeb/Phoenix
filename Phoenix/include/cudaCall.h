#pragma once
#include <vector>
#include <GdstkUtils.h>
#include <Warping.h>
#include <fstream>
#include <Utilities.h>


#define CUDA_CHECK(call)                                                      \
    do {                                                                      \
        cudaError_t error = call;                                             \
        if (error != cudaSuccess) {                                           \
            std::stringstream ss;                                             \
            ss << "CUDA Error: " << cudaGetErrorString(error) << " in "       \
               << __FILE__ << " at line " << __LINE__;                        \
            throw std::runtime_error(ss.str());                               \
        }                                                                     \
    } while (0)                                                                \


namespace CudaCall
{
	typedef unsigned int uint;

	#pragma pack(push, 1)
	struct BitmapFileHeader {
		uint16_t file_type{ 0x4D42 }; // 'BM'
		uint32_t file_size{ 0 };
		uint16_t reserved1{ 0 };
		uint16_t reserved2{ 0 };
		uint32_t offset_data{ 0 };
	};

	struct BitmapInfoHeader {
		uint32_t size{ 0 };
		int32_t width{ 0 };
		int32_t height{ 0 };
		uint16_t planes{ 1 };
		uint16_t bit_count{ 0 };
		uint32_t compression{ 0 };
		uint32_t size_image{ 0 };
		int32_t x_pixels_per_meter{ 0 };
		int32_t y_pixels_per_meter{ 0 };
		uint32_t colors_used{ 0 };
		uint32_t colors_important{ 0 };
	};
	#pragma pack(pop)


	struct Tile
	{
		uint offset = 0;
		uint temp_count = 0;
		uint count = 0;

		float2 min_pos = { 0, 0 };
		float2 max_pos = { 0, 0 };
	};


	struct RasterizationData
	{
		float2* dv = nullptr;
		uint3* dt = nullptr;
		Tile* dtile = nullptr;

		int nb_vertices = 0;
		int nb_triangles = 0;
		uint2 nb_tiles { 0, 0 };
		uint2 img_dim { 0, 0 };
	};

	void warping(
		float2* dv,
		uint3* dt,
		uint nb_vertices,
		uint nb_triangles,
		std::vector<Warping::Boxes>& src_dst
	);

	unsigned char* rasterization(Utils::Triangulation& dtriangulation, double scale, uint2 img_dim);

	void saveToBmp(const std::string& filename, int width, int height,
		unsigned char* hostData);

	void saveToTiff(unsigned char* img, uint2 img_dim);
};