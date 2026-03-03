#pragma once
#include <vector>
#include <GdstkUtils.h>
#include <Warping.h>
#include <fstream>


namespace CudaCall
{
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


	struct BVHNode
	{
		size_t id = (size_t)-1;

		float2 min_pos { FLT_MAX, FLT_MAX };
		float2 max_pos { FLT_MIN, FLT_MIN };

		size_t offset = (size_t)0;
		size_t count = (size_t)0;
		size_t temp_count = (size_t)0;
		int childs[4] = { -1, -1, -1, -1 };

		bool isLeaf = false;
	};


	struct Tile
	{
		uint offset = 0;
		uint temp_count = 0;
		uint count = 0;

		float2 min_pos = { 0, 0 };
		float2 max_pos = { 0, 0 };
	};


	void warping(
		std::pair<std::pair<float2*, uint3*>, uint2>& tris,
		std::vector<Warping::Boxes>& src_dst
	);

	void rasterization(
		std::pair<std::pair<float2*, uint3*>, uint2>& tris,
		std::pair<BVHNode*, int*>& bvh,
		int& depth,
		int nb_indices
	);

	std::pair<std::pair<BVHNode*, int*>, int> bvhV1(std::pair<std::pair<float2*, uint3*>, uint2>& tris, int& depth);

	void rasterizationV2(std::pair<std::pair<float2*, uint3*>, uint2>& tris);

	void rasterizationV3(std::pair<std::pair<float2*, uint3*>, uint2>& tris, double scale);

	void saveToBmp(const std::string& filename, int width, int height,
		unsigned char* hostData);

	void saveToTiff(unsigned char* img, uint2 img_dim);
};