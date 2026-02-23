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

	void warping(
		std::pair<std::pair<uint2*, uint3*>, uint2>& tris,
		std::vector<Warping::Boxes>& src_dst
	);

	void rasterization(std::pair<std::pair<uint2*, uint3*>, uint2>& tris, float& scale);

	void saveToBmp(const std::string& filename, int width, int height,
		unsigned char* hostData);

	void saveToTiff(unsigned char* img, uint2 img_dim);
};