#pragma once
#include <vector>
#include <GdstkUtils.h>
#include <Warping.h>
#include <fstream>
#include "TriangulationUtils.h"


namespace RasterizationStep
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


	// Représente une case contenant des triangles la touchant pour la rastérisation
	struct Tile
	{
		uint offset = 0;
		uint temp_count = 0;
		uint count = 0;

		float2 min_pos = { 0, 0 };
		float2 max_pos = { 0, 0 };
	};

	// Appliquer la déformation à une triangulation
	void Warping(Triangulation dtriangulation, std::vector<Warping::Boxes>& src_dst);

	// Rastérise une triangulation
	unsigned char* Rasterization(Triangulation& dtriangulation, double scale, uint2 img_dim);

	// Sauvegarde "img" en .bmp
	void SaveToBmp(const std::string& filename, int width, int height,
		unsigned char* img);
};