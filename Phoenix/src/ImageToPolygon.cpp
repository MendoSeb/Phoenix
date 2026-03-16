#include "ImageToPolygons.h"
#include <clipper2/clipper.core.h>
#include <Clipper2Utils.h>
#include <cuda_runtime.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

using namespace Clipper2Lib;


void ImageToPolygons::ConvertBMPToPolygons(const char* filepath)
{
	/// load image
	int width, height, pixel_components;
	unsigned char* img = stbi_load(filepath, &width, &height, &pixel_components, 0);

	int seuil = 125;
	const int thread_x = 5;
	const int thread_y = 4;

	PathsD polys[thread_x][thread_y];
	int x_grid_size = std::ceil((float)width / thread_x);
	int y_grid_size = std::ceil((float)height / thread_y);

	/// sort square polygons into grid
	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++)
		{
			int pixel_index = ((height - 1 - y) * width + x) * pixel_components;

			if (img[pixel_index] < seuil)
			{
				int x_grid_index = x / x_grid_size;
				int y_grid_index = y / y_grid_size;

				polys[x_grid_index][y_grid_index].push_back( 
					PathD{ PointD(x, y), PointD(x + 1, y), PointD(x + 1, y + 1), PointD(x, y + 1) } );
			}
		}
	}
	
	/// make union and save result as .gds
	std::thread threads[thread_x][thread_y];
	PathsD final_polys;

	auto thread_union = [&](int x, int y) 
	{
		PolyTreeD union_polytree;
		Clipper2Utils::MakeUnion(polys[x][y], union_polytree);
		polys[x][y] = PolyTreeToPathsD(union_polytree);
	};

	for (int x = 0; x < thread_x; x++)
		for (int y = 0; y < thread_y; y++)
			threads[x][y] = std::thread(thread_union, x, y);

	for (int x = 0; x < thread_x; x++) {
		for (int y = 0; y < thread_y; y++)
		{
			threads[x][y].join();
			final_polys.insert(final_polys.end(), polys[x][y].begin(), polys[x][y].end());
		}
	}

	PolyTreeD union_polytree;
	Clipper2Utils::MakeUnion(final_polys, union_polytree);
	
	Library lib = {};
	Clipper2Utils::ConvertPolyTreeDToGdsiiPath(union_polytree, lib);
	GdstkUtils::SaveToGdsii(lib, "C:/Users/PC/Desktop/poc/test.gds", false);

	delete[] img;
}