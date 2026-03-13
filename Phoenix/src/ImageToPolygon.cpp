#include "ImageToPolygons.h"
#include <clipper2/clipper.core.h>
#include <Clipper2Utils.h>
#include <cuda_runtime.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

using namespace Clipper2Lib;


void ImageToPolygons::ExtractContours(const char* filepath)
{
	int width, height, pixel_components;
	unsigned char* img = stbi_load(filepath, &width, &height, &pixel_components, 0);
	int nb_pixels = width * height;
	printf("width: %i, height: %i, pixel_bits: %i\n", width, height, pixel_components);

	ThresholdImage(img, width, height, pixel_components);

	unsigned char* edge_img = new unsigned char[width * height](255); // parenthèse met les valeurs à 0

	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {

			int pixel_index = (y * width + x) * pixel_components;
			float temp = 0;

			if (img[pixel_index] == 0) {
				for (int x1 = -1; x1 <= 1; x1++) {
					for (int y1 = -1; y1 <= 1; y1++) {

						int pixel_index2 = ((y + y1) * width + x + x1) * pixel_components;

						if (pixel_index2 >= 0 && pixel_index2 < nb_pixels
							&& img[pixel_index2] == 255)
						{
							edge_img[y * width + x] = 0;
							x1 = 2;
							y1 = 2;
						}
					}
				}
			}
		}
	}

	stbi_write_bmp("C:/Users/PC/Desktop/poc/test.bmp", width, height, pixel_components, edge_img);

	delete[] edge_img;
	delete[] img;
}


void ImageToPolygons::ThresholdImage(unsigned char* img, int& width, int& height, int& pixel_component)
{
	int seuil = 125;

	// créer les triangles pour les pixels noirs (en dessous du seuil)
	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++)
		{
			int pixel_index = ((height - 1 - y) * width + x) * pixel_component;

			if (img[pixel_index] < seuil)
				img[pixel_index] = 0;
			else
				img[pixel_index] = 255;
		}
	}
}


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



void ImageToPolygons::ConvertImageToPolygons(const char* filepath)
{
	int width, height, pixel_components;
	unsigned char* img = stbi_load(filepath, &width, &height, &pixel_components, 0);
	printf("width: %i, height: %i, pixel_bits: %i\n", width, height, pixel_components);

	/// compter le nombre de pixels noirs
	// allocation gpu
	unsigned char* dimg;
	cudaMalloc((void**)&dimg, width * height * sizeof(unsigned char));
	cudaMemcpy(dimg, img, width * height * sizeof(unsigned char), cudaMemcpyHostToDevice);

	int threshold = 125;
	int counter = ImageToPolygons::CountBlackPixels(dimg, width, height, threshold);

	/// créer un tableau de counter triangles


	printf("counter: %i\n", counter);
	cudaFree(dimg);
}