#include "DirectImagingSimulator.h"
#include <corecrt_math.h>
#include <Optix.h>


DirectImagingSimulator::DirectImagingSimulator()
{ 
	sp.dmd_width = 4096;
	sp.dmd_height = 2176;

	sp.img_width = 2000;
	sp.img_height = 2000;
	sp.resolution = 1;

	sp.sp = 8 * 8;
}


DirectImagingSimulator::~DirectImagingSimulator()
{
}


float* DirectImagingSimulator::CreateLuminanceMatrix(int width, int height)
{
	float* luminance_matrix = new float[height * width];

	float semi_width = (float)width / 2.0f;
	float semi_height = (float)height / 2.0f;
	float max_dist = sqrt(pow(semi_width, 2) + pow(semi_height, 2));

	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++)
		{
			float center_dist = sqrt(pow(x - semi_width, 2) + pow(y - semi_height, 2));
			luminance_matrix[y * width + x] = 1.0f - center_dist / max_dist;
		}
	}

	return luminance_matrix;
}


unsigned char* DirectImagingSimulator::CreateLuminanceCorrectionMatrix(int width, int height, float* luminance_matrix)
{
	unsigned char* luminance_correction = new unsigned char[height * width];

	// compute sum of luminance per column
	float* colums_luminance = new float[width];
	float min_luminance = FLT_MAX;

	for (int x = 0; x < width; x++) {
		float sum = 0;

		for (int y = 0; y < height; y++)
			sum += luminance_matrix[y * width + x];

		colums_luminance[x] = sum;

		if (colums_luminance[x] < min_luminance)
			min_luminance = colums_luminance[x];
	}

	// create matrix that tells if a pixel (a ray) is activated or not in the DMD
	for (int x = 0; x < width; x++) {
		float ratio = min_luminance / colums_luminance[x];
		float count = ratio;

		for (int y = 0; y < height; y++)
		{
			if (count >= 1)
			{
				luminance_correction[y * width + x] = 1;
				count -= 1.0f;
			}
			else
			{
				luminance_correction[y * width + x] = 0;
			}

			count += ratio;
		}
	}

	return luminance_correction;
}


void DirectImagingSimulator::SimulateLithography()
{
	TrisUtils::Triangulation tris = TrisUtils::LoadTriangulationLayersFromSVG("C:/Users/PC/Desktop/poc/test4.svg", 1950);
	sp.img_width = 200 * 4;
	sp.img_height = 200 * 4;

	Optix o(sp.dmd_width, sp.dmd_height);
	o.init();
	o.loadShaders();

	CUdeviceptr d_tris = o.initScene(tris);
	o.initPipeline(d_tris);

	float* luminance_matrix = CreateLuminanceMatrix(sp.dmd_width, sp.dmd_height);
	unsigned char* luminance_correction = CreateLuminanceCorrectionMatrix(sp.dmd_width, sp.dmd_height, luminance_matrix);

	unsigned char* lum_m2 = new unsigned char[sp.dmd_width * sp.dmd_height];

	for (int i = 0; i < sp.dmd_width * sp.dmd_height; i++)
		lum_m2[i] = luminance_matrix[i] * 255;

	o.saveToBmp("C:/Users/PC/Desktop/poc/luminance_matrix.bmp", sp.dmd_width, sp.dmd_height, lum_m2);
	o.saveToBmp("C:/Users/PC/Desktop/poc/luminance_correction.bmp", sp.dmd_width, sp.dmd_height, luminance_correction);

	delete[] lum_m2;

	o.DISimulation(sp, tris, luminance_matrix, luminance_correction);

	delete[] luminance_matrix;
	delete[] luminance_correction;
}
