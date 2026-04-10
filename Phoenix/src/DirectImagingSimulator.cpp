#include "DirectImagingSimulator.h"
#include <corecrt_math.h>
#include <CalibrationImageGenerator.h>


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


void DirectImagingSimulator::CreateImagesBuffer(
    float*& dsim_img,
	float*& dimg,
	float*& dluminance_matrix,
	unsigned char*& dluminance_correction
)
{
	// create the two matrices to simulate lens luminance and correction of that
	float* luminance_matrix = CreateLuminanceMatrix(sp.dmd_width, sp.dmd_height);
	unsigned char* luminance_correction = CreateLuminanceCorrectionMatrix(sp.dmd_width, sp.dmd_height, luminance_matrix);

	// allocation vram
	int nb_img_pixels_img = sp.img_width * sp.img_height;
	int nb_img_pixels_dmd = sp.dmd_width * sp.dmd_height;

	cudaMalloc((void**)&dsim_img, nb_img_pixels_img * sizeof(float));
	cudaMemset(dsim_img, 0, nb_img_pixels_img * sizeof(float));

	cudaMalloc((void**)(&dimg), nb_img_pixels_dmd * sizeof(float));
	cudaMemset(dimg, 0, nb_img_pixels_dmd * sizeof(float));

	cudaMalloc((void**)(&dluminance_matrix), nb_img_pixels_dmd * sizeof(float));
	cudaMemcpy(dluminance_matrix, luminance_matrix, nb_img_pixels_dmd * sizeof(float), cudaMemcpyHostToDevice);

	cudaMalloc((void**)(&dluminance_correction), nb_img_pixels_dmd * sizeof(unsigned char));	
	cudaMemcpy(dluminance_correction, luminance_correction, nb_img_pixels_dmd * sizeof(unsigned char), cudaMemcpyHostToDevice);

	delete[] luminance_matrix;
	delete[] luminance_correction;
}


void DirectImagingSimulator::SetOptixParams(
	CUdeviceptr& dparam,
	Params& hparam,
	float* dimg,
	float* dluminance_matrix,
	unsigned char* dluminance_correction,
	TrisUtils::Triangulation& tris,
	Optix& o
)
{
	int images_spacing = sp.dmd_height / sp.sp;

	hparam.image = dimg;
	hparam.dmd_width = sp.dmd_width;
	hparam.dmd_height = sp.dmd_height + images_spacing;
	hparam.img_width = sp.img_width;
	hparam.img_height = sp.img_height;
	hparam.cam_position = float3{ 0.0f, (float)-sp.dmd_height, 10.0f};

	hparam.luminance_matrix = dluminance_matrix;
	hparam.luminance_correction = dluminance_correction;
	hparam.handle = o.GetGasHandle();

    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&hparam.polarity), tris.nb_triangles * sizeof(unsigned char)));
	CUDA_CHECK(cudaMemcpy(hparam.polarity, tris.p, tris.nb_triangles * sizeof(unsigned char), cudaMemcpyHostToDevice));

	CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&dparam), sizeof(Params)));
}


void DirectImagingSimulator::SaveSimImageAsBmp(
	Optix& o, 
	int& img_width, 
	int& img_height, 
	float* dsim_img
)
{
	int nb_pixels_img = img_width * img_height;
	float* himg = new float[nb_pixels_img];
	cudaMemcpy(himg, dsim_img, nb_pixels_img * sizeof(float), cudaMemcpyDeviceToHost); // copy from vram to ram

	// convert from float image to unsigned char
	unsigned char* uc_img = new unsigned char[nb_pixels_img];

	// find max value to normalize
	float max_value = 0;

	for (int i = 0; i < nb_pixels_img; i++)
		if (himg[i] > max_value)
			max_value = himg[i];

	for (int i = 0; i < nb_pixels_img; i++)
		uc_img[i] = (1.0f - (himg[i] / max_value)) * 255.0f;

	o.saveToBmp("C:/Users/PC/Desktop/poc/DirectImagingSimulation.bmp", img_width, img_height, uc_img);

	delete[] uc_img;
	delete[] himg;
}


void DirectImagingSimulator::SimulateLithography()
{
	/// svg as triangulation layers
	TrisUtils::Triangulation tris = TrisUtils::LoadTriangulationLayersFromSVG("C:/Users/PC/Desktop/poc/test.svg", 1950);
	sp.img_width = 2000 * 7;
	sp.img_height = 2000 * 7;

	/// create image buffers for simulation
	float* dsim_img = nullptr;
	float* dimg = nullptr;
	float* dluminance_matrix = nullptr;
	unsigned char* dluminance_correction = nullptr;
	CreateImagesBuffer(dsim_img, dimg, dluminance_matrix, dluminance_correction);

	/// init optix
	Optix o(sp.dmd_width, sp.dmd_height);
	o.init();
	o.loadShaders();

	CUdeviceptr d_tris = o.initScene(tris);
	o.initPipeline(d_tris);

	/// params pour optix
	CUstream stream;
	CUDA_CHECK(cudaStreamCreate(&stream));

	Params hparam;
	CUdeviceptr dparam;
	SetOptixParams(dparam, hparam, dimg, dluminance_matrix, dluminance_correction, tris, o);

	/// begin simulation
	float2 dmd_vector{
		1.0f / 2176.0f, // quand le dmd avance d'un pixel en y il avance d'1/2176 en x
		1.0f + 1.0f / 2176.0f
	};

	int image_spacing = sp.dmd_height / sp.sp;
	int nb_images = (sp.img_height + sp.dmd_height) / image_spacing;

	for (int i = 0; i < nb_images; i++)
	{
		cudaMemcpy(reinterpret_cast<void*>(dparam), &hparam, sizeof(hparam), cudaMemcpyHostToDevice);
		o.RayCasting(sp.dmd_width, sp.dmd_height, stream, dparam);

		for (int k = 0; k < image_spacing; k++)
		{
			// copier l'image calculée par optix dans l'image de simulation à l'emplacement actuel du DMD/de la caméra
			CudaWriteImage(hparam, sp.dmd_width, sp.dmd_height, dimg, dsim_img, k);

			hparam.cam_position.x += dmd_vector.x;
			hparam.cam_position.y += dmd_vector.y;
		}
	}

	/// sauvegarder l'image de simulation/carte de hauteur en soit aussi
	SaveSimImageAsBmp(o, sp.img_width, sp.img_height, dsim_img);

	/// free memory
	CUDA_CHECK(cudaFree(dsim_img));
	CUDA_CHECK(cudaFree(dimg));
	CUDA_CHECK(cudaFree(dluminance_matrix));
	CUDA_CHECK(cudaFree(dluminance_correction));
	CUDA_CHECK(cudaStreamDestroy(stream));
}
