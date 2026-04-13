#include "DirectImagingSimulator.h"
#include <corecrt_math.h>
#include <CalibrationImageGenerator.h>


DirectImagingSimulator::DirectImagingSimulator()
{
	hparam.dmd_width = 4096;
	hparam.dmd_height = 2176;
	hparam.sp = 8 * 8;
	hparam.dmd_height_spacing = hparam.dmd_height / hparam.sp;

	hparam.resolution = 11;
	hparam.img_width = 500 * hparam.resolution;
	hparam.img_height = 500 * hparam.resolution;

	hparam.cam_position = float3{ 0.0f, -(float)hparam.dmd_height, 10.0f};
}


DirectImagingSimulator::~DirectImagingSimulator()
{
}


float* DirectImagingSimulator::CreateLuminanceMatrix(int dmd_width, int dmd_height)
{
	float* luminance_matrix = new float[dmd_height * dmd_width];

	float semi_width = (float)dmd_width / 2.0f;
	float semi_height = (float)dmd_height / 2.0f;
	float max_dist = sqrt(pow(semi_width, 2) + pow(semi_height, 2));

	for (int x = 0; x < dmd_width; x++) {
		for (int y = 0; y < dmd_height; y++)
		{
			float center_dist = sqrt(pow(x - semi_width, 2) + pow(y - semi_height, 2));
			luminance_matrix[y * dmd_width + x] = 1.0f - center_dist / max_dist;
		}
	}

	return luminance_matrix;
}


unsigned char* DirectImagingSimulator::CreateLuminanceCorrectionMatrix(float* luminance_matrix)
{
	unsigned char* luminance_correction = new unsigned char[hparam.dmd_height * hparam.dmd_width];

	// compute sum of luminance per column
	float* colums_luminance = new float[hparam.dmd_width];
	float min_luminance = FLT_MAX;

	for (int x = 0; x < hparam.dmd_width; x++) {
		float sum = 0;

		for (int y = 0; y < hparam.dmd_height; y++)
			sum += luminance_matrix[y * hparam.dmd_width + x];

		colums_luminance[x] = sum;

		if (colums_luminance[x] < min_luminance)
			min_luminance = colums_luminance[x];
	}

	// create matrix that tells if a pixel (a ray) is activated or not in the DMD
	for (int x = 0; x < hparam.dmd_width; x++) {
		float ratio = min_luminance / colums_luminance[x];
		float count = ratio;

		for (int y = 0; y < hparam.dmd_height; y++)
		{
			if (count >= 1)
			{
				luminance_correction[y * hparam.dmd_width + x] = 1;
				count -= 1.0f;
			}
			else
			{
				luminance_correction[y * hparam.dmd_width + x] = 0;
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
	float* luminance_matrix = CreateLuminanceMatrix(hparam.dmd_width, hparam.dmd_height);
	unsigned char* luminance_correction = CreateLuminanceCorrectionMatrix(luminance_matrix);

	// allocation vram
	int nb_img_pixels_img = hparam.img_width * hparam.img_height;
	int nb_img_pixels_dmd = hparam.dmd_width * hparam.dmd_height;
	int nb_img_pixels_dmd_image = hparam.dmd_width * (hparam.dmd_height + hparam.dmd_height_spacing);

	cudaMalloc((void**)&dsim_img, nb_img_pixels_img * sizeof(float));
	cudaMemset(dsim_img, 0, nb_img_pixels_img * sizeof(float));

	cudaMalloc((void**)(&dimg), nb_img_pixels_dmd_image * sizeof(float));
	cudaMemset(dimg, 0, nb_img_pixels_dmd_image * sizeof(float));

	cudaMalloc((void**)(&dluminance_matrix), nb_img_pixels_dmd * sizeof(float));
	cudaMemcpy(dluminance_matrix, luminance_matrix, nb_img_pixels_dmd * sizeof(float), cudaMemcpyHostToDevice);

	cudaMalloc((void**)(&dluminance_correction), nb_img_pixels_dmd * sizeof(unsigned char));
	cudaMemcpy(dluminance_correction, luminance_correction, nb_img_pixels_dmd * sizeof(unsigned char), cudaMemcpyHostToDevice);

	delete[] luminance_matrix;
	delete[] luminance_correction;
}


void DirectImagingSimulator::SetOptixParams(
	CUdeviceptr& dparam,
	float* dimg,
	float* dluminance_matrix,
	unsigned char* dluminance_correction,
	TrisUtils::Triangulation& tris,
	Optix& o
)
{
	hparam.image = dimg;
	hparam.luminance_matrix = dluminance_matrix;
	hparam.luminance_correction = dluminance_correction;
	hparam.handle = o.GetGasHandle();

	CUDA_CHECK(cudaMalloc((void**)&hparam.polarity, tris.nb_triangles * sizeof(unsigned char)));
	CUDA_CHECK(cudaMemcpy(hparam.polarity, tris.p, tris.nb_triangles * sizeof(unsigned char), cudaMemcpyHostToDevice));

	CUDA_CHECK(cudaMalloc((void**)& dparam, sizeof(Params)));
}


void DirectImagingSimulator::SaveSimImageAsBmp(
	Optix& o,
	float*& dsim_img
)
{
	int nb_pixels_img = hparam.img_width * hparam.img_height;
	float* himg = new float[nb_pixels_img]();
	cudaMemcpy(himg, dsim_img, nb_pixels_img * sizeof(float), cudaMemcpyDeviceToHost); // copy from vram to ram

	// convert from float image to unsigned char
	unsigned char* uc_img = new unsigned char[nb_pixels_img]();

	// find max value to normalize
	float max_value = 0;

	for (int i = 0; i < nb_pixels_img; i++)
		if (himg[i] > max_value)
			max_value = himg[i];

	std::cout << "Max value sim img: " << max_value << std::endl;

	if (max_value > 0)
		for (int i = 0; i < nb_pixels_img; i++)
			uc_img[i] = (unsigned char)((1.0f - (himg[i] / max_value)) * 255.0f);

	o.saveToBmp(
		"C:/Users/PC/Desktop/poc/DirectImagingSimulation.bmp",
		hparam.img_width,
		hparam.img_height,
		uc_img
	);

	delete[] uc_img;
	delete[] himg;
}


void DirectImagingSimulator::SaveSimImageAsOBJ(const char* filename, float*& dsim_img) const
{
	int nb_pixels_img = hparam.img_width * hparam.img_height;

	float* himg = new float[nb_pixels_img]();
	cudaMemcpy(himg, dsim_img, nb_pixels_img * sizeof(float), cudaMemcpyDeviceToHost);

	// write the .obj file
	std::ofstream file(filename);

	if (file.is_open())
	{
		float max_dim = std::max(hparam.img_width, hparam.img_height);

		// find max value to normalize
		float max_value = 0;

		for (int i = 0; i < nb_pixels_img; i++)
			if (himg[i] > max_value)
				max_value = himg[i];

		// write vertices
		for (unsigned int x = 0; x < hparam.img_width; x++) {
			for (unsigned int y = 0; y < hparam.img_height; y++)
			{
				file << "v "
					<< std::to_string(x / max_dim) << " "
					<< std::to_string(y / max_dim) << " "
					<< std::to_string(himg[y * hparam.img_width + x] / (max_value * 300)) << "\n";
			}
		}

		// write triangles
		for (int x = 0; x < hparam.img_width - 1; x++) {
			for (int y = 0; y < hparam.img_height - 1; y++)
			{
				int index0 = y * hparam.img_width + x;
				int index1 = y * hparam.img_width + x + 1;
				int index2 = (y + 1) * hparam.img_width + x;
				int index3 = (y + 1) * hparam.img_width + x + 1;

				file << "f "
					<< std::to_string(index0 + 1) << " "
					<< std::to_string(index1 + 1) << " "
					<< std::to_string(index2 + 1) << "\n";

				file << "f "
					<< std::to_string(index1 + 1) << " "
					<< std::to_string(index2 + 1) << " "
					<< std::to_string(index3 + 1) << "\n";
			}
		}
	}

	file.close();

	delete[] himg;
}


void DirectImagingSimulator::SimulateLithography()
{
	/// svg as triangulation layers
	TrisUtils::Triangulation tris = 
		TrisUtils::LoadTriangulationLayersFromSVG("C:/Users/PC/Desktop/poc/test.svg", 3000);
		//TrisUtils::LoadTriangulationLayersFromSVG("C:/Users/PC/Downloads/004672-647720058a0 (1).top_LAYERS.svg", 3000);
	//TrisUtils::WriteObj("C:/Users/PC/Desktop/poc/test.obj", tris);

	/// create image buffers for simulation
	float* dsim_img = nullptr;
	float* dimg = nullptr;
	float* dluminance_matrix = nullptr;
	unsigned char* dluminance_correction = nullptr;
	CreateImagesBuffer(dsim_img, dimg, dluminance_matrix, dluminance_correction);

	/// init optix
	Optix o;
	o.init();
	o.loadShaders();

	CUdeviceptr d_tris = o.initScene(tris);
	o.initPipeline(d_tris);

	/// params pour optix
	CUstream stream;
	cudaStreamCreate(&stream);

	CUdeviceptr dparam;
	SetOptixParams(dparam, dimg, dluminance_matrix, dluminance_correction, tris, o);

	/// begin simulation
	float2 dmd_vector{
		1.0f / hparam.dmd_height, // quand le dmd avance d'un pixel en y il avance d'1/2176 en x
		1.0f + 1.0f / (hparam.dmd_height / 8.0f)
	};

	int nb_images = (hparam.img_height + hparam.dmd_height) / hparam.dmd_height_spacing;

	for (int i = 0; i < nb_images; i++)
	{
		cudaMemcpy(reinterpret_cast<void*>(dparam), &hparam, sizeof(hparam), cudaMemcpyHostToDevice);
		o.RayCasting(hparam.dmd_width, hparam.dmd_height + hparam.dmd_height_spacing, stream, dparam);
		//printf("Calcul d'une image\n");
		
		for (int k = 0; k < hparam.dmd_height_spacing; k++)
		{
			// copier l'image calculée par optix dans l'image de simulation à l'emplacement actuel du DMD/de la caméra
			CudaWriteImage(dimg, dsim_img, k);
			//printf("HZ dmd\n");

			hparam.cam_position.x += dmd_vector.x;
			hparam.cam_position.y += dmd_vector.y;
		}
	}

	/// sauvegarder l'image de simulation/carte de hauteur en soit aussi
	SaveSimImageAsBmp(o, dsim_img);

	//SaveSimImageAsOBJ("C:/Users/PC/Desktop/poc/SimulationMap.obj", dsim_img);

	/// free memory
	CUDA_CHECK(cudaFree(dsim_img));
	CUDA_CHECK(cudaFree(dimg));
	CUDA_CHECK(cudaFree(dluminance_matrix));
	CUDA_CHECK(cudaFree(dluminance_correction));
	CUDA_CHECK(cudaStreamDestroy(stream));
}
