#pragma once
#include <TriangulationUtils.h>
#include <Optix.h>


class DirectImagingSimulator
{
private:
	Params hparam;

public:

	DirectImagingSimulator();
	~DirectImagingSimulator();

	float* CreateLuminanceMatrix(int width, int height);

	unsigned char* CreateLuminanceCorrectionMatrix(float* luminance_matrix);

	void CreateImagesBuffer(
		float*& sim_img,
		float*& dimg,
		float*& dluminance_matrix,
		unsigned char*& dluminance_correction
	);

	void SetOptixParams(
		CUdeviceptr& dparam,
		float* dimg,
		float* dluminance_matrix,
		unsigned char* dluminance_correction,
		TrisUtils::Triangulation& tris,
		Optix& o
	);

	void CudaWriteImage(
		float* dimg,
		float* dsim_img,
		int& shift
	);

	void SaveSimImageAsBmp(
		Optix& o,
		float*& dsim_img
	);

	void SaveSimImageAsOBJ(const char* filename, float*& dsim_img) const;

	void SimulateLithography();
};