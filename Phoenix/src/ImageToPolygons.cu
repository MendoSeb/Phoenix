#include <cstdio>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "ImageToPolygons.h"
#include <cmath>
#include <thrust/device_vector.h>


__global__ void KernelCountBlackPixels(unsigned char* img, int nb_pixels, int* dcounter, int threshold)
{
	int thread_id = blockIdx.x * blockDim.x + threadIdx.x;

	if (thread_id >= 0 && thread_id < nb_pixels
		&& img[thread_id] < threshold)
	{
		atomicAdd(dcounter, 1);
	}
}



int ImageToPolygons::CountBlackPixels(unsigned char* img, int& width, int& height, int& threshold)
{
	int nb_pixels = width * height;
	int nb_blocks = std::ceil(nb_pixels / 256);

	int* dcounter = nullptr;
	cudaMalloc((void**)&dcounter, sizeof(int));
	cudaMemset(dcounter, 0, sizeof(int));

	KernelCountBlackPixels <<<nb_blocks, 256>>> (img, nb_pixels, dcounter, threshold);
	cudaDeviceSynchronize();

	int* counter = new int(0);
	cudaMemcpy(counter, dcounter, sizeof(int), cudaMemcpyDeviceToHost);

	int cpt = *counter;
	delete counter;

	return cpt;
}




__global__ void AddTriangle()
{


}