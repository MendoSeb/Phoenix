#include "cudaCall.h"
#include <cstdio>
#include <cuda_runtime.h>
#include "Warping.h"
#include <tiffio.h>


// pineda edge function
__device__ bool isVertexRightOfSegment(uint2* v, uint2* s1, uint2* s2)
{
	return ((double)v->x - (double)s1->x) * ((double)s2->y - (double)s1->y) 
		- ((double)v->y - (double)s1->y) * ((double)s2->x - (double)s1->x) >= 0.0;
}


// les triangles sont dans le sens horaire
__device__ bool isVertexInsideTriangle(uint2* vertex, uint2 tri[3])
{
	return isVertexRightOfSegment(vertex, &tri[0], &tri[1])
		&& isVertexRightOfSegment(vertex, &tri[1], &tri[2])
		&& isVertexRightOfSegment(vertex, &tri[2], &tri[0]);
}


// les boites doivent être dans le sens horaire
__device__ bool isVertexInsideBox(const double2 box[4], uint2* vertex)
{
	uint2 tri1 [3] {
		{box[0].x, box[0].y},
		{box[1].x, box[1].y},
		{box[2].x, box[2].y},
	};

	uint2 tri2[3]{
		{box[0].x, box[0].y},
		{box[2].x, box[2].y},
		{box[3].x, box[3].y},
	};

	return isVertexInsideTriangle(vertex, tri1)
		|| isVertexInsideTriangle(vertex, tri2);
}


__global__ void warping_kernel(
	uint2* vertices,
	uint3* triangles,
	Warping::Boxes* src_dst,
	Eigen::Matrix3d* homography_matrices
)
{
	size_t thread_global_id = blockIdx.x * blockDim.x + threadIdx.x;
	uint2* vertex = &vertices[thread_global_id];

	// for every src box
	for (size_t i = 0; i < 1; i++)
		if (isVertexInsideBox(src_dst[i].src, vertex))
		{
			Eigen::Matrix<double, 3, 1> p;
			p.row(0) << vertex->x;
			p.row(1) << vertex->y;
			p.row(2) << 1;

			p = homography_matrices[i] * p;
			vertex->x = p(0) / p(2);
			vertex->y = p(1) / p(2);
		}
}


__device__ uint4 getTriangleBoundingBox(uint2 tri[3])
{
	uint4 min_max = { UINT32_MAX, UINT32_MAX, 0, 0 };

	for (size_t i = 0; i < 3; i++)
	{
		min_max.x = fmin((float)min_max.x, (float)tri[i].x);
		min_max.y = fmin((float)min_max.y, (float)tri[i].y);

		min_max.z = fmax((float)min_max.z, (float)tri[i].x);
		min_max.w = fmax((float)min_max.w, (float)tri[i].y);
	}

	return min_max;
}


__global__ void rasterization_kernel(
	uint2* vertices,
	uint3* triangles,
	size_t nb_triangles,
	uint2 img_dim,
	unsigned char* img
)
{
	uint2 tri[3] = {
		vertices[triangles[blockIdx.x].x],
		vertices[triangles[blockIdx.x].y],
		vertices[triangles[blockIdx.x].z],
	};

	uint4 min_max = getTriangleBoundingBox(tri);

	// est-ce que le thread a un travail à faire ? est-il dans la boite englobante du triangle ?
	float x_diff = min_max.z - min_max.x;
	float y_diff = min_max.w - min_max.y;

	uint nb_x_step = std::ceil(x_diff / 16);
	uint nb_y_step = std::ceil(y_diff / 16);

	for (size_t x = 0; x < nb_x_step; x++)
		for (size_t y = 0; y < nb_y_step; y++)
		{
			uint2 pixel{
				(min_max.x + threadIdx.x * nb_x_step) + x,
				(min_max.y + threadIdx.y * nb_y_step) + y
			};

			if (pixel.x >= min_max.x && pixel.x < min_max.z // dans le boite englobante du triangle
				&& pixel.y >= min_max.y && pixel.y < min_max.w)
			{
				if (isVertexInsideTriangle(&pixel, tri))
				{
					size_t pixel_index = pixel.y * img_dim.x + pixel.x;
					img[pixel_index] = 0;
				}
			}
		}
}


void CudaCall::warping
(
	std::pair<std::pair<uint2*, uint3*>, uint2>& tris,
	std::vector<Warping::Boxes>& src_dst
)
{
	//calcul des matrices de transformation
	Warping::Boxes* src_dst2 = new Warping::Boxes[src_dst.size()];
	Eigen::Matrix3d* homography_matrices = new Eigen::Matrix3d[src_dst.size()];

	for (size_t i = 0; i < src_dst.size(); i++)
	{
		src_dst2[i] = src_dst[i];

		homography_matrices[i] =
			Warping::getPerspectiveMatrixTransform(src_dst[i].src, src_dst[i].dst);
	}

	// allocation mémoire gpu
	uint2 *dv = nullptr;
	uint3* dt = nullptr;
	Warping::Boxes* db = nullptr;
	Eigen::Matrix3d* dm = nullptr;

	cudaStream_t stream;
	cudaStreamCreate(&stream);

	cudaMallocAsync((void**)&dv, sizeof(uint2) * tris.second.x, stream);
	cudaMallocAsync((void**)&dt, sizeof(uint3) * tris.second.y, stream);
	cudaMallocAsync((void**)&db, sizeof(Warping::Boxes) * src_dst.size(), stream);
	cudaMallocAsync((void**)&dm, sizeof(Eigen::Matrix3d) * src_dst.size(), stream);

	cudaMemcpyAsync(dv, tris.first.first, tris.second.x * sizeof(uint2), cudaMemcpyHostToDevice, stream);
	cudaMemcpyAsync(dt, tris.first.second, tris.second.y * sizeof(uint3), cudaMemcpyHostToDevice, stream);
	cudaMemcpyAsync(db, src_dst2, src_dst.size() * sizeof(Warping::Boxes), cudaMemcpyHostToDevice, stream);
	cudaMemcpyAsync(dm, homography_matrices, src_dst.size() * sizeof(Eigen::Matrix3d), cudaMemcpyHostToDevice, stream);

	// appel
	size_t nb_blocks = std::floor(tris.second.x / 1024);
	size_t nb_threads_per_blocks = 1024;
	std::cout << "Nb threads = " << nb_blocks * nb_threads_per_blocks << std::endl;
	std::cout << "Nb vertices = " << tris.second.x << std::endl;

	warping_kernel <<<nb_blocks, nb_threads_per_blocks>>> (dv, dt, db, dm);
	//kernel <<<1, 3>>> (dv, dt, db, dm);
	cudaDeviceSynchronize();

	// coppie du résultat vers les sommets
	cudaMemcpy(tris.first.first, dv, tris.second.x * sizeof(float2), cudaMemcpyDeviceToHost);

	// libération mémoire
	cudaFreeAsync(dv, stream);
	cudaFreeAsync(dt, stream);
	cudaFreeAsync(db, stream);
	cudaFreeAsync(dm, stream);
	cudaStreamDestroy(stream);

	delete src_dst2;
	delete homography_matrices;
}


void CudaCall::rasterization(
	std::pair<std::pair<uint2*, uint3*>, uint2>& tris,
	float& scale
	)
{
	uint2 img_dim = { 38400, 30000 };
	size_t nb_pixels = img_dim.x * img_dim.y;

	// allocation mémoire
	uint2* dv = nullptr;
	uint3* dt = nullptr;
	unsigned char* dimg = nullptr;

	std::chrono::steady_clock::time_point s1 = std::chrono::steady_clock::now();

	cudaStream_t stream;
	cudaStreamCreate(&stream);

	cudaMallocAsync((void**)&dv, sizeof(uint2) * tris.second.x, stream);
	cudaMallocAsync((void**)&dt, sizeof(uint3) * tris.second.y, stream);
	cudaMallocAsync((void**)&dimg, sizeof(unsigned char) * nb_pixels, stream);

	cudaMemcpyAsync(dv, tris.first.first, tris.second.x * sizeof(uint2), cudaMemcpyHostToDevice, stream);
	cudaMemcpyAsync(dt, tris.first.second, tris.second.y * sizeof(uint3), cudaMemcpyHostToDevice, stream);

	cudaMemsetAsync(dimg, 255, sizeof(unsigned char) * nb_pixels, stream);

	std::chrono::steady_clock::time_point s2 = std::chrono::steady_clock::now();
	std::cout << "allocation vram en " << std::chrono::duration_cast<std::chrono::milliseconds>(s2 - s1).count() << " ms" << std::endl;


	// call
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	rasterization_kernel <<<tris.second.y, dim3(16, 16, 1) >> >(dv, dt, tris.second.y, img_dim, dimg);
	cudaDeviceSynchronize();

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Rasterisation en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

	std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
	unsigned char* img = new unsigned char[nb_pixels];
	cudaMemcpy(img, dimg, nb_pixels * sizeof(unsigned char), cudaMemcpyDeviceToHost);
	std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
	std::cout << "copie de l'image gpu -> cpu " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count() << " ms" << std::endl;

	CudaCall::saveToBmp(
		"C:/Users/PC/Desktop/poc/rasterization.bmp", 
		img_dim.x, 
		img_dim.y,
		img
	);

	// libération de la mémoire
	cudaFreeAsync(dv, stream);
	cudaFreeAsync(dt, stream);
	cudaFreeAsync(dimg, stream);
	cudaStreamDestroy(stream);

	delete img;
}


void CudaCall::saveToBmp(const std::string& filename, int width, int height,
	unsigned char* hostData)
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	// 2. Prepare BMP Headers and Palette
	CudaCall::BitmapFileHeader fileHeader;
	CudaCall::BitmapInfoHeader infoHeader;

	infoHeader.width = width;
	infoHeader.height = height;
	infoHeader.bit_count = 8;
	infoHeader.size = sizeof(CudaCall::BitmapInfoHeader);

	uint32_t palette_size = 256 * 4; // 256 grayscale entries, 4 bytes each
	fileHeader.offset_data = sizeof(CudaCall::BitmapFileHeader) 
		+ sizeof(CudaCall::BitmapInfoHeader) + palette_size;

	fileHeader.file_size = fileHeader.offset_data + (width * height);

	std::vector<char> palette(palette_size);
	for (int i = 0; i < 256; ++i) {
		palette[i * 4 + 0] = 255 - i; // Blue
		palette[i * 4 + 1] = 255 - i; // Green
		palette[i * 4 + 2] = 255 - i; // Red
		palette[i * 4 + 3] = 0;  // Reserved
	}

	// 3. Write data to the file
	std::ofstream file(filename, std::ios::binary);
	if (!file.is_open()) {
		std::cerr << "Error: Could not open file for writing." << std::endl;
		delete[] hostData;
		return;
	}

	file.write(reinterpret_cast<const char*>(&fileHeader), sizeof(fileHeader));
	file.write(reinterpret_cast<const char*>(&infoHeader), sizeof(infoHeader));
	file.write(palette.data(), palette.size());
	file.write(reinterpret_cast<const char*>(hostData), width * height);

	file.close();

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Sauvegarde en .bmp en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
}


void CudaCall::saveToTiff(unsigned char* img, uint2 img_dim)
{
	TIFF* out = TIFFOpen("C:/Users/PC/Desktop/poc/rasterization.tif", "w8");
	int sampleperpixel = 1;    // or 3 if there is no alpha channel, you should get a understanding of alpha in class soon.

	TIFFSetField(out, TIFFTAG_IMAGEWIDTH, img_dim.x);  // set the width of the image
	TIFFSetField(out, TIFFTAG_IMAGELENGTH, img_dim.y);    // set the height of the image
	TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, sampleperpixel);   // set number of channels per pixel
	TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, 8);    // set the size of the channels
	TIFFSetField(out, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);    // set the origin of the image.
	//   Some other essential fields to set that you do not have to understand for now.
	TIFFSetField(out, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(out, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);

	size_t stride = (size_t)img_dim.x * sampleperpixel;
	unsigned char* buf = (unsigned char*)_TIFFmalloc(TIFFScanlineSize(out));

	TIFFSetField(out, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(out, img_dim.x * sampleperpixel));

	for (uint32 row = 0; row < img_dim.y; row++)
	{
		memcpy(buf, &img[(size_t)(img_dim.y - row - 1) * stride], stride);
		if (TIFFWriteScanline(out, buf, row, 0) < 0)
			break;
	}

	(void)TIFFClose(out);

	if (buf)
		_TIFFfree(buf);
}