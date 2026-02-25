#include "cudaCall.h"
#include <cstdio>
#include <cuda_runtime.h>
#include "Warping.h"
#include <tiffio.h>

using namespace CudaCall;


// pineda edge function
__device__ bool isVertexRightOfSegment(const float2& v, const float2& s1, const float2& s2)
{
	return (v.x - s1.x) * (s2.y - s1.y) - (v.y - s1.y) * (s2.x - s1.x) >= 0.0f;
}

__device__ bool checkPointInTriangleFast(const float2& pixel, float2 tri[3]
) 
{
	// On calcule le signe de l'aire formée par le point et chaque arête
	// (ax-px)*(by-py) - (ay-py)*(bx-px)

	auto sign = [](float x1, float y1, float x2, float y2, float x3, float y3) 
	{
		return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
	};

	float d1 = sign(pixel.x, pixel.y, tri[0].x, tri[0].y, tri[1].x, tri[1].y);
	float d2 = sign(pixel.x, pixel.y, tri[1].x, tri[1].y, tri[2].x, tri[2].y);
	float d3 = sign(pixel.x, pixel.y, tri[2].x, tri[2].y, tri[0].x, tri[0].y);

	// Un point est dedans si toutes les aires ont le même signe
	bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
	bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

	// Le résultat est vrai si on n'a pas à la fois du positif et du négatif
	// (ce qui couvre aussi le cas où l'un des d est à 0 : point sur l'arête)
	return !(has_neg && has_pos);
}

// les triangles sont dans le sens horaire
__device__ bool isVertexInsideTriangle(const float2& vertex, const float2 tri[3])
{
	return isVertexRightOfSegment(vertex, tri[0], tri[1])
		&& isVertexRightOfSegment(vertex, tri[1], tri[2])
		&& isVertexRightOfSegment(vertex, tri[2], tri[0]);
}

// les boites doivent être dans le sens horaire
__device__ bool isVertexInsideBox(const float2 box[4], const float2& vertex)
{
	float2 tri1 [3] {
		{box[0].x, box[0].y},
		{box[1].x, box[1].y},
		{box[2].x, box[2].y},
	};

	float2 tri2[3]{
		{box[0].x, box[0].y},
		{box[2].x, box[2].y},
		{box[3].x, box[3].y},
	};

	return isVertexInsideTriangle(vertex, tri1)
		|| isVertexInsideTriangle(vertex, tri2);
}


__global__ void warping_kernel(
	float2* vertices,
	uint3* triangles,
	Warping::Boxes* src_dst,
	Eigen::Matrix3d* homography_matrices
)
{
	size_t thread_global_id = blockIdx.x * blockDim.x + threadIdx.x;
	float2* vertex = &vertices[thread_global_id];

	// for every src box
	for (size_t i = 0; i < 1; i++)
		if (isVertexInsideBox(src_dst[i].src, *vertex))
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


__device__ float4 getTriangleBoundingBox(float2 tri[3])
{
	float4 min_max = { FLT_MAX, FLT_MAX, FLT_MIN, FLT_MIN };

	for (size_t i = 0; i < 3; i++)
	{
		min_max.x = fmin(min_max.x, tri[i].x);
		min_max.y = fmin(min_max.y, tri[i].y);

		min_max.z = fmax(min_max.z, tri[i].x);
		min_max.w = fmax(min_max.w, tri[i].y);
	}

	return min_max;
}


__global__ void rasterization_kernel(
	float2* vertices,
	uint3* triangles,
	size_t nb_triangles,
	uint2 img_dim,
	unsigned char* img
)
{
	size_t thread_x = blockIdx.x * blockDim.x + threadIdx.x;
	size_t thread_y = blockIdx.y * blockDim.y + threadIdx.y;
	size_t pixel_index = thread_y * img_dim.x + thread_x;

	unsigned char val = 255;

	if (pixel_index < img_dim.x * img_dim.y)
	{
		for (size_t i = 0; i < nb_triangles; i++)
		{
			float2 tri[3] = {
				vertices[triangles[i].x],
				vertices[triangles[i].y],
				vertices[triangles[i].z],
			};

			float4 min_max = getTriangleBoundingBox(tri);
			float2 pixel{ thread_x, thread_y };

			if ( checkPointInTriangleFast(pixel, tri) )
			{
				val = 0;
				break;
			}
		}

		img[pixel_index] = val;
	}
}


__device__ bool isTriangleInBoundingBox(float2 triangle[3], const float2& min_pos, const float2& max_pos)
{
	return (triangle[0].x >= min_pos.x && triangle[0].x <= max_pos.x
		&& triangle[0].y >= min_pos.y && triangle[0].y <= max_pos.y)
		|| (triangle[1].x >= min_pos.x && triangle[1].x <= max_pos.x
			&& triangle[1].y >= min_pos.y && triangle[1].y <= max_pos.y)
		|| (triangle[2].x >= min_pos.x && triangle[2].x <= max_pos.x
			&& triangle[2].y >= min_pos.y && triangle[2].y <= max_pos.y);
}


__global__ void bvhKernel(
	CudaCall::BVHNode* nodes,
	float2* d_vertices,
	uint3* d_triangles,
	uint* d_all_indices,
	uint* indices_counter,
	uint parent_index,
	uint depth,
	uint nb_nodes_done
)
{
	uint parent_offset = nodes[0].offset;
	uint parent_count = nodes[0].count;

	float half_width = (nodes[parent_index].max_pos.x - nodes[parent_index].min_pos.x) / 2.0f;
	float half_height = (nodes[parent_index].max_pos.y - nodes[parent_index].min_pos.y) / 2.0f;

	// enfant 1
	for (uint x = 0; x < 2; x++)
		for (uint y = 0; y < 2; y++)
		{
			uint node_id = nb_nodes_done + threadIdx.x;
			BVHNode* child = &nodes[node_id];

			// compter le nombre de triangles qui sont dans la boite englobante de cet enfant
			for (uint i = 0; i < parent_count; i++) // pour chaque indice de triangle
			{
				uint tri_index = d_all_indices[parent_offset + i];

				float2 tri[3] = {
					d_vertices[d_triangles[tri_index].x],
					d_vertices[d_triangles[tri_index].y],
					d_vertices[d_triangles[tri_index].z]
				};

				child->min_pos.x = nodes[parent_index].min_pos.x + i * half_width;
				child->min_pos.y = nodes[parent_index].min_pos.y + i * half_height;
				child->max_pos.x = nodes[parent_index].min_pos.x + (i + 1) * half_width;
				child->max_pos.y = nodes[parent_index].min_pos.y + (i + 1) * half_height;

				if (isTriangleInBoundingBox(tri, child->min_pos, child->max_pos))
					child->count++;
			}

			printf("%i child: %f, %f, %f, %f\n", depth, child->min_pos.x, child->min_pos.y, child->max_pos.x, child->max_pos.y);

			// ajouter count à indices_counter et renvoie indices_counter avant l'addition
			child->offset = atomicAdd(indices_counter, child->count);
			uint current_nb_child_tri = 0;

			// ajouter dans all_indices les index des triangles qui tombent dans cet enfant
			for (uint i = 0; i < parent_count; i++) // pour chaque indice de triangle
			{
				uint tri_index = d_all_indices[parent_offset + i];

				float2 tri[3] = {
					d_vertices[d_triangles[tri_index].x],
					d_vertices[d_triangles[tri_index].y],
					d_vertices[d_triangles[tri_index].z]
				};

				BVHNode* child = &nodes[node_id];

				if (isTriangleInBoundingBox(tri, child->min_pos, child->max_pos))
				{
					d_all_indices[child->offset + current_nb_child_tri] = tri_index;
					current_nb_child_tri++;
				}
			}
		}
}


__global__ void bvhKernelV1(
	CudaCall::BVHNode* dleafs,
	int* d_all_indices,
	float2* d_vertices,
	uint3* d_triangles,
	uint nb_triangles,
	uint nb_leafs
)
{
	uint tri_id = blockIdx.x * blockDim.x + threadIdx.x;

	float2 tri[3] = {
		d_vertices[d_triangles[tri_id].x],
		d_vertices[d_triangles[tri_id].y],
		d_vertices[d_triangles[tri_id].z]
	};

	for (uint i = 0; i < nb_leafs; i++)
	{
		BVHNode* leaf = &dleafs[i];

		if (isTriangleInBoundingBox(tri, leaf->min_pos, leaf->max_pos))
		{
			d_all_indices[leaf->offset + tri_id] = tri_id;
			leaf->count++;
		}
	}
}

void CudaCall::warping
(
	std::pair<std::pair<float2*, uint3*>, uint2>& tris,
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
	float2 *dv = nullptr;
	uint3* dt = nullptr;
	Warping::Boxes* db = nullptr;
	Eigen::Matrix3d* dm = nullptr;

	cudaStream_t stream;
	cudaStreamCreate(&stream);

	cudaMallocAsync((void**)&dv, sizeof(float2) * tris.second.x, stream);
	cudaMallocAsync((void**)&dt, sizeof(uint3) * tris.second.y, stream);
	cudaMallocAsync((void**)&db, sizeof(Warping::Boxes) * src_dst.size(), stream);
	cudaMallocAsync((void**)&dm, sizeof(Eigen::Matrix3d) * src_dst.size(), stream);

	cudaMemcpyAsync(dv, tris.first.first, tris.second.x * sizeof(float2), cudaMemcpyHostToDevice, stream);
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
	std::pair<std::pair<float2*, uint3*>, uint2>& tris)
{
	uint2 img_dim = { 1000, 1000 };
	size_t nb_pixels = img_dim.x * img_dim.y;

	// allocation mémoire
	float2* dv = nullptr;
	uint3* dt = nullptr;
	unsigned char* dimg = nullptr;

	std::chrono::steady_clock::time_point s1 = std::chrono::steady_clock::now();

	cudaStream_t stream;
	cudaStreamCreate(&stream);

	cudaMallocAsync((void**)&dv, sizeof(float2) * tris.second.x, stream);
	cudaMallocAsync((void**)&dt, sizeof(uint3) * tris.second.y, stream);
	cudaMallocAsync((void**)&dimg, sizeof(unsigned char) * nb_pixels, stream);

	cudaMemcpyAsync(dv, tris.first.first, tris.second.x * sizeof(float2), cudaMemcpyHostToDevice, stream);
	cudaMemcpyAsync(dt, tris.first.second, tris.second.y * sizeof(uint3), cudaMemcpyHostToDevice, stream);

	cudaMemsetAsync(dimg, 255, sizeof(unsigned char) * nb_pixels, stream);

	std::chrono::steady_clock::time_point s2 = std::chrono::steady_clock::now();
	std::cout << "allocation vram en " << std::chrono::duration_cast<std::chrono::milliseconds>(s2 - s1).count() << " ms" << std::endl;

	// call
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	rasterization_kernel <<<dim3(img_dim.x / 16, img_dim.y / 16, 1), dim3(16, 16, 1) >> >(dv, dt, tris.second.y, img_dim, dimg);

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Rasterisation en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

	std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
	unsigned char* img = new unsigned char[nb_pixels];
	cudaMemcpyAsync(img, dimg, nb_pixels * sizeof(unsigned char), cudaMemcpyDeviceToHost, stream);
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

void CudaCall::bvhV1(std::pair<std::pair<float2*, uint3*>, uint2>& tris)
{
	uint depth = 2;
	uint nb_leafs = std::pow(4, depth);
	uint nb_indices_max = nb_leafs * tris.second.y;

	/// construire l'arbre équilibré (équilibré je crois)
	std::vector<BVHNode*> leafs;
	//leafs.resize(nb_leafs);

	auto lambda = [&](auto&& lambda, std::vector<BVHNode*>& leafs, BVHNode* parent, uint depth)
	{
		if (depth == 0)
			return;

		float half_width = (parent->max_pos.x - parent->min_pos.x) / 2.0f;
		float half_height = (parent->max_pos.y - parent->min_pos.y) / 2.0f;
		uint current_child_index = 0;

		for (float x = 0; x < 2; x++)
			for (float y = 0; y < 2; y++)
			{
				BVHNode* child = new BVHNode();
				child->min_pos.x = parent->min_pos.x + x * half_width;
				child->min_pos.y = parent->min_pos.y + y * half_height;
				child->max_pos.x = parent->min_pos.x + (x + 1) * half_width;
				child->max_pos.y = parent->min_pos.y + (y + 1) * half_height;

				parent->childs[current_child_index] = child;

				if (depth - 1 == 0)
				{
					child->isLeaf = true;
					child->offset = leafs.size() * tris.second.y;
					leafs.push_back(child);
				}
					
				lambda(lambda, leafs, child, depth - 1);
				current_child_index++;
			}
	};

	BVHNode* root = new BVHNode();
	root->min_pos = { 0, 0 };
	root->max_pos = { 5000, 5000 };
	lambda(lambda, leafs, root, depth);

	printf("nb leafs: %i\n", leafs.size());
	printf("nb triangles: %i\n", tris.second.y);

	/// affecter les triangles aux feuilles
	// mémoire des indices des triangles des feuilles
	int* d_all_indices;
	cudaMalloc((void**)&d_all_indices, nb_leafs * tris.second.y * sizeof(int));
	cudaMemset(d_all_indices, -1, nb_leafs * tris.second.y * sizeof(int));

	// mémoire des feuilles
	BVHNode* hleafs, *dleafs;
	cudaMallocHost((void**)&hleafs, nb_leafs * sizeof(BVHNode));

	for (uint i = 0; i < nb_leafs; i++)
		hleafs[i] = *leafs[i];

	cudaMalloc((void**)&dleafs, nb_leafs * sizeof(BVHNode));
	cudaMemcpy(dleafs, hleafs, nb_leafs * sizeof(BVHNode), cudaMemcpyHostToDevice);

	// mémoire des sommets et triangles
	float2* d_vertices = nullptr;
	uint3* d_triangles = nullptr;

	cudaMalloc((void**)&d_vertices, tris.second.x * sizeof(float2));
	cudaMalloc((void**)&d_triangles, tris.second.y * sizeof(uint3));
	cudaMemcpy(d_vertices, tris.first.first, tris.second.x * sizeof(float2), cudaMemcpyHostToDevice);
	cudaMemcpy(d_triangles, tris.first.second, tris.second.y * sizeof(uint3), cudaMemcpyHostToDevice);

	uint nb_blocks = std::ceil((float)tris.second.y / 256); // un thread par triangle
	bvhKernelV1 <<<nb_blocks, 256>>> (dleafs, d_all_indices, d_vertices, d_triangles, tris.second.y, nb_leafs);
	cudaDeviceSynchronize();

	//// copie des données vers le cpu
	int* h_all_indices = new int[nb_leafs * tris.second.y];
	cudaMemcpy(h_all_indices, d_all_indices, nb_leafs * tris.second.y * sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpy(hleafs, dleafs, nb_leafs * sizeof(BVHNode), cudaMemcpyDeviceToHost);

	// libération de la mémoire
	cudaFree(d_all_indices);
	cudaFree(dleafs);
	cudaFree(d_all_indices);
}


__device__ bool isVertexInTriangleBVH(
	std::pair<std::pair<float2*, uint3*>, uint2>& tris,
	int* all_indices,
	BVHNode* parent,
	const float2& pixel
)
{
	BVHNode* current_node = parent;

	while (1)
	{
		for (uint i = 0; i < 4; i++)
		{
			BVHNode* child = current_node->childs[i];

			if (child != nullptr // il y a un enfant
				&& child->count > 0 // l'enfant a des triangles à tester
				&& pixel.x >= child->min_pos.x && pixel.x <= child->max_pos.x // pixel dans la boite englobante
				&& pixel.y >= child->min_pos.y && pixel.y <= child->max_pos.y)
			{
				if (child->isLeaf)
				{
					for (uint k = 0; k < child->count; k++)
					{
						uint tri_index = all_indices[child->offset + k];

						float2 tri[3] = {
							tris.first.first[tris.first.second[tri_index].x],
							tris.first.first[tris.first.second[tri_index].y],
							tris.first.first[tris.first.second[tri_index].z]
						};

						if (checkPointInTriangleFast(pixel, tri))
							return true;
					}
				}
				else
				{
					current_node = child;
					break;
				}
			}
		}
	}

	return false;
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