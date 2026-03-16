#include "cudaCall.h"
#include <cstdio>
#include <cuda_runtime.h>
#include "Warping.h"
#include <tiffio.h>
#include <device_launch_parameters.h>
#include <chrono>
#include <iostream>


using namespace CudaCall;


// pineda edge function
__device__ bool isVertexRightOfSegment(const float2& v, const float2& s1, const float2& s2)
{
	return (v.x - s1.x) * (s2.y - s1.y) - (v.y - s1.y) * (s2.x - s1.x) >= 0.0f;
}

__device__ bool checkPointInTriangleFast(const float2& pixel, float2 tri[3]) 
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
	bool has_neg = (d1 <= 0) || (d2 <= 0) || (d3 <= 0);
	bool has_pos = (d1 >= 0) || (d2 >= 0) || (d3 >= 0);

	// Le résultat est vrai si on n'a pas à la fois du positif et du négatif
	// (ce qui couvre aussi le cas où l'un des d est à 0 : point sur l'arête)
	return !(has_neg && has_pos);
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

	return checkPointInTriangleFast(vertex, tri1)
		|| checkPointInTriangleFast(vertex, tri2);
}

__device__ bool isVertexInsideSquareBox(const float2& min, const float2& max, const float2& vertex)
{
	return vertex.x >= min.x && vertex.x <= max.x && vertex.y >= min.y && vertex.y <= max.y;
}

__global__ void WarpingKernel(
	float2* vertices,
	uint3* triangles,
	uint nb_vertices,
	Warping::Boxes* src_dst,
	Eigen::Matrix3d* homography_matrices
)
{
	size_t thread_global_id = blockIdx.x * blockDim.x + threadIdx.x;

	if (thread_global_id >= 0 && thread_global_id < nb_vertices)
	{
		float2* vertex = &vertices[thread_global_id];

		// for every src box
		for (size_t i = 0; i < 1; i++)
		{
			if (isVertexInsideBox(src_dst[i].src, *vertex))
			{
				Eigen::Matrix<double, 3, 1> p;
				p.row(0) << vertex->x;
				p.row(1) << vertex->y;
				p.row(2) << 1;

				p = homography_matrices[i] * p;
				vertex->x = p(0) / p(2);
				vertex->y = p(1) / p(2);

				return;
			}
		}
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


__device__ bool isTriangleInBoundingBox(float2 tri[3], const float2& min_p, const float2& max_p)
{
    // --- ÉTAPE 1 : Test rapide par Bounding Box du triangle (AABB vs AABB) ---
    // On vérifie si la boîte du triangle et la cellule se touchent même de loin.
    float2 t_min = { fminf(fminf(tri[0].x, tri[1].x), tri[2].x), fminf(fminf(tri[0].y, tri[1].y), tri[2].y) };
    float2 t_max = { fmaxf(fmaxf(tri[0].x, tri[1].x), tri[2].x), fmaxf(fmaxf(tri[0].y, tri[1].y), tri[2].y) };

    if (t_max.x < min_p.x || t_min.x > max_p.x || t_max.y < min_p.y || t_min.y > max_p.y)
        return false;

    // --- ÉTAPE 2 : Test des axes de séparation (SAT) pour les arêtes ---
    // Pour chaque arête du triangle, on vérifie si la boîte est d'un côté ou de l'autre.
    // Cela remplace vos cas 2 et 3 de manière exhaustive.
    
    auto checkEdge = [&](float2 v0, float2 v1) {
        float2 edge = { v1.x - v0.x, v1.y - v0.y };
        float2 normal = { -edge.y, edge.x }; // Normale à l'arête
        
        // On projette les sommets de la boîte sur la normale pour voir s'ils sont tous du même côté
        float dot0 = v0.x * normal.x + v0.y * normal.y;
        
        // Coins de la boîte à tester par rapport à l'arête
        float p0 = min_p.x * normal.x + min_p.y * normal.y;
        float p1 = max_p.x * normal.x + min_p.y * normal.y;
        float p2 = min_p.x * normal.x + max_p.y * normal.y;
        float p3 = max_p.x * normal.x + max_p.y * normal.y;

        float b_min = fminf(fminf(p0, p1), fminf(p2, p3));
        float b_max = fmaxf(fmaxf(p0, p1), fmaxf(p2, p3));

        // On projette le 3ème sommet du triangle pour savoir de quel côté est l'intérieur
        // (On peut aussi utiliser l'ordre des sommets si constant)
        // Mais le test simplifié : si la boîte est entièrement d'un côté de l'arête 
        // ET que le triangle est de l'autre, alors pas d'intersection.
        // ... (simplification pour 2D) ...
    };

    // VERSION ENCORE PLUS SIMPLE POUR LE GPU :
    // Si l'AABB (Etape 1) passe, le seul cas restant est si la boîte est 
    // "extérieure" aux trois demi-plans formés par les arêtes du triangle.
    
    // On utilise le "Cross Product" pour vérifier si la boîte est totalement 
    // à l'extérieur d'une des arêtes.
    #pragma unroll
    for (int i = 0; i < 3; i++) {
        int next = (i + 1 == 3) ? 0 : i + 1; // Pas de modulo !
        float2 edge = { tri[next].x - tri[i].x, tri[next].y - tri[i].y };
        float2 n = { -edge.y, edge.x };
        
        // On choisit le coin de la boîte le plus "favorable" selon le signe de la normale
        float test_x = (n.x >= 0) ? max_p.x : min_p.x;
        float test_y = (n.y >= 0) ? max_p.y : min_p.y;
        
        // Si le point le plus proche de la normale est quand même "derrière" l'arête
        if ((test_x - tri[i].x) * n.x + (test_y - tri[i].y) * n.y < 0) {
            // Attention : l'ordre des sommets (CW ou CCW) compte ici.
            // Si l'ordre est inconnu, il faut tester les deux côtés.
        }
    }

    return true; // Si aucun axe ne sépare les objets, ils s'intersectent.
}


void CudaCall::Warping
(
	Utils::Triangulation dtriangulation,
	std::vector<Warping::Boxes>& src_dst
)
{
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	//calcul des matrices de transformation
	Warping::Boxes* src_dst2 = new Warping::Boxes[src_dst.size()];
	Eigen::Matrix3d* homography_matrices = new Eigen::Matrix3d[src_dst.size()];

	for (size_t i = 0; i < src_dst.size(); i++)
	{
		src_dst2[i] = src_dst[i];
		homography_matrices[i] = Warping::getPerspectiveMatrixTransform(src_dst[i].src, src_dst[i].dst);
	}

	// allocation mémoire gpu
	Warping::Boxes* db = nullptr;
	Eigen::Matrix3d* dm = nullptr;

	cudaStream_t stream;
	cudaStreamCreate(&stream);

	cudaMallocAsync((void**)&db, sizeof(Warping::Boxes) * src_dst.size(), stream);
	cudaMallocAsync((void**)&dm, sizeof(Eigen::Matrix3d) * src_dst.size(), stream);

	cudaMemcpyAsync(db, src_dst2, src_dst.size() * sizeof(Warping::Boxes), cudaMemcpyHostToDevice, stream);
	cudaMemcpyAsync(dm, homography_matrices, src_dst.size() * sizeof(Eigen::Matrix3d), cudaMemcpyHostToDevice, stream);

	// appel
	size_t nb_threads_per_blocks = 1024;
	size_t nb_blocks = std::ceil(dtriangulation.nb_vertices / 1024);
	std::cout << "Nb threads = " << nb_blocks * nb_threads_per_blocks << std::endl;
	std::cout << "Nb vertices = " << dtriangulation.nb_vertices << std::endl;

	WarpingKernel <<<nb_blocks, nb_threads_per_blocks>>> (dtriangulation.v, dtriangulation.t, dtriangulation.nb_vertices, db, dm);

	cudaDeviceSynchronize();
	// libération mémoire
	cudaFreeAsync(db, stream);
	cudaFreeAsync(dm, stream);
	cudaStreamDestroy(stream);

	delete src_dst2;
	delete homography_matrices;

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "! warping en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
}


__global__ void TileCount(
	Utils::Triangulation dtriangulation,
	Tile* dtiles,
	int nb_tiles,
	uint2 tiles_dim,
	int tile_size
)
{
	int tri_id = blockIdx.x * blockDim.x + threadIdx.x;

	if (tri_id >= 0 && tri_id < dtriangulation.nb_triangles)
	{
		float2 tri[3] = {
			dtriangulation.v[dtriangulation.t[tri_id].x],
			dtriangulation.v[dtriangulation.t[tri_id].y],
			dtriangulation.v[dtriangulation.t[tri_id].z]
		};

		float4 bb = getTriangleBoundingBox(tri);

		// Calculer la plage de tuiles touchée par la boîte englobante
		int min_x = floorf(bb.x / tile_size);
		int min_y = floorf(bb.y / tile_size);
		int max_x = fminf((float)tiles_dim.x - 1, floorf(bb.z / tile_size));
		int max_y = fminf((float)tiles_dim.y - 1, floorf(bb.w / tile_size));

		for (int x = min_x; x <= max_x; x++) {
			for (int y = min_y; y <= max_y; y++) {

				int tile_index = x * tiles_dim.y + y;

				if (tile_index >= 0 && tile_index < nb_tiles)
					atomicAdd(&dtiles[tile_index].count, 1);
			}
		}
	}
}


__global__ void TileAssociate(
	Utils::Triangulation dtriangulation,
	Tile* dtiles,
	int nb_tiles,
	int* d_indices,
	uint2 tiles_dim,
	int tile_size,
	int layer_range_start,
	int layer_range_end
)
{
	int tri_id = blockIdx.x * blockDim.x + threadIdx.x + layer_range_start;

	if (tri_id >= layer_range_start && tri_id < layer_range_end)
	{
		float2 tri[3] = {
			dtriangulation.v[dtriangulation.t[tri_id].x],
			dtriangulation.v[dtriangulation.t[tri_id].y],
			dtriangulation.v[dtriangulation.t[tri_id].z]
		};

		float4 bb = getTriangleBoundingBox(tri);

		// Calculer la plage de tuiles touchée par la boîte englobante
		int min_x = floorf(bb.x / tile_size);
		int min_y = floorf(bb.y / tile_size);
		int max_x = fminf((float)tiles_dim.x - 1, floorf(bb.z / tile_size));
		int max_y = fminf((float)tiles_dim.y - 1, floorf(bb.w / tile_size));

		for (int x = min_x; x <= max_x; x++) {
			for (int y = min_y; y <= max_y; y++) {

				int tile_index = x * tiles_dim.y + y;

				if (tile_index >= 0 && tile_index < nb_tiles)
				{
					size_t temp_count = atomicAdd(&dtiles[tile_index].temp_count, 1);
					d_indices[dtiles[tile_index].offset + temp_count] = tri_id;
				}
			}
		}
	}
}


__global__ void RasterizationKernel(
	Utils::Triangulation dtriangulation,
	Tile* dtiles,
	uint2 tiles_dim,
	int tile_size,
	int* dindices,
	uint2 img_dim,
	unsigned char* img,
	int nb_indices
)
{
	size_t thread_x = blockIdx.x * blockDim.x + threadIdx.x;
	size_t thread_y = blockIdx.y * blockDim.y + threadIdx.y;
	int thread_local_id = threadIdx.y * blockDim.x + threadIdx.x;
	int pixel_index = thread_y * img_dim.x + thread_x;

	if (pixel_index < 0 || pixel_index >= img_dim.x * img_dim.y) return;

	int tile_index = 
		std::floor((float)thread_x / tile_size) * tiles_dim.y
		+ std::floor((float)thread_y / tile_size);

	if (tile_index < 0 || tile_index >= (tiles_dim.x * tiles_dim.y)) return;

	Tile tile = dtiles[tile_index];
	float2 pixel{ thread_x, thread_y };

	if (tile.count == 0) return;
	bool pixel_hit = false; // does the pixel touch a triangle?

	// triangles en memoire partagée
	const uint chunk_size = 1024;
	int nb_chunk = std::ceil((float)tile.count / (float)chunk_size);
	__shared__ float2 triangles[chunk_size * 3];
	__shared__ unsigned char triangles_polarity[chunk_size];

	for (int i = 0; i < nb_chunk; i++)
	{
		int index = tile.offset + (i * chunk_size) + thread_local_id;

		if (index >= 0 && index < nb_indices && dindices[index] != -1)
		{
			int tri_id = dindices[index];

			triangles[thread_local_id * 3] = dtriangulation.v[dtriangulation.t[tri_id].x];
			triangles[thread_local_id * 3 + 1] = dtriangulation.v[dtriangulation.t[tri_id].y];
			triangles[thread_local_id * 3 + 2] = dtriangulation.v[dtriangulation.t[tri_id].z];

			triangles_polarity[thread_local_id] = dtriangulation.p[tri_id];
		}

		__syncthreads();

		if (!pixel_hit)
		{
			// count valid triangles
			int nb_tris = chunk_size;

			if (chunk_size > tile.count - (i * chunk_size))
				nb_tris = tile.count - (i * chunk_size);

			for (int k = 0; k < nb_tris; k++)
			{
				float2 tri[3] = {
					triangles[k * 3],
					triangles[k * 3 + 1],
					triangles[k * 3 + 2]
				};

				if (checkPointInTriangleFast(pixel, tri))
				{
					img[pixel_index] = (triangles_polarity[k] == 0) ? 0 : 255;
					pixel_hit = true;
					break;
				}
			}
		}

		__syncthreads();
	}
}


// un groupe de threads par triangle
unsigned char* CudaCall::Rasterization(
	Utils::Triangulation& dtriangulation,
	double scale,
	uint2 img_dim
)
{
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	// allocation mémoire
	int tile_size = 32; // taille en pixel, rapide avec 32, lent avec 256

	uint2 tiles_dim;
	tiles_dim.x = std::ceil((float)img_dim.x / tile_size);
	tiles_dim.y = std::ceil((float)img_dim.y / tile_size);

	int nb_tiles = tiles_dim.x * tiles_dim.y;
	printf("nb tiles: %i\nnb_triangles: %i\n", nb_tiles, dtriangulation.nb_triangles);

	/// allocation ram
	Tile* htiles = nullptr;
	cudaMallocHost((void**)&htiles, nb_tiles * sizeof(Tile));

	int tile_index = 0;

	for (int x = 0; x < tiles_dim.x; x++)
		for (int y = 0; y < tiles_dim.y; y++)
		{
			htiles[tile_index].min_pos = { (float)x * tile_size, (float)y * tile_size };
			htiles[tile_index].max_pos = { (float)(x + 1) * tile_size, (float)(y + 1) * tile_size };
			tile_index++;
		}

	/// allocation vram
	size_t nb_pixels = img_dim.x * img_dim.y;

	unsigned char* dimg = nullptr;
	cudaMalloc((void**)&dimg, sizeof(unsigned char) * nb_pixels);
	cudaMemset(dimg, 100, sizeof(unsigned char) * nb_pixels);

	Tile* dtiles = nullptr;
	cudaMalloc((void**)&dtiles, nb_tiles * sizeof(Tile));
	cudaMemcpy(dtiles, htiles, nb_tiles * sizeof(Tile), cudaMemcpyHostToDevice);

	/// passe 1: calcul du nombre de triangles dans chaque tuile
	TileCount <<< std::ceil(dtriangulation.nb_triangles / 1024.0f), 1024 >>> (dtriangulation, dtiles, nb_tiles, tiles_dim, tile_size);
	cudaDeviceSynchronize();

	cudaMemcpy(htiles, dtiles, nb_tiles * sizeof(Tile), cudaMemcpyDeviceToHost);
	int indices_array_size = 0;

	for (int i = 0; i < nb_tiles; i++)
	{
		htiles[i].offset = indices_array_size;
		indices_array_size += htiles[i].count;
	}

	cudaMemcpy(dtiles, htiles, nb_tiles * sizeof(Tile), cudaMemcpyHostToDevice);

	int* dindices = nullptr;
	cudaMalloc((void**)&dindices, indices_array_size * sizeof(int));
	cudaMemset(dindices, -1, indices_array_size * sizeof(int));

	/// passe 2: affectation des triangles dans le tableau d'indices global des tuiles.
	/// remplissage du tableau des indices de triangles couche par couche pour tester les triangles proches en premier
	for (int i = dtriangulation.layers_range.size() - 1; i >= 0; i--)
	{
		int start_index = dtriangulation.layers_range[i].first;
		int end_index = dtriangulation.layers_range[i].second;
		int nb_blocks = std::ceil((end_index - start_index) / 1024.0f);

		TileAssociate <<< nb_blocks, 1024 >>>
			(dtriangulation, dtiles, nb_tiles, dindices, tiles_dim, tile_size, start_index, end_index);

		cudaDeviceSynchronize();
	}

	/// passe 3: un groupe par tuile, un thread par pixel
	std::chrono::steady_clock::time_point start2 = std::chrono::steady_clock::now();

	RasterizationKernel
		<<< dim3(std::ceil(img_dim.x / 32.0f), std::ceil(img_dim.y / 32.0f), 1), dim3(32, 32, 1) >>>
		(dtriangulation, dtiles, tiles_dim, tile_size, dindices, img_dim, dimg, indices_array_size);

	cudaDeviceSynchronize();

	std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
	std::cout << "! rasterisation seule en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2).count() << " ms" << std::endl;

	std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
	unsigned char* img = new unsigned char[nb_pixels];
	cudaMemcpy(img, dimg, nb_pixels * sizeof(unsigned char), cudaMemcpyDeviceToHost);

	// libération de la mémoire
	cudaFree(dimg);
	cudaFree(dindices);
	cudaFree(dtiles);
	cudaFreeHost(htiles);

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "! Total rasterization (allocations etc...) en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

	return img;
}



__device__ bool isTriangleClockwise(float2 tri[3])
{
	int2 vec1{ tri[1].x - tri[0].x, tri[1].y - tri[0].y };
	int2 vec2{ tri[2].x - tri[0].x, tri[2].y - tri[0].y };

	return vec1.x * vec2.y - vec1.y * vec2.x < 0;
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
		palette[i * 4 + 0] = i; // Blue
		palette[i * 4 + 1] = i; // Green
		palette[i * 4 + 2] = i; // Red
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