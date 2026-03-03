#include "cudaCall.h"
#include <cstdio>
#include <cuda_runtime.h>
#include "Warping.h"
#include <tiffio.h>
#include <device_launch_parameters.h>


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

__device__ bool isVertexInTriangleBVH(
	float2* d_vertices,
	uint3* d_triangles,
	BVHNode* dtree,
	int* d_all_indices,
	int depth,
	uint nb_triangles,
	float2& pixel
)
{
	BVHNode* current_node = &dtree[0];
	int nb_leafs = std::pow(4, depth);

	for (int p = 0; p < depth; p++)
	{
		for (int i = 0; i < 4; i++)
		{
			BVHNode* child = nullptr;

			if (current_node->childs[i] >= 0)
				child = &dtree[current_node->childs[i]];

			if (child
				&& pixel.x >= child->min_pos.x && pixel.x <= child->max_pos.x // pixel dans la boite englobante
				&& pixel.y >= child->min_pos.y && pixel.y <= child->max_pos.y)
			{
				if (p == depth - 1)
				{
					for (int k = 0; k < child->count; k++)
					{
						int tri_index = d_all_indices[child->offset + k];

						if (tri_index >= 0 && tri_index < nb_triangles * nb_leafs)
						{
							float2 tri[3] = {
								d_vertices[d_triangles[tri_index].x],
								d_vertices[d_triangles[tri_index].y],
								d_vertices[d_triangles[tri_index].z]
							};

							if (checkPointInTriangleFast(pixel, tri))
								return true;
						}
					}
					return false;
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


__global__ void rasterization_kernel(
	float2* d_vertices,
	uint3* d_triangles,
	BVHNode* dtree,
	int* d_all_indices,
	size_t nb_triangles,
	uint depth,
	uint2 img_dim,
	unsigned char* img
)
{
	size_t thread_x = blockIdx.x * blockDim.x + threadIdx.x;
	size_t thread_y = blockIdx.y * blockDim.y + threadIdx.y;
	size_t pixel_index = thread_y * img_dim.x + thread_x;

	if (pixel_index < img_dim.x * img_dim.y)
	{
		float2 pixel{ thread_x, thread_y };

		if (isVertexInTriangleBVH(d_vertices, d_triangles, dtree, d_all_indices, depth, nb_triangles, pixel))
			img[pixel_index] = 255;
	}
}



__global__ void rasterization_kernelV2(
	float2* d_vertices,
	uint3* d_triangles,
	size_t nb_triangles,
	uint2 img_dim,
	unsigned char* img
)
{
	size_t tri_id = blockIdx.x * blockDim.x + threadIdx.x;

	if (tri_id >= nb_triangles)
		return;

	float2 tri[3] = {
		{d_vertices[d_triangles[tri_id].x]},
		{d_vertices[d_triangles[tri_id].y]},
		{d_vertices[d_triangles[tri_id].z]}
	};

	float4 bb = getTriangleBoundingBox(tri);

	for (int x = bb.x; x < bb.z; x++)
		for (int y = bb.y; y < bb.w; y++)
		{
			float2 pixel{x, y};

			if (x >= 0 && x < img_dim.x
				&& y >= 0 && y < img_dim.y
				&& checkPointInTriangleFast(pixel, tri))
				img[y * img_dim.x + x] = 255;
		}
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


__global__ void bvhCount(
	CudaCall::BVHNode* dleafs,
	float2* d_vertices,
	uint3* d_triangles,
	uint nb_triangles,
	uint nb_leafs
)
{
	int tri_id = blockIdx.x * blockDim.x + threadIdx.x;

	if (tri_id >= 0 && tri_id < nb_triangles)
	{
		float2 tri[3] = {
			d_vertices[d_triangles[tri_id].x],
			d_vertices[d_triangles[tri_id].y],
			d_vertices[d_triangles[tri_id].z]
		};

		for (uint i = 0; i < nb_leafs; i++)
			if (isTriangleInBoundingBox(tri, dleafs[i].min_pos, dleafs[i].max_pos))
				atomicAdd(&dleafs[i].count, 1);
	}
}


__global__ void bvhAssociate(
	CudaCall::BVHNode* dleafs,
	int* d_all_indices,
	float2* d_vertices,
	uint3* d_triangles,
	uint nb_triangles,
	uint nb_leafs
)
{
	int tri_id = blockIdx.x * blockDim.x + threadIdx.x;

	if (tri_id >= 0 && tri_id < nb_triangles)
	{
		float2 tri[3] = {
			d_vertices[d_triangles[tri_id].x],
			d_vertices[d_triangles[tri_id].y],
			d_vertices[d_triangles[tri_id].z]
		};

		for (uint i = 0; i < nb_leafs; i++)
		{
			if (isTriangleInBoundingBox(tri, dleafs[i].min_pos, dleafs[i].max_pos))
			{
				size_t temp_count = atomicAdd(&dleafs[i].temp_count, 1);
				d_all_indices[dleafs[i].offset + temp_count] = tri_id;
			}
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


// un thread par pixel (avec bvh)
void CudaCall::rasterization(
	std::pair<std::pair<float2*, uint3*>, uint2>& tris,
	std::pair<BVHNode*, int*>& bvh,
	int& depth,
	int nb_indices
)
{
	/*for (uint i = 0; i < nb_indices; i++)
		printf("indices; %i\n", bvh.second[i]);*/

	uint nb_nodes = (std::pow(4, depth + 1) - 1) / 3;
	uint nb_leafs = std::pow(4, depth);

	/*for (uint i = 0; i < tris.second.y; i++)
		printf("tri %i, %f, %f, %f, %f, %f, %f\n", 
			i,
			tris.first.first[tris.first.second[i].x].x, tris.first.first[tris.first.second[i].x].y,
			tris.first.first[tris.first.second[i].y].x, tris.first.first[tris.first.second[i].y].y,
			tris.first.first[tris.first.second[i].z].x, tris.first.first[tris.first.second[i].z].y);

	for (uint i = 0; i < 4; i++)
	{
		BVHNode* child = &bvh.first[bvh.first[0].childs[i]];
		printf("\nleaf id: %i, %i\n", child->id, child->offset);

		for (uint k = 0; k < child->count; k++)
		{
			printf("k: %i, index: %i ", child->offset + k, bvh.second[child->offset + k]);
		}
	}*/

	// allocation mémoire
	cudaStream_t stream;
	cudaStreamCreate(&stream);

	int* d_all_indices = nullptr;
	cudaMallocAsync((void**)&d_all_indices, nb_indices * sizeof(int), stream);
	cudaMemcpyAsync(d_all_indices, bvh.second, nb_indices * sizeof(int), cudaMemcpyHostToDevice, stream);

	BVHNode* dtree = nullptr;
	cudaMallocAsync((void**)&dtree, nb_nodes * sizeof(CudaCall::BVHNode), stream);
	cudaMemcpyAsync(dtree, bvh.first, nb_nodes * sizeof(CudaCall::BVHNode), cudaMemcpyHostToDevice, stream);

	uint2 img_dim = { 10000, 10000 };
	size_t nb_pixels = img_dim.x * img_dim.y;

	// sommets et triangles
	float2* dv = nullptr;
	uint3* dt = nullptr;
	unsigned char* dimg = nullptr;

	std::chrono::steady_clock::time_point s1 = std::chrono::steady_clock::now();

	cudaMallocAsync((void**)&dv, sizeof(float2) * tris.second.x, stream);
	cudaMallocAsync((void**)&dt, sizeof(uint3) * tris.second.y, stream);
	cudaMallocAsync((void**)&dimg, sizeof(unsigned char) * nb_pixels, stream);

	cudaMemcpyAsync(dv, tris.first.first, tris.second.x * sizeof(float2), cudaMemcpyHostToDevice, stream);
	cudaMemcpyAsync(dt, tris.first.second, tris.second.y * sizeof(uint3), cudaMemcpyHostToDevice, stream);

	cudaMemsetAsync(dimg, 0, sizeof(unsigned char) * nb_pixels, stream);

	std::chrono::steady_clock::time_point s2 = std::chrono::steady_clock::now();
	std::cout << "allocation vram en " << std::chrono::duration_cast<std::chrono::milliseconds>(s2 - s1).count() << " ms" << std::endl;

	// call
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	uint nb_x_blocks = std::ceil((float)img_dim.x / 16.0f);
	uint nb_y_blocks = std::ceil((float)img_dim.y / 16.0f);

	rasterization_kernel <<< dim3(nb_x_blocks, nb_y_blocks, 1), dim3(16, 16, 1) >>>
		(dv, dt, dtree, d_all_indices, tris.second.y, depth, img_dim, dimg);

	/*rasterization_kernel << < dim3(1, 1, 1), dim3(1, 1, 1) >> >
		(dv, dt, dtree, d_all_indices, tris.second.y, depth, img_dim, dimg);*/

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
	cudaFree(d_all_indices);
	cudaFreeAsync(dimg, stream);
	cudaStreamDestroy(stream);

	delete img;
}

std::pair<std::pair<BVHNode*, int*>, int> CudaCall::bvhV1(
	std::pair<std::pair<float2*, uint3*>, uint2>& tris, 
	int& depth
)
{
	uint nb_nodes = (std::pow(4, depth + 1) - 1) / 3;
	uint nb_leafs = std::pow(4, depth);

	printf("NB leafs: %i\nNB nodes: %i\nNB triangles: %i\n", nb_leafs, nb_nodes, tris.second.y);

	/// construire l'arbre équilibré (équilibré je crois)
	std::vector<BVHNode*> leafs;
	uint current_node_index = 0;

	BVHNode *tree = nullptr, *dtree = nullptr;
	cudaMallocHost((void**)&tree, nb_nodes * sizeof(BVHNode));

	tree[0].min_pos = { 0, 0 };
	tree[0].max_pos = { 38400, 30000 };

	// fonction récursive pour construire l'arbre
	auto lambda = [&](auto&& lambda, int parent_index, uint current_depth)
	{
		if (current_depth == depth)
			return;

		float half_width = (tree[parent_index].max_pos.x - tree[parent_index].min_pos.x) / 2.0f;
		float half_height = (tree[parent_index].max_pos.y - tree[parent_index].min_pos.y) / 2.0f;
		int current_child_index = 0;

		for (float x = 0; x < 2; x++)
			for (float y = 0; y < 2; y++)
			{
				BVHNode* child = new BVHNode();
				child->min_pos.x = tree[parent_index].min_pos.x + x * half_width;
				child->min_pos.y = tree[parent_index].min_pos.y + y * half_height;
				child->max_pos.x = tree[parent_index].min_pos.x + (x + 1) * half_width;
				child->max_pos.y = tree[parent_index].min_pos.y + (y + 1) * half_height;

				current_node_index++;
				child->id = current_node_index;
				tree[parent_index].childs[current_child_index] = current_node_index;
				tree[current_node_index] = *child;

				if (current_depth + 1 == depth)
				{
					child->isLeaf = true;
					leafs.push_back(child);
				}

				lambda(lambda, current_node_index, current_depth + 1);
				current_child_index++;
			}
	};

	lambda(lambda, 0, 0);

	/*for (uint i = 0; i < nb_nodes; i++)
		printf("node id: %i, child: %i, %i, %i, %i\n",
			tree[i].id, tree[i].childs[0], tree[i].childs[1], tree[i].childs[2], tree[i].childs[3]);*/

	/// affecter les triangles aux feuilles
	// mémoire des feuilles
	BVHNode *hleafs = nullptr;
	BVHNode *dleafs = nullptr;
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

	uint nb_blocks = std::ceil((float)tris.second.y / 256.0f); // un thread par triangle
	bvhCount <<<nb_blocks, 256>>> (dleafs, d_vertices, d_triangles, tris.second.y, nb_leafs);
	cudaDeviceSynchronize();

	//// copie des feuilles modifiées vers le cpu
	cudaMemcpy(hleafs, dleafs, nb_leafs * sizeof(BVHNode), cudaMemcpyDeviceToHost);

	/// passe pour associer les triangles des feuilles dans le tableau global de taille: sum des count
	int nb_indices = 0;

	for (uint i = 0; i < nb_leafs; i++)
	{
		hleafs[i].offset = nb_indices;
		nb_indices += hleafs[i].count;
	}

	for (uint i = 0; i < nb_leafs; i++)
		tree[hleafs[i].id] = hleafs[i];

	std::cout << "nb_indices: " << nb_indices << std::endl;

	cudaMemcpy(dleafs, hleafs, nb_leafs * sizeof(BVHNode), cudaMemcpyDeviceToHost);

	// mémoire des indices des triangles des feuilles
	int* d_all_indices = nullptr;
	cudaMalloc((void**)&d_all_indices, nb_indices * sizeof(int));
	cudaMemset(d_all_indices, -1, nb_indices * sizeof(int));

	bvhAssociate <<<nb_blocks, 256>>> (dleafs, d_all_indices, d_vertices, d_triangles, tris.second.y, nb_leafs);
	cudaDeviceSynchronize();

	int* h_all_indices = nullptr;
	cudaMallocHost((void**)&h_all_indices, nb_indices * sizeof(int));
	cudaMemcpy(h_all_indices, d_all_indices, nb_indices * sizeof(int), cudaMemcpyDeviceToHost);

	/*for (uint i = 0; i < nb_leafs; i++)
	{
		printf("leafs: %i, offset: %i, count: %i, is_leaf: %i\n", 
			hleafs[i].id, hleafs[i].offset, hleafs[i].count, hleafs[i].isLeaf);

		for (uint k = 0; k < hleafs[i].count; k++)
			printf("indices bvh: %i\n", h_all_indices[hleafs[i].offset + k]);

		for (uint k = 0; k < hleafs[i].count; k++)
			if (h_all_indices[hleafs[i].offset + k] < 0)
				printf("erreur chef\n");
	}

	for (uint i = 0; i < nb_nodes; i++)
		printf("node id: %i, child: %i, %i, %i, %i\n", 
			tree[i].id, tree[i].childs[0], tree[i].childs[1], tree[i].childs[2], tree[i].childs[3]);*/

	// libération de la mémoire
	cudaFree(dleafs);
	cudaFree(d_vertices);
	cudaFree(d_triangles);

	return { { tree, h_all_indices }, nb_indices};
}


// un thread par triangle
void CudaCall::rasterizationV2(
	std::pair<std::pair<float2*, uint3*>, uint2>& tris
)
{
	// allocation mémoire
	cudaStream_t stream;
	cudaStreamCreate(&stream);

	uint2 img_dim = { 38400, 30000 };
	size_t nb_pixels = img_dim.x * img_dim.y;

	// sommets et triangles
	float2* dv = nullptr;
	uint3* dt = nullptr;
	unsigned char* dimg = nullptr;

	std::chrono::steady_clock::time_point s1 = std::chrono::steady_clock::now();

	cudaMallocAsync((void**)&dv, sizeof(float2) * tris.second.x, stream);
	cudaMallocAsync((void**)&dt, sizeof(uint3) * tris.second.y, stream);
	cudaMallocAsync((void**)&dimg, sizeof(unsigned char) * nb_pixels, stream);

	cudaMemcpyAsync(dv, tris.first.first, tris.second.x * sizeof(float2), cudaMemcpyHostToDevice, stream);
	cudaMemcpyAsync(dt, tris.first.second, tris.second.y * sizeof(uint3), cudaMemcpyHostToDevice, stream);

	cudaMemsetAsync(dimg, 0, sizeof(unsigned char) * nb_pixels, stream);

	std::chrono::steady_clock::time_point s2 = std::chrono::steady_clock::now();
	std::cout << "allocation vram en " << std::chrono::duration_cast<std::chrono::milliseconds>(s2 - s1).count() << " ms" << std::endl;

	// call
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	uint nb_blocks = std::ceil((float)tris.second.y / 256.0f);

	rasterization_kernelV2 <<< nb_blocks, 256 >>> (dv, dt, tris.second.y, img_dim, dimg);

	cudaDeviceSynchronize();

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Rasterisation en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

	std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
	unsigned char* img = new unsigned char[nb_pixels];
	cudaMemcpy(img, dimg, nb_pixels * sizeof(unsigned char), cudaMemcpyDeviceToHost);

	std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
	std::cout << "copie de l'image gpu -> cpu " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count() << " ms" << std::endl;

	CudaCall::saveToBmp(
		"C:/Users/PC/Desktop/poc/rasterizationV2.bmp",
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



__global__ void TileCount(
	float2* d_vertices,
	uint3* d_triangles,
	int nb_triangles,
	Tile* dtiles,
	int nb_tiles,
	int tile_size,
	int nb_side_tiles
)
{
	int tri_id = blockIdx.x * blockDim.x + threadIdx.x;

	if (tri_id < nb_triangles)
	{
		float2 tri[3] = {
			d_vertices[d_triangles[tri_id].x],
			d_vertices[d_triangles[tri_id].y],
			d_vertices[d_triangles[tri_id].z]
		};

		float4 bb = getTriangleBoundingBox(tri);
			
		// Calculer la plage de tuiles touchée par la boîte englobante
		int min_x = fmaxf(0.0f, floorf(bb.x / tile_size));
		int min_y = fmaxf(0.0f, floorf(bb.y / tile_size));
		int max_x = fminf((float)nb_side_tiles - 1, floorf(bb.z / tile_size));
		int max_y = fminf((float)nb_side_tiles - 1, floorf(bb.w / tile_size));

		for (int x = min_x; x <= max_x; x++) {
			for (int y = min_y; y <= max_y; y++) {
				atomicAdd(&dtiles[x * nb_side_tiles + y].count, 1);
			}
		}
		
		//if (isTriangleInBoundingBox(tri, dtiles[i].min_pos, dtiles[i].max_pos))
	}
}


__global__ void TileAssociate(
	float2* d_vertices,
	uint3* d_triangles,
	int nb_triangles,
	Tile* dtiles,
	int nb_tiles,
	int* d_indices,
	int tile_size,
	int nb_side_tiles
)
{
	int tri_id = blockIdx.x * blockDim.x + threadIdx.x;

	if (tri_id >= 0 && tri_id < nb_triangles)
	{
		float2 tri[3] = {
			d_vertices[d_triangles[tri_id].x],
			d_vertices[d_triangles[tri_id].y],
			d_vertices[d_triangles[tri_id].z]
		};

		float4 bb = getTriangleBoundingBox(tri);

		// Calculer la plage de tuiles touchée par la boîte englobante
		int min_x = fmaxf(0.0f, floorf(bb.x / tile_size));
		int min_y = fmaxf(0.0f, floorf(bb.y / tile_size));
		int max_x = fminf((float)nb_side_tiles - 1, floorf(bb.z / tile_size));
		int max_y = fminf((float)nb_side_tiles - 1, floorf(bb.w / tile_size));

		for (int x = min_x; x <= max_x; x++) {
			for (int y = min_y; y <= max_y; y++) {
				size_t temp_count = atomicAdd(&dtiles[x * nb_side_tiles + y].temp_count, 1);
				d_indices[dtiles[x * nb_side_tiles + y].offset + temp_count] = tri_id;
			}
		}
	}
}


__global__ void rasterization_kernelV3(
	float2* d_vertices,
	uint3* d_triangles,
	int nb_triangles,
	Tile* dtiles,
	int nb_side_tiles,
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
		std::floor((float)thread_x / tile_size) * nb_side_tiles 
		+ std::floor((float)thread_y / tile_size);

	if (tile_index < 0 || tile_index >= (nb_side_tiles * nb_side_tiles)) return;

	Tile tile = dtiles[tile_index];
	float2 pixel{ thread_x, thread_y };

	if (tile.count == 0) return;
	bool pixel_hit = false; // does the pixel touch a triangle?

	// triangles en memoire partagée
	const uint chunk_size = 256;
	int nb_chunk = std::ceil((float)tile.count / (float)chunk_size);
	__shared__ float2 triangles[chunk_size * 3];

	for (int i = 0; i < nb_chunk; i++)
	{
		int index = tile.offset + (i * chunk_size) + thread_local_id;

		if (index >= 0 && index < nb_indices && dindices[index] != -1)
		{
			int tri_id = dindices[index];

			triangles[thread_local_id * 3] = d_vertices[d_triangles[tri_id].x];
			triangles[thread_local_id * 3 + 1] = d_vertices[d_triangles[tri_id].y];
			triangles[thread_local_id * 3 + 2] = d_vertices[d_triangles[tri_id].z];
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
					img[pixel_index] = 255;
					pixel_hit = true;
					break;
				}
			}
		}

		__syncthreads();
	}
}


// un groupe de threads par triangle
void CudaCall::rasterizationV3(
	std::pair<std::pair<float2*, uint3*>, uint2>& tris,
	double scale
)
{
	// allocation mémoire
	cudaStream_t stream;
	cudaStreamCreate(&stream);

	uint2 img_dim = { scale, scale };
	size_t nb_pixels = img_dim.x * img_dim.y;

	int tile_size = 32; // taille en pixel, rapide avec 32, lent avec 256
	int tile_side = std::ceil((float)std::max(img_dim.x, img_dim.y) / tile_size);
	int nb_tiles = tile_side * tile_side;

	printf("Nb triangles: %i\n", tris.second.y);
	printf("tile_side %i\n", tile_side);
	printf("NB tiles: %i\n", nb_tiles);

	/// allocation ram
	Tile* htiles = nullptr;
	cudaMallocHost((void**)&htiles, nb_tiles * sizeof(Tile));

	int tile_index = 0;

	for (int x = 0; x < tile_side; x++)
		for (int y = 0; y < tile_side; y++)
		{
			htiles[tile_index].min_pos = { (float)x * tile_size, (float)y * tile_size };
			htiles[tile_index].max_pos = { (float)(x + 1) * tile_size, (float)(y + 1) * tile_size };
			tile_index++;
		}

	/// allocation vram
	float2* dv = nullptr;
	uint3* dt = nullptr;
	Tile* dtiles = nullptr;
	unsigned char* dimg = nullptr;

	std::chrono::steady_clock::time_point s1 = std::chrono::steady_clock::now();

	cudaMallocAsync((void**)&dv, tris.second.x * sizeof(float2), stream);
	cudaMallocAsync((void**)&dt, tris.second.y * sizeof(uint3), stream);
	cudaMallocAsync((void**)&dtiles, nb_tiles * sizeof(Tile), stream);
	cudaMallocAsync((void**)&dimg, sizeof(unsigned char) * nb_pixels, stream);

	cudaMemcpyAsync(dv, tris.first.first, tris.second.x * sizeof(float2), cudaMemcpyHostToDevice, stream);
	cudaMemcpyAsync(dt, tris.first.second, tris.second.y * sizeof(uint3), cudaMemcpyHostToDevice, stream);
	cudaMemcpyAsync(dtiles, htiles, nb_tiles * sizeof(Tile), cudaMemcpyHostToDevice, stream);
	cudaMemsetAsync(dimg, 0, sizeof(unsigned char) * nb_pixels, stream);

	std::chrono::steady_clock::time_point s2 = std::chrono::steady_clock::now();
	std::cout << "Allocation vram en " << std::chrono::duration_cast<std::chrono::milliseconds>(s2 - s1).count() << " ms" << std::endl;

	/// passe 1: calcul du nombre de triangles dans chaque tuile
	TileCount <<< std::ceil(tris.second.y / 256.0f), 256 >>> 
		(dv, dt, tris.second.y, dtiles, nb_tiles, tile_size, tile_side);
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
	printf("Nb indices: %i\n", indices_array_size);
	printf("Passe 1 faite\n");

	/// passe 2: affectation des triangles dans le tableau d'indices global des tuiles
	TileAssociate <<< std::ceil(tris.second.y / 256.0f), 256 >>> 
		(dv, dt, tris.second.y, dtiles, nb_tiles, dindices, tile_size, tile_side);
	cudaDeviceSynchronize();

	printf("Passe 2 faite\n");

	/// passe 3: un groupe par tuile, un thread par pixel
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	rasterization_kernelV3 
		<<< dim3(std::ceil(img_dim.x / 16.0f), std::ceil(img_dim.y / 16.0f), 1), dim3(16, 16, 1) >>>
		(dv, dt, tris.second.y, dtiles, tile_side, tile_size, dindices, img_dim, dimg, indices_array_size);

	cudaDeviceSynchronize();

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Rasterisation en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	std::cout << "Total en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - s1).count() << " ms" << std::endl;

	std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
	unsigned char* img = new unsigned char[nb_pixels];
	cudaMemcpy(img, dimg, nb_pixels * sizeof(unsigned char), cudaMemcpyDeviceToHost);

	std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
	std::cout << "Copie de l'image gpu -> cpu " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count() << " ms" << std::endl;

	CudaCall::saveToBmp(
		"C:/Users/PC/Desktop/poc/rasterizationV3.bmp",
		img_dim.x,
		img_dim.y,
		img
	);

	// libération de la mémoire
	cudaFreeAsync(dv, stream);
	cudaFreeAsync(dt, stream);
	cudaFreeAsync(dimg, stream);
	cudaFreeAsync(dindices, stream);
	cudaFreeAsync(dtiles, stream);
	cudaStreamDestroy(stream);

	cudaFreeHost(htiles);

	delete[] img;
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