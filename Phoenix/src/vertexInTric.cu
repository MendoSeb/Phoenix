#include "cudaCall.h"
#include <cstdio>
#include <cuda_runtime.h>
#include "cublas_v2.h"
#include "Warping.h"


__device__ void normalize(float2& v)
{
	double n = norm3d(v.x, v.y, 0.0);
	v.x /= n;
	v.y /= n;
}


__device__ bool isVertexLeftOfSegment(float2* vertex, float2* s1, float2* s2)
{
	float2 v1{vertex->x - s1->x, vertex->y - s1->y};
	float2 v2{s2->x - s1->x, s2->y - s1->y};

	normalize(v1);
	normalize(v2);

	return (v1.x * v2.y - v1.y * v2.x) > 0;
}


// les triangles sont dans le sens horaire
__device__ bool isVertexInsideTriangle(float2* vertex, float2 tri[3])
{
	return isVertexLeftOfSegment(vertex, &tri[0], &tri[1])
		&& isVertexLeftOfSegment(vertex, &tri[1], &tri[2])
		&& isVertexLeftOfSegment(vertex, &tri[2], &tri[0]);
}


// les boites doivent être dans le sens horaire
__device__ bool isVertexInsideBox(const double2 box[4], float2* vertex)
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


__global__ void kernel(
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


void CudaCall::warping(
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

	cudaMalloc((void**)&dv, sizeof(float2) * tris.second.x);
	cudaMalloc((void**)&dt, sizeof(uint3) * tris.second.y);
	cudaMalloc((void**)&db, sizeof(Warping::Boxes) * src_dst.size());
	cudaMalloc((void**)&dm, sizeof(Eigen::Matrix3d) * src_dst.size());

	cudaMemcpy(dv, tris.first.first, tris.second.x * sizeof(float2), cudaMemcpyHostToDevice);
	cudaMemcpy(dt, tris.first.second, tris.second.y * sizeof(uint3), cudaMemcpyHostToDevice);
	cudaMemcpy(db, src_dst2, src_dst.size() * sizeof(Warping::Boxes), cudaMemcpyHostToDevice);
	cudaMemcpy(dm, homography_matrices, src_dst.size() * sizeof(Eigen::Matrix3d), cudaMemcpyHostToDevice);

	// appel
	size_t nb_blocks = std::floor(tris.second.x / 1024);
	size_t nb_threads_per_blocks = 1024;
	std::cout << "Nb threads = " << nb_blocks * nb_threads_per_blocks << std::endl;
	std::cout << "Nb vertices = " << tris.second.x << std::endl;

	kernel <<<nb_blocks, nb_threads_per_blocks>>> (dv, dt, db, dm);
	//kernel <<<1, 3>>> (dv, dt, db, dm);
	cudaDeviceSynchronize();

	// coppie du résultat vers les sommets
	cudaMemcpy(tris.first.first, dv, tris.second.x * sizeof(float2), cudaMemcpyDeviceToHost);

	// libération mémoire
	cudaFree(dv);
	cudaFree(dt);
	cudaFree(db);
	cudaFree(dm);

	delete src_dst2;
	delete homography_matrices;
}