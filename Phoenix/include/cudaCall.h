#pragma once
#include <vector>
#include <GdstkUtils.h>
#include <Warping.h>


namespace CudaCall
{
	void warping(
		std::pair<std::pair<float2*, uint3*>, uint2>& tris,
		std::vector<Warping::Boxes>& src_dst
	);
}