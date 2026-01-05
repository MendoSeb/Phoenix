#pragma once
#include <gdstk/gdstk.hpp>
#include <vector>

using namespace gdstk;

using earcutPoint = std::array<double, 2>;
using earcutPolys = std::vector<std::vector<std::vector<earcutPoint>>>;
using earcutLayer = std::pair<earcutPolys, std::vector<std::vector<uint32_t>>>;


class GdstkOperators
{
private:

public:
	GdstkOperators();

	Library MakeUnion(Library& lib);

	Library MakeDegraissement(Library lib, double dist);

	Library MakeDifference(Library& lib1, Library& lib2);

};