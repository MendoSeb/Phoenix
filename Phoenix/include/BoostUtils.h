#pragma once
#include <gdstk/gdstk.hpp>
#include <boost/geometry.hpp>


using namespace gdstk;
namespace bg = boost::geometry;

using point_t = bg::model::point<double, 2, bg::cs::cartesian>;
using polygon_t = bg::model::polygon<point_t>;
using multi_polygon_t = bg::model::multi_polygon<polygon_t>;


namespace BoostUtils
{
	multi_polygon_t ConvertGdstkToBoostPolygon(const Library& lib);

	void ConvertBoostPolygonToGdstk(multi_polygon_t& polys, const char* fileName);

	multi_polygon_t MakeUnion(const multi_polygon_t& polys);

	void MakeDegraissement(int buffer_dist, const char* savePath);

	void MakeDifference();
};