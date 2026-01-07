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
	/* Convertit les polygones gdstk en polygones boostGeometry */
	multi_polygon_t ConvertGdstkToBoostPolygon(const Library& lib);

	/* Convertit les polygones boostGeometry en polygones gdstk */
	void ConvertBoostPolygonToGdstk(multi_polygon_t& polys, const char* fileName);

	/* Fait l'union d'une liste de polygone (lent) */
	multi_polygon_t MakeUnion(const multi_polygon_t& polys);

	/* Applique le dégraissement à une liste de polygones */
	void MakeDegraissement(int buffer_dist, const char* savePath);

	/* Applique la différence à une liste de polygones */
	void MakeDifference();
};