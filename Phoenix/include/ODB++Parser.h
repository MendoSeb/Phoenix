#pragma once
#include <gdstk/vec.hpp>
#include <vector>
#include <string>
#include <map>
#include <Clipper2Utils.h>
#include "ODBStructs.h"


namespace ODB
{
	// les largeurs/hauteurs/diamètres sont 1 000 fois trop grands par rapport aux positions des points
	const float UNIT_CONVERSION = 1000.0;

	// Parse les géométries définies dans le dossier "symbols/"
	std::map<std::string, std::vector<Polygon*>> readSymbols(std::string folder_path);

	// lis chaque fichier "features" dans le dossier /steps/.../layers/ et les sauvegarde
	std::vector<Library> readLayers(
		std::string folder_path, 
		std::map<std::string, std::vector<Polygon*>>& symbols);

	// parse un fichier feature
	Feature readFeatureFile(const char* file_name);

	/// fonctions pour convertir les objets en polygones
	Cell* convertODBToPolygons(
		Feature& feature, 
		std::map<std::string, std::vector<Polygon*>>& symbols);

	gdstk::Polygon* roundToPolygon(const Round* r);
	gdstk::Polygon* rectangleToPolygon(const Rectangle* rect);
	gdstk::Polygon* donutToPolygon(const Donut* d);

	gdstk::Polygon* arcToPolygon(const Arc* a);
	std::vector<gdstk::Polygon*> surfaceToPolygon(const Surface* s);
	gdstk::Polygon* padToPolygon(const Pad* p, Polygon poly);
	gdstk::Polygon* lineToPolygon(const Line& l);


	/// fonction pour convertir en polygone un symbole tracé le long d'un segment
	// calcule la norme d'un vecteur
	float norme(const Vec2& v);
	float sinusOfVectors(const Vec2& v1, const Vec2& v2);
	float dotProduct(const Vec2& v1, const Vec2& v2);

	// trouve les indices des sommets les plus éloignés à gauche et à droite de segment
	std::pair<size_t, size_t> findFarthestVertices(const Vec2& segment, const Polygon* poly);
	gdstk::Polygon* lineToPolygon(const Line& l, const gdstk::Polygon* poly);
}