#pragma once
#include <array>
#include <gdstk/gdstk.hpp>
#include <vector>

using namespace gdstk;

using earcutPoint = std::array<double, 2>;
using earcutPolys = std::vector<std::vector<std::vector<earcutPoint>>>;
using earcutLayer = std::pair<earcutPolys, std::vector<std::vector<uint32_t>>>;


namespace GdstkUtils
{
	Library LoadGDS(const char* fileName);

	void SaveToGdsii(Library& lib, const char* fileName);

	// convertit les flex paths (chemins et cercles) en polygones pour faire l'union etc...
	void ConvertFlexPathsToPolygon(Library& lib);

	// duplique un circuit rep_x * rep_y fois en carré
	void RepeatAndTranslateGdstk(Library& lib, int rep_x, int rep_y, double width, double height);

	// normalise les coordonnées des points entre -1e6 et 1e6 (pour la précision)
	// la normalisation permet de ne pas avoir à choisir un nombre pour scale les coordonnées, car les
	// fichiers .gds ne sont pas forcément représentés pareil( primaire: entre -10 et 10, solder: entre -300000 et 30000)
	void Normalize(Library& lib);

	// Découpe les polygones ayant plus de 8190 sommets en plusieurs polygones plus petits
	void MakeFracture(Library& lib);

	Library MakeUnion(Library& lib);

	Library MakeDegraissement(Library lib, double dist);

	Library MakeDifference(Library& lib1, Library& lib2);
};