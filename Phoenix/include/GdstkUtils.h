#pragma once
#include <array>
#include <gdstk/gdstk.hpp>
#include <vector>

using namespace gdstk;

using earcutPoint = std::array<double, 2>; // point 2D
using earcutPoly = std::vector<std::vector<earcutPoint>>; // Polygone
using earcutPolys = std::vector<earcutPoly>; // list de polygones*

// type de retour après triangulation
using earcutLayer = std::pair<earcutPolys, std::vector<std::vector<uint32_t>>>;


namespace GdstkUtils
{
	// Charge un fichier .gds
	Library LoadGDS(const char* fileName);

	// Sauvegarde les polygones en .gds
	void SaveToGdsii(Library& lib, const char* fileName, bool make_fracture);

	// convertit les flex paths(chemins et cercles) en polygones pour faire l'union etc...
	void ConvertFlexPathsToPolygon(Library& lib);

	// duplique un circuit rep_x* rep_y fois avec un offset entre chaque duplication de widthxheight
	void RepeatAndTranslateGdstk(Library& lib, int rep_x, int rep_y, double width, double height);

	// normalise les coordonnées des points entre - 1e6 et 1e6 (pour la précision)
	// la normalisation permet de ne pas avoir à choisir un nombre pour scale les coordonnées, car les
	// fichiers .gds ne sont pas forcément représentés pareil( primaire: entre -10 et 10, solder: entre -300000 et 30000)
	void Normalize(Library& lib, Vec2&& real_dim_cm);

	void Scaling(Library& lib, double& scale);

	// Divise les polygones ayant plus de 8190 sommets (pour la triangulation et le warning)
	void MakeFracture(Library& lib, uint64_t&& max_points = 8190);

	// Applique l'union aux polygones de type gdstk
	Library MakeUnion(Library& lib);

	// Applique le dégraissement aux polygones de type gdstk
	Library MakeDegraissement(Library lib, double dist);

	// Applique la différence entre deux listes de polygones de type gdstk: (lib1 - lib2)
	Library MakeDifference(Library& lib1, Library& lib2);

	// Multiplie les points par "scale"
	void Scale(Library& lib, double scale);

	float FindMinimum(const Library& lib);

	// Set coordonate positive
	void Add(Library& lib, float value);
};