#pragma once
#include <array>
#include <gdstk/gdstk.hpp>
#include <vector>
#include <opencv2/core/types.hpp>

using namespace gdstk;

using earcutPoint = std::array<double, 2>;
using earcutPoly = std::vector<std::vector<earcutPoint>>;
using earcutPolys = std::vector<earcutPoly>;
using earcutLayer = std::pair<earcutPolys, std::vector<std::vector<uint32_t>>>;


namespace GdstkUtils
{
	/* Charge un fichier .gds */
	Library LoadGDS(const char* fileName);

	/* Sauvegarde les polygones en .gds */
	void SaveToGdsii(Library& lib, const char* fileName, bool make_fracture);

	/* Save OpenCV polys */
	void SaveToGdsii(std::vector<std::vector<cv::Point2f>>& polys, const char* fileName);

	/* convertit les flex paths(chemins et cercles) en polygones pour faire l'union etc... */
	void ConvertFlexPathsToPolygon(Library& lib);

	void RepeatAndTranslateGdstkNoTransformV2(Library& lib, int rep_x, int rep_y, double width, double height);

	/* Duplique le circuit y copris les flexpath */
	void RepeatAndTranslateGdstkNoTransformV1(Library& lib, int rep_x, int rep_y, double width, double height);

	/* duplique un circuit rep_x* rep_y fois en carré */
	void RepeatAndTranslateGdstk(Library& lib, int rep_x, int rep_y, double width, double height);

	/* normalise les coordonnées des points entre - 1e6 et 1e6 (pour la précision)
	* la normalisation permet de ne pas avoir à choisir un nombre pour scale les coordonnées, car les
	* fichiers .gds ne sont pas forcément représentés pareil( primaire: entre -10 et 10, solder: entre -300000 et 30000) */
	void Normalize(Library& lib, double limit = 1e5);

	/* Divise les polygones ayant plus de 8190 sommets (pour la triangulation et le warning) */
	void MakeFracture(Library& lib);

	/* Applique l'union aux polygones de type gdstk */
	Library MakeUnion(Library& lib);

	/* Applique le dégraissement aux polygones de type gdstk */
	Library MakeDegraissement(Library lib, double dist);

	/* Applique la différence entre deux listes de polygones de type gdstk: (lib1 - lib2) */
	Library MakeDifference(Library& lib1, Library& lib2);
};