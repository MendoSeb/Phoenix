#pragma once

#include <gdstk/gdstk.hpp>
#include "clipper2/clipper.h"
#include "clipper.triangulation.h"
#include <thread>
#include <mutex>


using namespace gdstk;
using namespace Clipper2Lib;


namespace Clipper2Utils
{
	///* Fonction de sauvegarde *///
	// convertit les polygones chargés avec gdstk en une suite de polygones Paths64 de clipper2
	Paths64 ConvertGdstkPolygonsToPaths64(Library& lib);

	// Fonction récursive pour récupérer les polygones d'un polytree en polygones gdstk (avec trous)
	void GetGdstkPolygonsFromClipper2Tree(PolyTree64& node, int depth, std::vector<gdstk::Polygon*>& polys, double epsilon);

	// Fonction récursive pour récupérer les polygones d'un polytree sous forme de couches
	// (hauteur de l'arbre 0 = polygones pleins, 1 = trous, 2 = pleins etc...)
	void GetTreeLayers(PolyTree64& node, int depth, std::vector<Paths64>& layers);

	// Convertit des polygones Paths64 en polygones gdstk (Library)
	Library ConvertPaths64ToGdsii(const Paths64& polys, double epsilon);

	// Convertit un polytree64 en une polygones gdstk (avec trous)
	void ConvertPolyTree64ToGdsiiPath(PolyTree64& tree, Library& output, double epsilon);

	// convertit un polytree64 en polygones gdstk triés par couches
	std::vector<Library> ConvertPolyTree64ToGdsiiLayers(PolyTree64& tree);

	///* opérations de base *///
	// Fait l'union des polygones polys
	void MakeUnionPolyTree(const Paths64& polys, PolyTree64& output);

	// Fait un dégraissement des polygones de tree de taille deg
	void MakeDegraissement(const PolyTree64& tree, double deg, PolyTree64& output);

	// Fait la différence entre les polygones de tree et de polys2
	void MakeDifference(const PolyTree64& tree, const PolyTree64& polys2, PolyTree64& output);

	// Fait l'inverse des polygones actuels
	void MakeInverse(const PolyTree64& tree, PolyTree64& output);

	// Triangule les polygones d'un polytree64
	void MakeTriangulationPolyTree(const PolyTree64& tree, Library& output);

	// Triangule les polygones d'un paths64
	void MakeTriangulationPaths(Paths64& paths, Library& output);
};