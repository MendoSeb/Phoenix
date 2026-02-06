#pragma once

#include <gdstk/gdstk.hpp>
#include "clipper2/clipper.h"
#include "clipper.triangulation.h"
#include <thread>
#include <mutex>
#include "GdstkUtils.h"


using namespace gdstk;
using namespace Clipper2Lib;


namespace Clipper2Utils
{
	///* Fonction de sauvegarde *///
	// convertit les polygones chargés avec gdstk en une suite de polygones Paths64 de clipper2
	Paths64 ConvertGdstkPolygonsToPaths64(Library& lib);

	std::unique_ptr<PolyTree64> ConvertGdstkPolygonsToPolyTree64(Library& lib);

	std::unique_ptr<PolyTree64> ConvertPaths64ToPolyTree64(const Paths64& paths);

	// Fonction récursive pour récupérer les polygones d'un polytree en polygones gdstk (avec trous)
	void GetTreeLayerGdstkRecursive(PolyTree64& node,std::vector<gdstk::Polygon*>& polys);

	/* Fonction recusrive pour parcourir l'arbre et extraire les polygones en earcut */
	void GetTreeLayerEarcutRecursive(PolyTree64& node, earcutPolys& polys);

	// Fonction récursive pour récupérer les polygones d'un polytree sous forme de couches
	// (hauteur de l'arbre 0 = polygones pleins, 1 = trous, 2 = pleins etc...)
	void GetTreeLayersRecursive(PolyTree64& node, int depth, std::vector<Paths64>& layers);

	// Convertit des polygones Paths64 en polygones gdstk (Library)
	Library ConvertPaths64ToGdsii(const Paths64& polys);

	// Convertit un polytree64 en une polygones gdstk (avec trous)
	void ConvertPolyTree64ToGdsiiPath(PolyTree64& tree, Library& output);

	void ConvertPolyTree64ToGdsiiPath2(std::unique_ptr<PolyTree64>& tree, Library& output);

	// convertit un polytree64 en polygones gdstk triés par couches
	std::vector<Library> ConvertPolyTree64ToGdsiiLayers(PolyTree64& tree);

	/* Triangule les polygones chargés avec gdstk sans faire d'union (pour garder des triangulations simples) */
	void TriangulateWithoutUnion();

	///* opérations de base *///
	// Fait l'union des polygones polys
	void MakeUnionPolyTree(const Paths64& polys, PolyTree64& output);

	std::unique_ptr<PolyTree64> MakeUnion(const std::unique_ptr<PolyTree64>& polys);

	std::unique_ptr<PolyTree64> MakeUnion(
		const std::unique_ptr<PolyTree64>& i1, const std::unique_ptr<PolyTree64>& i2);

	// Fait un dégraissement des polygones de tree de taille deg
	std::unique_ptr<PolyTree64> MakeDegraissement(const std::unique_ptr<PolyTree64>& input, double size);

	// Fait la différence entre les polygones de tree et de polys2
	std::unique_ptr<PolyTree64> MakeDifference(
		const std::unique_ptr<PolyTree64>& tree, const std::unique_ptr<PolyTree64>& polys2);

	// Fait l'inverse des polygones actuels
	void MakeInverse(const PolyTree64& tree, PolyTree64& output);

	std::unique_ptr<PolyTree64> MakeIntersection(
		const std::unique_ptr<PolyTree64>& input1, const std::unique_ptr<PolyTree64>& input2);

	std::unique_ptr<PolyTree64> MakeMirrorY(const std::unique_ptr<PolyTree64>& input);

	// Triangule les polygones d'un polytree64
	void MakeTriangulationPolyTree(const PolyTree64& tree, Library& output);

	// Triangule les polygones d'un paths64
	void MakeTriangulationPaths(Paths64& paths, Library& output);
};