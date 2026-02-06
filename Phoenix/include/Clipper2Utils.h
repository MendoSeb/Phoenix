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
	/// FONCTIONS DE CONVERSION ///
	
	// gdstk::Library ---> Clipper2Lib::Paths64
	Paths64 ConvertGdstkPolygonsToPaths64(Library& lib);

	// gdstk::Library ---> Clipper2Lib::PolyTree64
	std::unique_ptr<PolyTree64> ConvertGdstkPolygonsToPolyTree64(Library& lib);

	// Clipper2Lib::Paths64 ---> Clipper2Lib::PolyTree64
	std::unique_ptr<PolyTree64> ConvertPaths64ToPolyTree64(const Paths64& paths);

	// Fonction récursive pour récupérer les polygones d'un polytree en polygones gdstk (avec trous)
	void GetTreeLayerGdstkRecursive(PolyTree64& node,std::vector<gdstk::Polygon*>& polys);

	/* Fonction recusrive pour parcourir l'arbre et extraire les polygones en earcut */
	void GetTreeLayerEarcutRecursive(PolyTree64& node, earcutPolys& polys);

	// Fonction récursive pour récupérer les polygones d'un polytree sous forme de couches
	// (hauteur de l'arbre 0 = polygones pleins, 1 = trous, 2 = pleins etc...)
	void GetTreeLayersRecursive(PolyTree64& node, int depth, std::vector<Paths64>& layers);

	// Clipper2Lib::Paths64 ---> gdstk::Library
	Library ConvertPaths64ToGdsii(const Paths64& polys);

	// Clipper2Lib::PolyTree64 ---> gdstk::Library
	void ConvertPolyTree64ToGdsiiPath(PolyTree64& tree, Library& output);

	// Clipper2Lib::std::unique_ptr<PolyTree64> ---> gdstk::Library
	void ConvertPolyTree64ToGdsiiPath2(std::unique_ptr<PolyTree64>& tree, Library& output);

	// Clipper2Lib::PolyTree64 ---> std::vector<gdstk::Library> (couches selon la polarité)
	std::vector<Library> ConvertPolyTree64ToGdsiiLayers(PolyTree64& tree);


	/// OPERATIONS SUR LES POLYGONES ///
	/* Triangule les polygones chargés avec gdstk sans faire d'union (pour garder des triangulations simples) */
	void TriangulateWithoutUnion();

	// Union d'un Clipper2::Paths64 et retourne un Clipper2::PolyTree64
	void MakeUnion(const Paths64& polys, PolyTree64& output);

	// Union entre les polygones d'un même Clipper2::PolyTree64
	std::unique_ptr<PolyTree64> MakeUnion(const std::unique_ptr<PolyTree64>& polys);

	// Union entre les polygones de deux Clipper2::PolyTree64
	std::unique_ptr<PolyTree64> MakeUnion(
		const std::unique_ptr<PolyTree64>& i1, const std::unique_ptr<PolyTree64>& i2);

	// Fait un dégraissement des polygones de tree de taille deg
	std::unique_ptr<PolyTree64> MakeDegraissement(const std::unique_ptr<PolyTree64>& input, double size);

	// Fait la différence entre les polygones de tree et de polys2
	std::unique_ptr<PolyTree64> MakeDifference(
		const std::unique_ptr<PolyTree64>& tree, const std::unique_ptr<PolyTree64>& polys2);

	// Inverse les trous et zones pleins d'un Clipper2::PolyTree64
	void MakeInverse(const PolyTree64& tree, PolyTree64& output);

	// Intersection entre deux Clipper2::PolyTree64
	std::unique_ptr<PolyTree64> MakeIntersection(
		const std::unique_ptr<PolyTree64>& input1, const std::unique_ptr<PolyTree64>& input2);

	// Transforme les positions y des points en -y (inversion par rapport à 0)
	std::unique_ptr<PolyTree64> MakeMirrorY(const std::unique_ptr<PolyTree64>& input);

	// Triangule les polygones d'un polytree64
	void MakeTriangulationPolyTree(const PolyTree64& tree, Library& output);

	// Triangule les polygones d'un paths64
	void MakeTriangulationPaths(Paths64& paths, Library& output);
};