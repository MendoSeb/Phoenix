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
	
	// gdstk::Library ---> Clipper2Lib::PathsD
	PathsD ConvertGdstkPolygonsToPathsD(Library& lib);

	// gdstk::Library ---> Clipper2Lib::PolyTreeD
	std::unique_ptr<PolyTreeD> ConvertGdstkPolygonsToPolyTreeD(Library& lib);

	// Clipper2Lib::PathsD ---> Clipper2Lib::PolyTreeD
	std::unique_ptr<PolyTreeD> ConvertPathsDToPolyTreeD(const PathsD& paths);

	// Fonction récursive pour récupérer les polygones d'un polytree en polygones gdstk (avec trous)
	void GetTreeLayerGdstkRecursive(PolyTreeD& node,std::vector<gdstk::Polygon*>& polys);

	/* Fonction recusrive pour parcourir l'arbre et extraire les polygones en earcut */
	void GetTreeLayerEarcutRecursive(PolyTreeD& node, earcutPolys& polys);

	// Fonction récursive pour récupérer les polygones d'un polytree sous forme de couches
	// (hauteur de l'arbre 0 = polygones pleins, 1 = trous, 2 = pleins etc...)
	void GetTreeLayersRecursive(PolyTreeD& node, int depth, std::vector<PathsD>& layers);

	// Clipper2Lib::PathsD ---> gdstk::Library
	Library ConvertPathsDToGdsii(const PathsD& polys);

	// Clipper2Lib::PolyTreeD ---> gdstk::Library
	void ConvertPolyTreeDToGdsiiPath(PolyTreeD& tree, Library& output);

	// Clipper2Lib::std::unique_ptr<PolyTreeD> ---> gdstk::Library
	void ConvertPolyTreeDToGdsiiPath2(std::unique_ptr<PolyTreeD>& tree, Library& output);

	// Clipper2Lib::PolyTreeD ---> std::vector<gdstk::Library> (couches selon la polarité)
	std::vector<Library> ConvertPolyTreeDToGdsiiLayers(PolyTreeD& tree);


	/// OPERATIONS SUR LES POLYGONES ///
	/* Triangule les polygones chargés avec gdstk sans faire d'union (pour garder des triangulations simples) */
	void TriangulateWithoutUnion();

	// Union d'un Clipper2::PathsD et retourne un Clipper2::PolyTreeD
	void MakeUnion(const PathsD& polys, PolyTreeD& output);

	// Union entre les polygones d'un même Clipper2::PolyTreeD
	std::unique_ptr<PolyTreeD> MakeUnion(const std::unique_ptr<PolyTreeD>& polys);

	// Union entre les polygones de deux Clipper2::PolyTreeD
	std::unique_ptr<PolyTreeD> MakeUnion(
		const std::unique_ptr<PolyTreeD>& i1, const std::unique_ptr<PolyTreeD>& i2);

	// Fait un dégraissement des polygones de tree de taille deg
	std::unique_ptr<PolyTreeD> MakeDegraissement(const std::unique_ptr<PolyTreeD>& input, double size);

	// Fait la différence entre les polygones de tree et de polys2
	std::unique_ptr<PolyTreeD> MakeDifference(
		const std::unique_ptr<PolyTreeD>& tree, const std::unique_ptr<PolyTreeD>& polys2);

	// Inverse les trous et zones pleins d'un Clipper2::PolyTreeD
	void MakeInverse(const PolyTreeD& tree, PolyTreeD& output);

	// Intersection entre deux Clipper2::PolyTreeD
	std::unique_ptr<PolyTreeD> MakeIntersection(
		const std::unique_ptr<PolyTreeD>& input1, const std::unique_ptr<PolyTreeD>& input2);

	// Transforme les positions y des points en -y (inversion par rapport à 0)
	std::unique_ptr<PolyTreeD> MakeMirrorY(const std::unique_ptr<PolyTreeD>& input);

	// Triangule les polygones d'un PolyTreeD
	void MakeTriangulationPolyTree(const PolyTreeD& tree, Library& output);

	// Triangule les polygones d'un PathsD
	void MakeTriangulationPaths(PathsD& paths, Library& output);
};