#include "Clipper2Utils.h"
#include <__msvc_chrono.hpp>
#include <string>
#include "clipper2/clipper.h" // Ajouté pour TriangulatePaths
#include <GdstkUtils.h>
#include <thread>
#include <Utilities.h>


namespace Clipper2Utils
{
	Paths64 ConvertGdstkPolygonsToPaths64(Library& lib)
	{
		assert(lib.cell_array.count == 1);
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		Paths64 paths;
		paths.resize(lib.cell_array[0]->polygon_array.count);

		for (size_t i = 0; i < lib.cell_array[0]->polygon_array.count; i++)
		{
			Polygon* p = lib.cell_array[0]->polygon_array[i];
			paths[i].resize(p->point_array.count);

			for (size_t m = 0; m < p->point_array.count; m++)
				paths[i][m] = Point64((uint64_t)p->point_array[m].x, (uint64_t)p->point_array[m].y);
		}
		
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Polygones convertis en polygones clipper2 en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
		return paths;
	}


	Library ConvertPaths64ToGdsii(const Paths64& polys)
	{
		gdstk::Library lib = {};
		lib.init("library", 1e-6, 1e-9);

		gdstk::Cell* cell = new Cell();
		cell->name = copy_string("FIRST", NULL);
		lib.cell_array.append(cell);

		for (const Path64& poly : polys)
		{
			gdstk::Polygon* p_new = (Polygon*)allocate_clear(sizeof(Polygon));

			for (const Point64& point : poly)
				p_new->point_array.append(Vec2{ (double)point.x, (double)point.y });

			cell->polygon_array.append(p_new);
		}

		return lib;
	}


	void GetTreeLayerGdstkRecursive(PolyTree64& node, std::vector<gdstk::Polygon*>& polys)
	{
		// parcourir l'arbre
		for (size_t i = 0; i < node.Count(); i++)
			GetTreeLayerGdstkRecursive(*node.Child(i), polys);

		// ajouter le polygone plein et ses enfants comme trou
		if (!node.IsHole() && node.Polygon().size() > 0)
		{
			Polygon* poly = (Polygon*)allocate_clear(sizeof(Polygon));
			Path64 poly_parent = node.Polygon();

			// ajouter les polygones enfants au polygone gdstk
			for (size_t k = 0; k < node.Count(); k++)
			{
				Path64 current_path;
				Path64 child_poly = node.Child(k)->Polygon();
				int nb_points_parent = poly_parent.size();
				int nb_points_child = child_poly.size();
				double min_dist = INT32_MAX;
				size_t parent_index = -1;
				size_t child_index = -1;
				int step = 100;

				// trouver les index du parent et enfant des sommets les plus proches
				for (size_t a = 0; a < nb_points_parent; a += step)
					for (size_t b = 0; b < node.Child(k)->Polygon().size(); b++)
					{
						double dist = Distance(poly_parent[a], node.Child(k)->Polygon()[b]);

						if (dist < min_dist)
						{
							min_dist = dist;
							parent_index = a;
							child_index = b;
						}
					}

				// ajouter les sommets du parent jusqu'au parent_index
				for (size_t a = 0; a <= parent_index; a++)
					current_path.push_back(Point64{ poly_parent[a].x, poly_parent[a].y });

				// ajouter les sommets de l'enfant en partant de child_index
				for (size_t a = 0; a < child_poly.size() + 1; a++)
				{
					int temp = (child_index + a) % nb_points_child;
					current_path.push_back(Point64{ child_poly[temp].x, child_poly[temp].y });
				}

				// fermer le contour parent
				for (size_t a = parent_index; a < nb_points_parent; a++)
				{
					int temp = a % nb_points_parent;
					current_path.push_back(Point64{ poly_parent[temp].x, poly_parent[temp].y });
				}

				poly_parent = current_path;
			}

			// convertir current_path en polygone gdstk
			for (const Point64& point : poly_parent)
				poly->point_array.append(Vec2{ (double)point.x, (double)point.y });

			polys.push_back(poly);
		}
	}


	void ConvertPolyTree64ToGdsiiPath(PolyTree64& tree, Library& output)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		output.init("library", 1e-6, 1e-9);

		gdstk::Cell* cell = new Cell();
		cell->name = copy_string("FIRST", NULL);
		output.cell_array.append(cell);

		std::vector<gdstk::Polygon*> polys;
		GetTreeLayerGdstkRecursive(tree, polys);

		Array<gdstk::Polygon*> gdstk_polys = {};

		for (Polygon* p : polys)
			gdstk_polys.append(p);

		cell->polygon_array = gdstk_polys;

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "conversion polytree64 vers gdstk " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
	}


	void GetTreeLayersRecusrive(PolyTree64& node, int depth, std::vector<Paths64>& layers)
	{
		// parcourir les enfants
		for (size_t i = 0; i < node.Count(); i++)
			GetTreeLayersRecusrive(*node.Child(i), depth + 1, layers);

		// enregistrer le polygone dans la bonne couche
		Path64 p = node.Polygon();

		if (layers.size() < depth)
			layers.resize(depth);

		layers[depth - 1].push_back(p);
	}


	void GetTreeLayerEarcutRecursive(PolyTree64& node, earcutPolys& polys)
	{
		// parcourir l'arbre
		for (size_t i = 0; i < node.Count(); i++)
			GetTreeLayerEarcutRecursive(*node.Child(i), polys);

		// ajouter le polygone plein et ses enfants comme trou
		if (!node.IsHole())
		{
			Path64 poly_parent = node.Polygon();
			earcutPoly poly;

			// ajouter le polygone parent
			std::vector<earcutPoint> parent;

			for (Point64& point : poly_parent)
				parent.push_back(earcutPoint{ (double)point.x,(double) point.y });

			poly.push_back(parent);

			// ajouter les polygones enfants au polygone gdstk
			for (size_t k = 0; k < node.Count(); k++)
			{
				std::vector<earcutPoint> child;

				for (const Point64& point : node.Child(k)->Polygon())
					child.push_back(earcutPoint{ (double)point.x,(double)point.y });

				poly.push_back(child);
			}

			polys.push_back(poly);
		}
	}


	std::vector<Library> ConvertPolyTree64ToGdsiiLayers(PolyTree64& polys)
	{
		std::vector<Paths64> paths64_layers;
		GetTreeLayersRecusrive(polys, 1, paths64_layers);

		std::vector<Library> layers;

		for (size_t i = 0; i < paths64_layers.size(); i++)
			layers.push_back(ConvertPaths64ToGdsii(paths64_layers[i]));

		return layers;
	}


	void TriangulateWithoutUnion()
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		Library lib1 = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/Image Primaire V2.gds");
		//Library lib1 = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/0 - Image Solder PHC.gds");
		GdstkUtils::RepeatAndTranslateGdstk(lib1, 4, 3, 12, 12);
		//GdstkUtils::RepeatAndTranslateGdstk(lib1, 4, 3, 300000, 300000);
		GdstkUtils::Normalize(lib1, Vec2{10, 10});
		Paths64 paths = Clipper2Utils::ConvertGdstkPolygonsToPaths64(lib1);

		Library lib = {};
		Clipper2Utils::MakeTriangulationPaths(paths, lib);

		std::vector<Library> layers = { lib };
		Utils::WriteLibraryToObj(layers, "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/solder/triangulation_clipper2_monocouche_v2.obj");

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Triangulation monocouche V2 (sans union) en : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	}


	void MakeUnionPolyTree(const Paths64& polys, PolyTree64& output)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		Clipper64 cd;
		cd.AddSubject(polys);
		cd.Execute(ClipType::Union, FillRule::NonZero, output);

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Union faite en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
	}


	void MakeDegraissement(const PolyTree64& tree, double deg, PolyTree64& output)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		Paths64 input = PolyTreeToPaths64(tree);

		// dégraissement
		ClipperOffset offsetter;
		offsetter.AddPaths(input, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
		offsetter.Execute(deg, output);

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Degraissement fait en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	}


	void MakeDifference(const PolyTree64& polys1, const PolyTree64& polys2, PolyTree64& output)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		Paths64 input1 = PolyTreeToPaths64(polys1);
		Paths64 input2 = PolyTreeToPaths64(polys2);

		Clipper64 cd;
		cd.AddSubject(input1);
		cd.AddClip(input2);
		cd.Execute(ClipType::Difference, FillRule::NonZero, output);

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Difference faite en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	}


	void MakeInverse(const PolyTree64& tree, PolyTree64& output)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		Paths64 input = PolyTreeToPaths64(tree);

		// trouver la boite englobante de tous les polygones pour créer le masque de fond
		Point64 min = Point64(INT64_MAX, INT64_MAX);
		Point64 max = Point64(INT64_MIN, INT64_MIN);

		for (const Path64& poly : input)
		{
			for (const Point64& point : poly)
			{
				min.x = std::min(min.x, point.x);
				min.y = std::min(min.y, point.y);
				max.x = std::max(max.x, point.x);
				max.y = std::max(max.y, point.y);
			}
		}

		// créer le masque à partir des bornes
		Path64 mask;
		mask.push_back(Point64(min.x, min.y));
		mask.push_back(Point64(min.x, max.y));
		mask.push_back(Point64(max.x, max.y));
		mask.push_back(Point64(max.x, min.y));

		Paths64 masks;
		masks.push_back(mask);

		Clipper64 cd;
		cd.AddSubject(masks);
		cd.AddClip(input);
		cd.Execute(ClipType::Difference, FillRule::NonZero, output);

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Inverse fait en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	}


	void MakeTriangulationPolyTree(const PolyTree64& tree, Library& output)
	{
		printf("\nTriangulation Clipper2\n");
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		Paths64 input = PolyTreeToPaths64(tree);
		Paths64 paths_output;

		Triangulate(input, paths_output, false);
		output = ConvertPaths64ToGdsii(paths_output);

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Triangulation faite: " << paths_output.size() << " en: "
			<< std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << std::endl;
	}


	void MakeTriangulationPaths(Paths64& paths, Library& output)
	{
		Paths64 final;

		for (Path64& path : paths)
		{
			Paths64 path_output;
			Paths64 temp;
			temp.push_back(path);

			Triangulate(temp, path_output, true);
			final.insert(final.end(), path_output.begin(), path_output.end());
		}

		output = ConvertPaths64ToGdsii(final);
	}
}


