#include "Clipper2Utils.h"
#include <__msvc_chrono.hpp>
#include <string>
#include "clipper2/clipper.h" // Ajouté pour TriangulatePaths
#include <GdstkUtils.h>
#include <thread>
#include <Utilities.h>


namespace Clipper2Utils
{
	PathsD ConvertGdstkPolygonsToPathsD(Library& lib)
	{
		assert(lib.cell_array.count == 1);
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		PathsD paths;
		paths.resize(lib.cell_array[0]->polygon_array.count);

		for (size_t i = 0; i < lib.cell_array[0]->polygon_array.count; i++)
		{
			Polygon* p = lib.cell_array[0]->polygon_array[i];
			paths[i].resize(p->point_array.count);

			for (size_t m = 0; m < p->point_array.count; m++)
			{
				paths[i][m] = PointD(p->point_array[m].x, p->point_array[m].y);
				//std::cout << p->point_array[m].x << ", " << (uint64_t)p->point_array[m].x << std::endl;
			}
		}
		
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Polygones convertis en polygones clipper2 en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
		return paths;
	}


	std::unique_ptr<PolyTreeD> ConvertGdstkPolygonsToPolyTreeD(Library& lib)
	{
		auto output = std::make_unique<PolyTreeD>();
		PathsD paths = ConvertGdstkPolygonsToPathsD(lib);

		for (size_t i = 0; i < paths.size(); i++)
			output->AddChild(paths[i]);

		return output;
	}


	Library ConvertPathsDToGdsii(const PathsD& polys)
	{
		gdstk::Library lib = {};
		lib.init("library", 1e-6, 1e-9);

		gdstk::Cell* cell = new Cell();
		cell->name = copy_string("FIRST", NULL);
		lib.cell_array.append(cell);

		for (const PathD& poly : polys)
		{
			gdstk::Polygon* p_new = (Polygon*)allocate_clear(sizeof(Polygon));

			for (const PointD& point : poly)
				p_new->point_array.append(Vec2{ (double)point.x, (double)point.y });

			cell->polygon_array.append(p_new);
		}

		return lib;
	}


	std::unique_ptr<PolyTreeD> ConvertPathsDToPolyTreeD(const PathsD& paths)
	{
		std::unique_ptr<PolyTreeD> output = std::make_unique<PolyTreeD>();

		for (size_t i = 0; i < paths.size(); i++)
			output->AddChild(paths[i]);

		return output;
	}


	void GetTreeLayerGdstkRecursive(PolyTreeD& node, std::vector<gdstk::Polygon*>& polys)
	{
		// parcourir l'arbre
		for (size_t i = 0; i < node.Count(); i++)
			GetTreeLayerGdstkRecursive(*node.Child(i), polys);

		// ajouter le polygone plein et ses enfants comme trou
		if (!node.IsHole() && node.Polygon().size() > 0)
		{
			Polygon* poly = (Polygon*)allocate_clear(sizeof(Polygon));
			PathD poly_parent = node.Polygon();

			// ajouter les polygones enfants au polygone gdstk
			for (size_t k = 0; k < node.Count(); k++)
			{
				PathD current_path;
				PathD child_poly = node.Child(k)->Polygon();
				int nb_points_parent = poly_parent.size();
				int nb_points_child = child_poly.size();
				double min_dist = INT32_MAX;
				size_t parent_index = -1;
				size_t child_index = -1;
				int step = 10; // 100

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
					current_path.push_back(PointD{ poly_parent[a].x, poly_parent[a].y });

				// ajouter les sommets de l'enfant en partant de child_index
				for (size_t a = 0; a < child_poly.size() + 1; a++)
				{
					int temp = (child_index + a) % nb_points_child;
					current_path.push_back(PointD{ child_poly[temp].x, child_poly[temp].y });
				}

				// fermer le contour parent
				for (size_t a = parent_index; a < nb_points_parent; a++)
				{
					int temp = a % nb_points_parent;
					current_path.push_back(PointD{ poly_parent[temp].x, poly_parent[temp].y });
				}

				poly_parent = current_path;
			}

			// convertir current_path en polygone gdstk
			for (const PointD& point : poly_parent)
				poly->point_array.append(Vec2{ point.x, point.y });

			polys.push_back(poly);
		}
	}


	void ConvertPolyTreeDToGdsiiPath(PolyTreeD& tree, Library& output)
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
		std::cout << "conversion PolyTreeD vers gdstk " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
	}


	void ConvertPolyTreeDToGdsiiPath2(std::unique_ptr<PolyTreeD>& tree, Library& output)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		output.init("library", 1e-6, 1e-9);

		gdstk::Cell* cell = new Cell();
		cell->name = copy_string("FIRST", NULL);
		output.cell_array.append(cell);

		std::vector<gdstk::Polygon*> polys;
		GetTreeLayerGdstkRecursive(*tree, polys);

		Array<gdstk::Polygon*> gdstk_polys = {};

		for (Polygon* p : polys)
			gdstk_polys.append(p);

		cell->polygon_array = gdstk_polys;

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "conversion PolyTreeD vers gdstk " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
	}


	void GetTreeLayersRecusrive(PolyTreeD& node, int depth, std::vector<PathsD>& layers)
	{
		// parcourir les enfants
		for (size_t i = 0; i < node.Count(); i++)
			GetTreeLayersRecusrive(*node.Child(i), depth + 1, layers);

		// enregistrer le polygone dans la bonne couche
		PathD p = node.Polygon();

		if (layers.size() < depth)
			layers.resize(depth);

		layers[depth - 1].push_back(p);
	}


	void GetTreeLayerEarcutRecursive(PolyTreeD& node, earcutPolys& polys)
	{
		// parcourir l'arbre
		for (size_t i = 0; i < node.Count(); i++)
			GetTreeLayerEarcutRecursive(*node.Child(i), polys);

		// ajouter le polygone plein et ses enfants comme trou
		if (!node.IsHole())
		{
			PathD poly_parent = node.Polygon();
			earcutPoly poly;

			// ajouter le polygone parent
			std::vector<earcutPoint> parent;

			for (PointD& point : poly_parent)
				parent.push_back(earcutPoint{ (double)point.x,(double) point.y });

			poly.push_back(parent);

			// ajouter les polygones enfants au polygone gdstk
			for (size_t k = 0; k < node.Count(); k++)
			{
				std::vector<earcutPoint> child;

				for (const PointD& point : node.Child(k)->Polygon())
					child.push_back(earcutPoint{ (double)point.x,(double)point.y });

				poly.push_back(child);
			}

			polys.push_back(poly);
		}
	}


	std::vector<Library> ConvertPolyTreeDToGdsiiLayers(PolyTreeD& polys)
	{
		std::vector<PathsD> PathsD_layers;
		GetTreeLayersRecusrive(polys, 1, PathsD_layers);

		std::vector<Library> layers;

		for (size_t i = 0; i < PathsD_layers.size(); i++)
			layers.push_back(ConvertPathsDToGdsii(PathsD_layers[i]));

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
		PathsD paths = Clipper2Utils::ConvertGdstkPolygonsToPathsD(lib1);

		Library lib = {};
		Clipper2Utils::MakeTriangulationPaths(paths, lib);

		std::vector<Library> layers = { lib };
		Utils::WriteLibraryToObj(layers, "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/solder/triangulation_clipper2_monocouche_v2.obj");

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Triangulation monocouche V2 (sans union) en : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	}


	void MakeUnion(const PathsD& polys, PolyTreeD& output)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		ClipperD cd;
		cd.AddSubject(polys);
		cd.Execute(ClipType::Union, FillRule::NonZero, output);

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		//std::cout << "Union faite en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	}


	std::unique_ptr<PolyTreeD> MakeUnion(const std::unique_ptr<PolyTreeD>& polys)
	{
		auto output = std::make_unique<PolyTreeD>();

		ClipperD cd;
		cd.AddSubject(PolyTreeToPathsD(*polys));
		cd.Execute(ClipType::Union, FillRule::NonZero, *output);

		return output;
	}


	std::unique_ptr<PolyTreeD> MakeUnion(
		const std::unique_ptr<PolyTreeD>& i1, const std::unique_ptr<PolyTreeD>& i2)
	{
		auto output = std::make_unique<PolyTreeD>();

		ClipperD cd;
		cd.AddSubject(PolyTreeToPathsD(*i1));
		cd.AddSubject(PolyTreeToPathsD(*i2));
		cd.Execute(ClipType::Union, FillRule::NonZero, *output);

		return output;
	}


	std::unique_ptr<PolyTreeD> MakeDegraissement(const std::unique_ptr<PolyTreeD>& input, double deg)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		std::unique_ptr<PolyTreeD> output = std::make_unique<PolyTreeD>();
		PathsD paths = PolyTreeToPathsD(*input);

		PathsD p;

		// dégraissement
		ClipperOffset offsetter;
		//offsetter.AddPaths(paths, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
		//offsetter.Execute(deg, *output);

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Degraissement fait en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

		return output;
	}


	std::unique_ptr<PolyTreeD> MakeDifference
	(
		const std::unique_ptr<PolyTreeD>& polys1, 
		const std::unique_ptr<PolyTreeD>& polys2
	)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		PathsD input1 = PolyTreeToPathsD(*polys1);
		PathsD input2 = PolyTreeToPathsD(*polys2);
		std::unique_ptr<PolyTreeD> output = std::make_unique<PolyTreeD>();

		ClipperD cd;
		cd.AddSubject(input1);
		cd.AddClip(input2);
		cd.Execute(ClipType::Difference, FillRule::NonZero, *output);

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Difference faite en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	
		return output;
	}


	void MakeInverse(const PolyTreeD& tree, PolyTreeD& output)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		PathsD input = PolyTreeToPathsD(tree);

		// trouver la boite englobante de tous les polygones pour créer le masque de fond
		PointD min = PointD(INT64_MAX, INT64_MAX);
		PointD max = PointD(INT64_MIN, INT64_MIN);

		for (const PathD& poly : input)
		{
			for (const PointD& point : poly)
			{
				min.x = std::min(min.x, point.x);
				min.y = std::min(min.y, point.y);
				max.x = std::max(max.x, point.x);
				max.y = std::max(max.y, point.y);
			}
		}

		// créer le masque à partir des bornes
		PathD mask;
		mask.push_back(PointD(min.x, min.y));
		mask.push_back(PointD(min.x, max.y));
		mask.push_back(PointD(max.x, max.y));
		mask.push_back(PointD(max.x, min.y));

		PathsD masks;
		masks.push_back(mask);

		ClipperD cd;
		cd.AddSubject(masks);
		cd.AddClip(input);
		cd.Execute(ClipType::Difference, FillRule::NonZero, output);

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Inverse fait en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	}


	std::unique_ptr<PolyTreeD> MakeIntersection
	(
		const std::unique_ptr<PolyTreeD>& input1,
		const std::unique_ptr<PolyTreeD>& input2
	)
	{
		std::unique_ptr<PolyTreeD> output = std::make_unique<PolyTreeD>();
		PathsD paths1 = PolyTreeToPathsD(*input1);
		PathsD paths2 = PolyTreeToPathsD(*input2);

		ClipperD cd;
		cd.AddSubject(paths1);
		cd.AddClip(paths2);
		cd.Execute(ClipType::Intersection, FillRule::NonZero, *output);

		return output;
	}


	std::unique_ptr<PolyTreeD> MakeMirrorY(const std::unique_ptr<PolyTreeD>& input)
	{
		PathsD paths = PolyTreeToPathsD(*input);

		for (PathD& path : paths)
			for (PointD& point : path)
				point.y = -point.y;


		std::unique_ptr<PolyTreeD> output = std::make_unique<PolyTreeD>();

		ClipperD cd;
		cd.AddSubject(paths);
		cd.Execute(ClipType::Union, FillRule::NonZero, *output);

		return output;
	}


	void MakeTriangulationPolyTree(const PolyTreeD& tree, Library& output)
	{
		printf("\nTriangulation Clipper2\n");
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		PathsD input = PolyTreeToPathsD(tree);
		PathsD paths_output;

		//Triangulate(input, paths_output, false);
		output = ConvertPathsDToGdsii(paths_output);

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Triangulation faite: " << paths_output.size() << " en: "
			<< std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << std::endl;
	}


	void MakeTriangulationPaths(PathsD& paths, Library& output)
	{
		PathsD final;

		for (PathD& path : paths)
		{
			PathsD path_output;
			PathsD temp;
			temp.push_back(path);

			//Triangulate(temp, path_output, true);
			final.insert(final.end(), path_output.begin(), path_output.end());
		}

		output = ConvertPathsDToGdsii(final);
	}


	void essaiClientComparison();
}


