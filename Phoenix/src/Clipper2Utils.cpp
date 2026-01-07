#include "Clipper2Utils.h"
#include <__msvc_chrono.hpp>
#include <string>
#include "clipper2/clipper.h" // Ajouté pour TriangulatePaths


namespace Clipper2Utils
{
    Paths64 ConvertGdstkPolygonsToPaths64(Library& lib)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        assert(lib.cell_array.count == 1); // pour simplifier le code et la gestion des threads

        Paths64 paths;
        int NB_POLYS = lib.cell_array[0]->polygon_array.count;
        paths.resize(NB_POLYS);

        for (size_t i = 0; i < NB_POLYS; i++)
        {
            Path64 path;
            Polygon* p = lib.cell_array[0]->polygon_array[i];
            path.resize(p->point_array.count);

            for (size_t m = 0; m < p->point_array.count; m++)
                path[m] = std::move(Point64(p->point_array[m].x, p->point_array[m].y));

            paths[i] = std::move(path);
        }

        // free lib, all polygons are now in paths_list
        lib.clear();

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Polygones convertis en polygones clipper2 en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
        return paths;
    }


    Library ConvertPaths64ToGdsii(const Paths64& polys, double epsilon)
    {
        gdstk::Library lib = {};
        lib.init("library", 1e-6, 1e-9);

        gdstk::Cell* cell = new Cell();
        cell->name = copy_string("FIRST", NULL);
        lib.cell_array.append(cell);

        for (const Path64& poly : polys)
        {
            gdstk::Polygon* p_new = (Polygon*)allocate_clear(sizeof(Polygon));
            Path64 path = SimplifyPath(poly, epsilon);

            for (const Point64& point : path)
                p_new->point_array.append( Vec2{ (double)point.x, (double)point.y } );

            cell->polygon_array.append(p_new);
        }

        return lib;
    }


    void GetGdstkPolygonsFromClipper2Tree(PolyTree64& node, int depth, std::vector<gdstk::Polygon*>& polys, double epsilon)
    {
        // parcourir l'arbre
        for (size_t i = 0; i < node.Count(); i++)
            GetGdstkPolygonsFromClipper2Tree(*node.Child(i), depth + 1, polys, epsilon);

        // ajouter le polygone plein et ses enfants comme trou
        if (!node.IsHole() && node.Polygon().size() > 0)
        {
            Polygon* poly = (Polygon*)allocate_clear(sizeof(Polygon));
            Path64 simplified_poly = SimplifyPath(node.Polygon(), epsilon);

            // ajouter le polygone plein
            for (const Point64& point : simplified_poly)
                poly->point_array.append(Vec2{ (double)point.x, (double)point.y });

            // push le premier point pour fermer la boucle
            Vec2 first{ simplified_poly.front().x, simplified_poly.front().y };
            poly->point_array.append(first);

            // ajouter les polygones enfants au polygone gdstk
            for (size_t k = 0; k < node.Count(); k++)
            {
                Path64 simplified_poly = SimplifyPath(node.Child(k)->Polygon(), epsilon);

                for (const Point64& point : simplified_poly)
                    poly->point_array.append(Vec2{ (double)point.x, (double)point.y });

                Vec2 child_first{ simplified_poly.front().x, simplified_poly.front().y };
                poly->point_array.append(child_first);
                poly->point_array.append(first);
            }

            polys.push_back(poly);
        }
    }


    void ConvertPolyTree64ToGdsiiPath(PolyTree64& tree, Library& output, double epsilon)
    {
        output.init("library", 1e-6, 1e-9);

        gdstk::Cell* cell = new Cell();
        cell->name = copy_string("FIRST", NULL);
        output.cell_array.append(cell);

        std::vector<gdstk::Polygon*> polys;
        GetGdstkPolygonsFromClipper2Tree(tree, 1, polys, epsilon);

        Array<gdstk::Polygon*> gdstk_polys = {};

        for (Polygon* p : polys)
            gdstk_polys.append(p);

        cell->polygon_array = gdstk_polys;
    }


    void GetTreeLayers(PolyTree64& node, int depth, std::vector<Paths64>& layers)
    {
        // parcourir les enfants
        for (size_t i = 0; i < node.Count(); i++)
            GetTreeLayers(*node.Child(i), depth + 1, layers);

        // enregistrer le polygone dans la bonne couche
        Path64 p = node.Polygon();

        if (layers.size() < depth)
            layers.resize(depth);

        layers[depth - 1].push_back(p);
    }


    std::vector<Library> ConvertPolyTree64ToGdsiiLayers(PolyTree64& polys)
    {
        std::vector<Paths64> paths64_layers;
        GetTreeLayers(polys, 1, paths64_layers);

        std::vector<Library> layers;

        for (size_t i = 0; i < paths64_layers.size(); i++)
            layers.push_back(ConvertPaths64ToGdsii(paths64_layers[i], 0));

        return layers;
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
        std::cout << "Degraissement fait en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
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
        std::cout << "Difference faite en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
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
        std::cout << "Inverse fait en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
    }


    void MakeTriangulationPolyTree(const PolyTree64& tree, Library& output)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        Paths64 input = PolyTreeToPaths64(tree);
        Paths64 paths_output;

        Triangulate(input, paths_output, false);
        output = ConvertPaths64ToGdsii(paths_output, 0);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Triangulation faite: " << paths_output.size() << " en: " 
            << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() <<std::endl;
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

        output = ConvertPaths64ToGdsii(final, 0);
    }
}


