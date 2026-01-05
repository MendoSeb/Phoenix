#include "../include/BoostOperators.h"

BoostOperators::BoostOperators()
{
}


multi_polygon_t BoostOperators::ConvertGdstkToBoostPolygon(const Library& lib)
{
    multi_polygon_t polygons;

    for (size_t i = 0; i < lib.cell_array.count; i++)
    {
        Cell* c = lib.cell_array[i];

        for (int k = 0; k < c->polygon_array.count; k++)
        {
            Polygon* p = c->polygon_array[k];
            polygon_t poly;

            for (int m = 0; m < p->point_array.count; m++)
            {
                double x = p->point_array[m].x;
                double y = p->point_array[m].y;
                point_t point(x, y);

                bg::append(poly.outer(), point);
            }

            bg::correct(poly);
            polygons.push_back(poly);
        }
    }

    printf("Polygones convertis en polygones boost\n");
    return polygons;
}


void BoostOperators::ConvertBoostPolygonToGdstk(multi_polygon_t& polys, const char* fileName)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    gdstk::Library lib = {};
    lib.init("library", 1e-6, 1e-9);

    gdstk::Cell cell = {};
    cell.name = copy_string("FIRST", NULL);
    lib.cell_array.append(&cell);
    std::cout << "sauvegarde\n";

    for (polygon_t& p : polys)
    {
        bg::correct(p);

        auto exterior = bg::exterior_ring(p);
        auto interior = bg::interior_rings(p);
        gdstk::Polygon* p_new = (Polygon*)allocate_clear(sizeof(Polygon));

        // point extérieurs
        for (point_t point : exterior)
        {
            gdstk::Vec2 v;
            v.x = point.get<0>();
            v.y = point.get<1>();
            p_new->point_array.append(v);
        }

        // ajouter le premier point pour fermer le contour extérieur
        p_new->point_array.append(Vec2{ exterior[0].get<0>(), exterior[0].get<1>() });

        // point intérieurs
        for (auto int_poly : interior)
        {
            for (point_t point : int_poly)
            {
                gdstk::Vec2 v;
                v.x = point.get<0>();
                v.y = point.get<1>();
                p_new->point_array.append(v);
            }

            // ajouter le premier point pour fermer le contour intérieur
            p_new->point_array.append(Vec2{ interior[0].front().get<0>(), interior[0].front().get<1>() });
        }

        cell.polygon_array.append(p_new);
    }

    // sauvegarde des polygones finaux gdstk dans un gdsii
    lib.write_gds(fileName, INT32_MAX, NULL);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "sauvegarde faite en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
}


multi_polygon_t BoostOperators::MakeUnion(const multi_polygon_t& polys)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    multi_polygon_t result;

    /*for (const polygon_t& p : polys)
    {
        multi_polygon_t temp;
        bg::union_(p, result, temp);
        result = temp;
    }*/

    multi_polygon_t temp;
    bg::union_(polys, polys, result);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Union faite en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
    return result;
}


void BoostOperators::MakeDegraissement(int buffer_dist, const char* savePath)
{
    /*// chargement des polygones boostGeometry
    const char* fileName = "C:/Users/PC/Desktop/poc/fichiers_gdsii/result_union_boost.gds";
    multi_polygon_t polys = convertGdstkToBoostPolygon(fileName);
    std::cout << "nb polys: " << polys.size() << std::endl;

    // Declare strategies
    const double buffer_distance = buffer_dist; // distance ajoutée/enlevée pour l'engraissement/dégraissement
    const int points_per_circle = 36;
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_distance);
    boost::geometry::strategy::buffer::join_miter                 join_strategy(points_per_circle);
    boost::geometry::strategy::buffer::end_flat                   end_strategy;
    boost::geometry::strategy::buffer::point_circle               circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight              side_strategy;

    //multi_polygon_t result;

    bg::buffer(
        polys, 
        result,
        distance_strategy, side_strategy,
        join_strategy, end_strategy, circle_strategy
    );

    convertBoostPolygonToGdstk(result, savePath);*/
}


void BoostOperators::MakeDifference()
{
    /*const char* fileName1 = "C:/Users/PC/Desktop/poc/fichiers_gdsii/result_union_boost.gds";
    const char* fileName2 = "C:/Users/PC/Desktop/poc/fichiers_gdsii/result_degraissement_boost.gds";
    multi_polygon_t polygonsFull = convertGdstkToBoostPolygon(fileName1);
    multi_polygon_t polygonsEmpty = convertGdstkToBoostPolygon(fileName2);

    // tableau qui stockera tous les résultats des différences
    std::vector<polygon_t> result_polys;
    std::cout << polygonsFull.size() << ", " << polygonsEmpty.size() << std::endl;

    // difference v2
    bg::difference(polygonsFull, polygonsEmpty, result_polys);
    std::cout << result_polys.size() << std::endl;

    // conversion des polygones boost en polygones gdstk
    convertBoostPolygonToGdstk(result_polys, "C:/Users/PC/Desktop/poc/fichiers_gdsii/result_difference_boost.gds");*/
}
