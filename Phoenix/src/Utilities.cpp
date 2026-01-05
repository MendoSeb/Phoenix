#pragma once

namespace Utils
{
    std::vector<earcutLayer> Triangulation(std::vector<Library>& layers)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::vector<earcutLayer> result;

        // create structure for triangulation
        for (Library& poly_layer : layers)
        {
            earcutLayer layer_result;
            earcutPolys p;

            // conversion en polygones earcut pour triangulation
            for (size_t i = 0; i < poly_layer.cell_array[0]->polygon_array.count; i++)
            {
                gdstk::Polygon* poly = poly_layer.cell_array[0]->polygon_array[i];
                std::vector<std::vector<earcutPoint>> gdstk_poly;
                std::vector<earcutPoint> main_edge;

                for (size_t k = 0; k < poly->point_array.count; k++)
                {
                    earcutPoint new_point{ (double)poly->point_array[k].x, (double)poly->point_array[k].y };
                    main_edge.push_back(new_point);
                }

                gdstk_poly.push_back(main_edge);
                p.push_back(gdstk_poly);
            }

            // triangulation des polygones earcut
            layer_result.first = p;
            size_t vertex_cumul = 0;

            for (const auto& poly : p)
            {
                std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(poly);

                // incrémenter la valeur des indices pour les transformer en indice global pour les vertex
                for (uint32_t& i : indices)
                    i += vertex_cumul;

                layer_result.second.push_back(indices);
                vertex_cumul += poly[0].size();
            }
            result.push_back(layer_result);
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Triangulation faite en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << std::endl;
        return result;
    }


    void WriteLayersObj(std::vector<earcutLayer>& layers, const char* filename)
    {
        std::ofstream file(filename);
        double min = INT_MAX;
        double max = INT_MIN;

        // trouver le min et max
        for (auto& layer : layers)
            for (const std::vector<std::vector<earcutPoint>>& poly : layer.first)
                for (const std::vector<earcutPoint>& poly_sub : poly)
                    for (const earcutPoint& point : poly_sub)
                    {
                        double temp_min = std::min(point[0], point[1]);
                        double temp_max = std::max(point[0], point[1]);

                        if (temp_min < min) min = temp_min;
                        if (temp_max > max) max = temp_max;
                    }


        // normaliser les sommet
        for (int i = 0; i < layers.size(); i++)
            for (std::vector<std::vector<earcutPoint>>& poly : layers[i].first)
                for (std::vector<earcutPoint>& poly_sub : poly)
                    for (earcutPoint& point : poly_sub)
                    {
                        point[0] = ((point[0] + std::abs(min)) / (std::abs(min) + max)) * 2.0 - 1.0;
                        point[1] = ((point[1] + std::abs(min)) / (std::abs(min) + max)) * 2.0 - 1.0;
                        file << "v " << std::to_string(point[0]) << " " << std::to_string(point[1]) << " " << std::to_string(-i / 10.0) << "\n";
                    }

        // enregistrer les indices dans l'obj
        size_t nb_vertex = 0;
        size_t nb_color = 0;
        size_t nb_tris = 0;

        for (auto& layer : layers)
        {
            file << "c " << std::to_string(nb_color) << '\n';
            nb_color++;

            for (const std::vector<uint32_t>& indices : layer.second)
            {
                for (int i = 0; i < indices.size(); i += 3)
                {
                    std::string i1 = std::to_string(nb_vertex + indices[i] + 1);
                    std::string i2 = std::to_string(nb_vertex + indices[i + 1] + 1);
                    std::string i3 = std::to_string(nb_vertex + indices[i + 2] + 1);
                    file << "f " << i1 << " " << i2 << " " << i3 << "\n";
                    nb_tris++;
                }
            }

            // incrémenter le nombre de vertex pour offset les indices des layers
            for (auto& poly : layer.first)
                for (auto& p0 : poly)
                    for (auto& p1 : p0)
                        nb_vertex++;
        }

        std::cout << "Nombre de triangles: " << nb_tris << std::endl;
        file.close();
    }


    void WriteLibraryToObj(const std::vector<Library>& layers, const char* filename)
    {
        std::ofstream file(filename);
        double min = INT_MAX;
        double max = INT_MIN;

        // trouver le min et max
        for (const Library& layer : layers)
            for (size_t i = 0; i < layer.cell_array[0]->polygon_array.count; i++)
            {
                gdstk::Polygon* poly = layer.cell_array[0]->polygon_array[i];

                for (size_t k = 0; k < poly->point_array.count; k++)
                {
                    double temp_min = std::min(poly->point_array[k].x, poly->point_array[k].y);
                    double temp_max = std::max(poly->point_array[k].x, poly->point_array[k].y);

                    if (temp_min < min) min = temp_min;
                    if (temp_max > max) max = temp_max;
                }
            }

        // normaliser les sommet
        for (int m = 0; m < layers.size(); m++)
            for (size_t i = 0; i < layers[m].cell_array[0]->polygon_array.count; i++)
            {
                gdstk::Polygon* poly = layers[m].cell_array[0]->polygon_array[i];

                for (size_t k = 0; k < poly->point_array.count; k++)
                {
                    Vec2& point = poly->point_array[k];

                    point.x = ((point.x + std::abs(min)) / (std::abs(min) + max)) * 2.0 - 1.0;
                    point.y = ((point.y + std::abs(min)) / (std::abs(min) + max)) * 2.0 - 1.0;
                    file << "v " << std::to_string(point.x) << " " << std::to_string(point.y) << " " << std::to_string(-m / 10.0) << "\n";
                }
            }

        // enregistrer les indices dans l'obj
        size_t layer_nb_vertex = 0;
        size_t nb_color = 0;

        for (const Library& layer : layers)
        {
            size_t poly_nb_vertex = 0;

            for (size_t i = 0; i < layer.cell_array[0]->polygon_array.count; i++)
            {
                gdstk::Polygon* poly = layer.cell_array[0]->polygon_array[i];

                for (size_t k = 0; k < poly->point_array.count; k += 3)
                {
                    std::string i1 = std::to_string(layer_nb_vertex + poly_nb_vertex + +k + 1);
                    std::string i2 = std::to_string(layer_nb_vertex + poly_nb_vertex + k + 1 + 1);
                    std::string i3 = std::to_string(layer_nb_vertex + poly_nb_vertex + k + 2 + 1);
                    file << "f " << i1 << " " << i2 << " " << i3 << "\n";
                }

                poly_nb_vertex += poly->point_array.count;
            }
            layer_nb_vertex += poly_nb_vertex;
        }

        std::cout << "Nombre de triangles: " << std::round(layer_nb_vertex / 3.0) << std::endl;
        file.close();
    }


    void repeatAndTranslate(Library& lib, uint64_t rep_x, uint64_t rep_y, double width, double height)
    {
        for (size_t i = 0; i < lib.cell_array.count; i++)
            for (size_t k = 0; k < lib.cell_array[i]->polygon_array.count; k++)
            {
                lib.cell_array[i]->polygon_array[k]->repetition = Repetition{
                    RepetitionType::Rectangular,
                    rep_x,
                    rep_y,
                    Vec2{width, height},
                };
            }
    }


    Paths64 RepeatAndTranslateClipper2(const Library& lib, uint64_t rep_x, uint64_t rep_y, double width, double height, double factor)
    {
        Paths64 final_path;
        width *= factor; // pour déplacer les éléments selon leur scale
        height *= factor;

        for (size_t i = 0; i < lib.cell_array.count; i++)
        {
            for (size_t k = 0; k < lib.cell_array[i]->polygon_array.count; k++)
            {
                Polygon* p = lib.cell_array[i]->polygon_array[k];
                Path64 poly;
                poly.resize(p->point_array.count);

                // convert gdsii to paths64
                for (size_t m = 0; m < p->point_array.count; m++)
                    poly[m] = Point64(p->point_array[m].x * factor, p->point_array[m].y * factor);

                // duplicate
                for (size_t x = 0; x < rep_x; x++)
                    for (size_t y = 0; y < rep_y; y++)
                    {
                        Path64 new_path = poly;

                        for (Point64& point : new_path)
                            point = point + Point64(x * width, y * height);

                        final_path.push_back(new_path);
                    }
            }
        }

        std::cout << "Polygones dupliques en paths64" << std::endl;;
        return final_path;
    }

    void ConvertToPolygon(Library& lib)
    {
        for (size_t i = 0; i < lib.cell_array.count; i++)
        {
            Cell* c = lib.cell_array[i];

            /// flexpath
            for (size_t k = 0; k < c->flexpath_array.count; k++)
            {
                FlexPath* fp = c->flexpath_array[k];

                /// pour les cercles
                if (fp->spine.point_array.count == 2
                    && (fp->spine.point_array[0] == fp->spine.point_array[1]))
                {
                    double max_radius = std::max(fp->elements->half_width_and_offset[0].x, fp->elements->half_width_and_offset[0].y);

                    gdstk::Polygon* circle = new gdstk::Polygon(gdstk::ellipse(
                        fp->spine.point_array[0],
                        max_radius,
                        max_radius,
                        0.0, 0.0,
                        0.0, 0.0,
                        1e-4, // précision de la discrétisation du cercle
                        0
                    ));

                    c->polygon_array.append(circle);
                    continue;
                }
            }

            // transformer les flexpath en polygones
            Array<FlexPath*> flexpath_array = lib.cell_array[i]->flexpath_array;

            for (size_t m = 0; m < flexpath_array.count; m++)
                flexpath_array[m]->to_polygons(false, 0, lib.cell_array[i]->polygon_array);

            // destroy flexpath, they are now polygons
            lib.cell_array[i]->flexpath_array.clear();
        }
    }


    // Découpe les polygones ayant plus de 8190 sommets en plusieurs polygones plus petits
    void MakeFracture(Library& lib)
    {
        for (size_t i = 0; i < lib.cell_array.count; i++)
        {
            Array<gdstk::Polygon*> new_polys = {};

            for (size_t k = 0; k < lib.cell_array[i]->polygon_array.count; k++)
            {
                gdstk::Polygon* poly = lib.cell_array[i]->polygon_array[k];
                poly->fracture(8190, 1e-3, new_polys);

                if (poly->point_array.count > 8190)
                    std::cout << i << ", " << k << ": " << new_polys.count << std::endl;
            }
        }
    }



    Library LoadGDS(const char* fileName)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        Library lib = read_gds(fileName, 1e-6, 1e-9, nullptr, nullptr);
        std::cout << "Unit: " << lib.unit << std::endl;
        std::cout << "Precision: " << lib.precision << std::endl;
        std::cout << "Nb cell: " << lib.cell_array.count << "\n\n";

        for (size_t i = 0; i < lib.cell_array.count; i++)
        {
            std::cout << "cell: " << i << std::endl;
            std::cout << "Polygones: " << lib.cell_array[i]->polygon_array.count << std::endl;
            std::cout << "Reference: " << lib.cell_array[i]->reference_array.count << std::endl;
            std::cout << "FlexPath: " << lib.cell_array[i]->flexpath_array.count << std::endl;
            std::cout << "Robustpath: " << lib.cell_array[i]->robustpath_array.count << std::endl;
            std::cout << std::endl;
        }

        ConvertToPolygon(lib);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Fichier charge en polygones gdstk en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
        return lib;
    }


    void SaveToGdsii(Library& lib, const char* fileName)
    {
        //MakeFracture(lib);
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        lib.write_gds(fileName, INT32_MAX, NULL);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Sauvegarde vers " << fileName << " faite en: " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
    }
}