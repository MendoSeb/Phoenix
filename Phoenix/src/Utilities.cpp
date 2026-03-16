#pragma once
#include "Utilities.h"
#include <GdstkUtils.h>
#include <cudaCall.h>


namespace Utils
{
    std::vector<earcutLayer> EarcutTriangulation(std::vector<Library>& layers)
    {
        printf("\nTriangulation earcut\n");
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
        std::cout << "Triangulation faite en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
        return result;
    }

   earcutLayer earcutTriangulation(earcutPolys& polys)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        earcutLayer layer;
        int vertex_sum = 0;

        for (earcutPoly& poly : polys)
        {
            std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(poly);

            // pour indexer correctement les sommets avec les triangles
            for (uint32_t& index : indices)
                index += vertex_sum;

            // incrémente le nombre de sommets
            for (std::vector<earcutPoint>& contour : poly)
                vertex_sum += contour.size();

            layer.first.push_back(poly);
            layer.second.push_back(indices);
        }        

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Triangulation earcut faite en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
        return layer;
    }

    earcutLayer earcutTriangulation(const Library& lib, const uint&& NB_THREADS)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        std::thread* threads = new std::thread[NB_THREADS];

        size_t nb_polys = lib.cell_array[0]->polygon_array.count;
        size_t nb_polys_per_thread = nb_polys / NB_THREADS;

        earcutLayer triangulation;
        triangulation.first.resize(nb_polys);
        triangulation.second.resize(nb_polys);

        /// fonction pour chaque thread
        auto lambda = [&](size_t min_index, size_t max_index)
        {
            // count vertex_number until the min_index polygon
            size_t indices_count = 0;

            for (size_t i = 0; i < min_index; i++)
                indices_count += lib.cell_array[0]->polygon_array[i]->point_array.count;

            for (size_t i = min_index; i < max_index; i++)
            {
                earcutPoly poly = convertGdstkToEarcutPoly(lib.cell_array[0]->polygon_array[i]);
                std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(poly);

                for (uint32_t& index : indices)
                    index += indices_count;

                indices_count += poly[0].size();
                triangulation.first[i] = std::move(poly);
                triangulation.second[i] = std::move(indices);
            }
        };

        /// lancement des threads
        for (size_t i = 0; i < NB_THREADS; i++)
        {
            size_t min_index = i * nb_polys_per_thread;
            size_t max_index = (i + 1) * nb_polys_per_thread;

            if (i == NB_THREADS - 1)
                max_index = nb_polys;

            threads[i] = std::thread(lambda, min_index, max_index);
        }

        /// attendre que tous les threas aient finis
        for (size_t i = 0; i < NB_THREADS; i++)
            threads[i].join();

        delete[] threads;

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "! triangulation earcut en : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

        return triangulation;
    }

    earcutPoly convertGdstkToEarcutPoly(const Polygon* gdstk_poly)
    {
        earcutPoly poly = { {} };

        for (size_t k = 0; k < gdstk_poly->point_array.count; k++)
        {
            earcutPoint point{
               gdstk_poly->point_array[k].x,
               gdstk_poly->point_array[k].y
            };

            poly[0].push_back(point);
        }

        return poly;
    }

    std::pair<std::pair<float2*, uint3*>, uint2> convertEarcutLayerToPointer(earcutLayer& triangulation)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // count vertices and triangles
        size_t nb_vertices = 0;
        size_t nb_triangles = 0;

        for (earcutPoly& poly : triangulation.first)
            nb_vertices += poly[0].size();

        for (std::vector<uint32_t>& indices : triangulation.second)
            nb_triangles += indices.size();

        // convert to float2* and uint3*
        nb_triangles /= 3;
        std::chrono::steady_clock::time_point s1 = std::chrono::steady_clock::now();

        float2* hv = nullptr;
        uint3* ht = nullptr;
        cudaMallocHost((void**)&hv, nb_vertices * sizeof(float2));
        std::chrono::steady_clock::time_point e2 = std::chrono::steady_clock::now();

        cudaMallocHost((void**)&ht, nb_triangles * sizeof(uint3));

        std::chrono::steady_clock::time_point e1 = std::chrono::steady_clock::now();
        std::cout << "! allocation memoire sommets : " << std::chrono::duration_cast<std::chrono::milliseconds>(e2 - s1).count() << " ms" << std::endl;
        std::cout << "! allocation memoire triangles : " << std::chrono::duration_cast<std::chrono::milliseconds>(e1 - e2).count() << " ms" << std::endl;

        size_t vertex_index = 0;
        size_t triangle_index = 0;

        for (earcutPoly& poly : triangulation.first)
            for (std::vector<earcutPoint>& poly2 : poly)
                for (earcutPoint& point : poly2)
                {
                    hv[vertex_index].x = point.at(0);
                    hv[vertex_index].y = point.at(1);
                    vertex_index++;
                }

        for (std::vector<uint32_t>& indices : triangulation.second)
        {
            for (size_t i = 0; i < indices.size(); i += 3)
            {
                ht[triangle_index].x = indices[i];
                ht[triangle_index].y = indices[i+1];
                ht[triangle_index].z = indices[i+2];

                triangle_index++;
            }
        }

        uint2 count{ nb_vertices, nb_triangles };

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
       // std::cout << "! conversion triangulation en pointeur : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

        return { {hv, ht}, count };
    }

    Triangulation convertEarcutLayersToPointer(std::vector<earcutLayer>& triangulation_layers)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        /// count vertices and triangles
        Triangulation t;

        for (earcutLayer& layer : triangulation_layers)
        {
            size_t previous_nb_triangles = t.nb_triangles;

            for (earcutPoly& poly : layer.first)
                t.nb_vertices += poly[0].size();

            for (std::vector<uint32_t>& indices : layer.second)
                t.nb_triangles += indices.size() / 3;

            t.layers_range.push_back( {previous_nb_triangles, t.nb_triangles} );
        }

        /// allocation in RAM
        auto start = std::chrono::steady_clock::now();

        t.v = new float2[t.nb_vertices * sizeof(float2)];
        t.t = new uint3[t.nb_triangles * sizeof(uint3)];
        t.p = new unsigned char[t.nb_triangles * sizeof(unsigned char)];

        auto end2 = std::chrono::steady_clock::now();
        std::cout << "! allocation ram en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start).count() << " ms" << std::endl;

        /// assign values to vertices and triangles and polarity array
        size_t vertex_index = 0;
        size_t triangle_index = 0;
        size_t indices_count = 0;

        for (earcutLayer& layer : triangulation_layers)
            for (earcutPoly& poly : layer.first)
                for (std::vector<earcutPoint>& poly2 : poly)
                    for (earcutPoint& point : poly2)
                    {
                        t.v[vertex_index].x = point.at(0);
                        t.v[vertex_index].y = point.at(1);
                        vertex_index++;
                    }

        for (int i = 0; i < triangulation_layers.size(); i++)
        {
            for (std::vector<uint32_t>& indices : triangulation_layers[i].second)
            {
                for (size_t k = 0; k < indices.size(); k += 3)
                {
                    t.t[triangle_index].x = indices_count + indices[k];
                    t.t[triangle_index].y = indices_count + indices[k + 1];
                    t.t[triangle_index].z = indices_count + indices[k + 2];

                    t.p[triangle_index] = (unsigned char)((i % 2) == 0) * 255;
                    triangle_index++;
                }
            }

            // pour l'offset des indices des sommets des triangles
            for (earcutPoly& poly : triangulation_layers[i].first)
                indices_count += poly[0].size();
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "! Conversion triangulation en pointeur : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

        return t;
    }

    void ScaleTriangulation(
        Utils::Triangulation& triangulation, 
        float& scale
    )
    {
        auto start = std::chrono::steady_clock::now();

        float min_x = FLT_MAX;
        float min_y = FLT_MAX;
        float max_x = -FLT_MAX;
        float max_y = -FLT_MAX;

        // find min and max coordinates
        for (int vertice_i = 0; vertice_i < triangulation.nb_vertices; vertice_i++) 
        {
            min_x = std::min(min_x, triangulation.v[vertice_i].x);
            min_y = std::min(min_y, triangulation.v[vertice_i].y);

            max_x = std::max(max_x, triangulation.v[vertice_i].x);
            max_y = std::max(max_y, triangulation.v[vertice_i].y);
        }

        double max_side_size = std::max(max_x - min_x, max_y - min_y);

        // normalize between (0, 0) and (scale, scale) without modifying scale ratio
        for (int vertice_i = 0; vertice_i < triangulation.nb_vertices; vertice_i++)
        {
            triangulation.v[vertice_i].x =
                ((triangulation.v[vertice_i].x - min_x) / max_side_size) * scale;

            triangulation.v[vertice_i].y =
                ((triangulation.v[vertice_i].y - min_x) / max_side_size) * scale;
        }

        auto end = std::chrono::steady_clock::now();
        std::cout << "! Scaling en : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
    }

    void WriteLayersObj(std::vector<earcutLayer>& layers, const char* filename)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::ofstream file(filename);

        // écrire les sommets comme entiers, les doubles mettent du temps à être écrits (environ x2)
        for (char i = 0; i < layers.size(); i++)
            for (std::vector<std::vector<earcutPoint>>& poly : layers[i].first)
                for (std::vector<earcutPoint>& poly_sub : poly)
                    for (earcutPoint& point : poly_sub)
                    {
                        file << "v "<< std::to_string(point[0] / 5.4)
                             << " " << std::to_string(point[1] / 5.4)
                             << " " << std::to_string(i) << "\n";
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
                for (int i = 0; i < indices.size(); i += 3)
                {
                    std::string i1 = std::to_string(nb_vertex + indices[i] + 1);
                    std::string i2 = std::to_string(nb_vertex + indices[i + 1] + 1);
                    std::string i3 = std::to_string(nb_vertex + indices[i + 2] + 1);
                    file << "f " << i1 << " " << i2 << " " << i3 << "\n";
                    nb_tris++;
                }

            // incrémenter le nombre de vertex pour offset les indices des layers
            for (auto& poly : layer.first)
                for (auto& p0 : poly)
                    nb_vertex += p0.size();
        }

        std::cout << "Nombre de triangles: " << nb_tris << std::endl;
        std::cout << "Nombre de sommets: " << nb_vertex << std::endl;
        file.close();

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "WriteLayersObj fait en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    }

    void WriteLibraryToObj(const std::vector<Library>& layers, const char* filename)
    {
        std::ofstream file(filename);

        // normaliser les sommets entre -1 et 1
        for (int m = 0; m < layers.size(); m++)
        {
            Cell* cell = layers[m].cell_array[0];

            for (size_t i = 0; i < cell->polygon_array.count; i++)
            {
                gdstk::Polygon* poly = cell->polygon_array[i];

                for (size_t k = 0; k < poly->point_array.count; k++)
                {
                    Vec2& point = poly->point_array[k];

                    point.x /= 100000;
                    point.y /= 100000;
                    file << "v " << std::to_string(point.x) << " " << std::to_string(point.y) << " " << std::to_string(-m / 10.0) << "\n";
                }
            }
        }

        // enregistrer les indices dans l'obj
        size_t layer_nb_vertex = 0;
        size_t nb_color = 0;

        for (const Library& layer : layers)
        {
            size_t poly_nb_vertex = 0;
            Cell* cell = layer.cell_array[0];

            for (size_t i = 0; i < cell->polygon_array.count; i++)
            {
                gdstk::Polygon* poly = cell->polygon_array[i];

                for (size_t k = 0; k < poly->point_array.count; k += 3)
                {
                    std::string i1 = std::to_string(layer_nb_vertex + poly_nb_vertex + k + 1);
                    std::string i2 = std::to_string(layer_nb_vertex + poly_nb_vertex + k + 2);
                    std::string i3 = std::to_string(layer_nb_vertex + poly_nb_vertex + k + 3);
                    file << "f " << i1 << " " << i2 << " " << i3 << "\n";
                }

                poly_nb_vertex += poly->point_array.count;
            }
            layer_nb_vertex += poly_nb_vertex;
        }

        std::cout << "Nombre de triangles: " << std::round(layer_nb_vertex / 3.0) << std::endl;
        file.close();
    }

    void writeObj(const char* file_name, float2* vertices, uint3* triangles, 
        size_t nb_v, size_t nb_tris)
    {
        std::ofstream file(file_name);

        for (size_t i = 0; i < nb_v; i++)
            file << "v " 
            << std::to_string(vertices[i].x) << " " 
            << std::to_string(vertices[i].y) << " 0.0\n";

        for (size_t i = 0; i < nb_tris; i++)
            file << "f " 
            << std::to_string(triangles[i].x + 1) << " "
            << std::to_string(triangles[i].y + 1) << " "
            << std::to_string(triangles[i].z + 1) << "\n";

        file.close();

        printf("sauvegarde en .obj faite\n");
    }

    std::vector<earcutPolys> ConvertSVGToEarcutLayers(const char* svg_filepath)
    {
        auto start = std::chrono::steady_clock::now();

        TiXmlDocument doc;
        bool file_loaded = doc.LoadFile(svg_filepath);
        assert(file_loaded);

        TiXmlElement* svg = doc.FirstChildElement("svg");
        TiXmlElement* g = svg->FirstChildElement("g");
        TiXmlElement* current_path = g->FirstChildElement("path");
        std::vector<earcutPolys> polys_layers;
        std::string last_fill_color = "";

        while (current_path != nullptr)
        {
            const char* fill = current_path->Attribute("fill");
            const char* d = current_path->Attribute("d");

            // adding a new layer if the fill color is different
            if (fill != last_fill_color)
            {
                polys_layers.push_back({});
                last_fill_color = fill;
            }

            // load all points from <path> element
            std::stringstream full_string(d);
            std::string x_coordinate, y_coordinate;
            char delimiter = ' ';

            earcutPoly poly;
            poly.push_back({});

            while (getline(full_string, x_coordinate, delimiter)
                && getline(full_string, y_coordinate, delimiter))
            {
                // remove 'M' or 'L' character from x coordinate
                x_coordinate = x_coordinate.substr(1, x_coordinate.size() - 1);

                earcutPoint point;
                point.at(0) = std::stof(x_coordinate);
                point.at(1) = std::stof(y_coordinate);

                poly[0].push_back(point);
            }

            poly[0].pop_back();
            polys_layers.back().push_back(poly);
            current_path = current_path->NextSiblingElement("path");
        }


        auto end = std::chrono::steady_clock::now();
        std::cout << "! Conversion SVG -> Polygones earcut: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

        return polys_layers;
    }
};