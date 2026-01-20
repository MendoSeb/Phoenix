#pragma once
#include "Utilities.h"
#include <GdstkUtils.h>


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


    std::vector<earcutLayer> EarcutTriangulation(earcutPolys& polys)
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
        return { layer };
    }


    void SplitEarcutPolygons(earcutPolys& polys)
    {
        for (earcutPoly& poly : polys)
        {
            Polygon* gdstk_poly = (Polygon*)allocate_clear(sizeof(Polygon));


        }
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

    std::pair<std::vector<cv::Point2f>, std::vector<uint3>>  LoadObjVerticesTriangles(const char* filename)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        
        std::vector<cv::Point2f> vertices;
        std::vector<uint3> triangles;
        
        std::ifstream file(filename);
        std::string line;
        float temp = 0.0f;

        // récupérer les données du fichier .obj
        while (std::getline(file, line)) 
        {
            if (line[0] == 'v')
            {
                cv::Point2f vertex;
                int res = sscanf(line.c_str(), "v %f %f %f", &vertex.x, &vertex.y, &temp);
                vertices.push_back(vertex);
            }
            else if (line[0] == 'f')
            {
                uint3 tri = { 0.0f, 0.0f, 0.0f };
                int res = sscanf(line.c_str(), "f %d/%d/%d", &tri.x, &tri.y, &tri.z);
                triangles.push_back(tri);
            }
        }

        file.close();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Chargement de l'obj en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
        return { vertices, triangles };
    }


    std::pair<float3*, uint3*> GetVertexAndTriangles(std::vector<std::vector<cv::Point2f>>& polys)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        std::pair<float3*, uint3*> obj;
        obj.first = (float3*)malloc(polys.size() * 3 * sizeof(float3));
        obj.second = (uint3*)malloc(polys.size() * sizeof(uint3));

        for (size_t i = 0; i < polys.size(); i++)
        {
                obj.first[i*3] = make_float3(polys[i][0].x, polys[i][0].y, 0.0f);
                obj.first[i*3 + 1] = make_float3(polys[i][1].x, polys[i][1].y, 0.0f);
                obj.first[i*3 + 2] = make_float3(polys[i][2].x, polys[i][2].y, 0.0f);

                obj.second[i] = make_uint3(i * 3, i * 3 + 1, i * 3 + 2);
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Extract vertices and triangles: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms\n" << std::endl;
        return obj;
    }
}