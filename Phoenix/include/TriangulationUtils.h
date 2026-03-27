#pragma once
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <string>
#include <chrono>

#include <earcut.hpp>
#include <GdstkUtils.h>
#include <vector_functions.h>


typedef unsigned int uint;


namespace TrisUtils
{
    struct Triangulation
    {
        float2* v = nullptr;
        uint3* t = nullptr;
        unsigned char* p = nullptr;
        size_t nb_vertices = 0;
        size_t nb_triangles = 0;
        std::vector<std::pair<int, int>> layers_range;
    };


    // Applique la triangulation earcut à une série de liste de polygones de type gdstk
    std::vector<earcutLayer> EarcutTriangulation(std::vector<Library>& layers);

    // Applique la triangulation earcut à une série de liste de polygones earcut
    earcutLayer earcutTriangulation(earcutPolys& polys);

    // Triangule un .gds avec earcut avec NB_THREADS threads
    earcutLayer earcutTriangulation(const Library& lib, const uint&& NB_THREADS);

    // Conversion polygon gdstk -> polygone earcut
    earcutPoly convertGdstkToEarcutPoly(const gdstk::Polygon* poly);

    // convertit une seule couche de triangles en allocation dans le tas
    std::pair<std::pair<float2*, uint3*>, uint2> convertEarcutLayerToPointer(earcutLayer& triangulation);

    // convertit N couches de triangles en allocation dans le tas en un tableau
    Triangulation convertEarcutLayersToPointer(std::vector<earcutLayer>& triangulation_layers);

    // Normalise la triangulation entre (0, 0) et (scale, scale) sans modifier les échelles
    void ScaleTriangulation(Triangulation& triangulation, float& scale);

    // Sauvegarde des séries de liste de polygones de type earcut en .obj
    void WriteLayersObj(std::vector<earcutLayer>& layers, const char* filename);

    // Sauvegarde des séries de liste de polygones de type gdstk en .obj
    void WriteLibraryToObj(const std::vector<Library>& layers, const char* filename);

    // Enregistre une triangulation de la forme float2*, uint3* en .obj
    void WriteObj(const char* file_name, Triangulation& tris);
}