#pragma once
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <string>
#include <chrono>

#include <earcut.hpp>
#include <GdstkUtils.h>
#include <Clipper2Utils.h>
#include <BoostUtils.h>
#include <vector_functions.h>
#include "types.h"
#include "tinyxml.h"


namespace Utils
{
    /* Applique la triangulation earcut à une série de liste de polygones de type gdstk */
    std::vector<earcutLayer> EarcutTriangulation(std::vector<Library>& layers);

    /* Applique la triangulation earcut à une série de liste de polygones earcut */
    earcutLayer earcutTriangulation(earcutPolys& polys);

    earcutLayer earcutTriangulation(const Library& lib, const uint&& NB_THREADS);

    earcutPoly convertGdstkToEarcutPoly(const gdstk::Polygon* poly);

    std::pair<std::pair<float2*, uint3*>, uint2> convertEarcutLayerToPointer(earcutLayer& triangulation);

    void ScaleTriangulation(std::vector<std::pair<std::pair<float2*, uint3*>, uint2>>& triangulation, float& scale);

    /* Sauvegarde des séries de liste de polygones de type earcut en .obj */
    void WriteLayersObj(std::vector<earcutLayer>& layers, const char* filename);

    /* Sauvegarde des séries de liste de polygones de type gdstk en .obj */
    void WriteLibraryToObj(const std::vector<Library>& layers, const char* filename);

    void writeObj(const char* file_name, float2* vertices, uint3* triangles, 
        size_t nb_v, size_t nb_tris);

    std::vector<earcutPolys> ConvertSVGToEarcutLayers(const char* svg_filepath);
}