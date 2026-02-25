#pragma once
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <string>
#include <chrono>
#include <opencv2/core/types.hpp>

#include <earcut.hpp>
#include <GdstkUtils.h>
#include <Clipper2Utils.h>
#include <BoostUtils.h>
#include <vector_functions.h>


namespace Utils
{
    /* Applique la triangulation earcut à une série de liste de polygones de type gdstk */
    std::vector<earcutLayer> EarcutTriangulation(std::vector<Library>& layers);

    /* Applique la triangulation earcut à une série de liste de polygones earcut */
    std::vector<earcutLayer> EarcutTriangulation(earcutPolys& polys);

    earcutLayer earcutTriangulation(const Library& lib);

    bool isTriangleClockwise(int2 tri[3]);
    
    void correctTriangulation(std::pair<std::pair<float2*, uint3*>, uint2>& tris);

    earcutPoly convertGdstkToEarcutPoly(const Polygon* poly);

    std::pair<std::pair<float2*, uint3*>, uint2> convertEarcutLayerToPointer(earcutLayer& triangulation);

    /* Sauvegarde des séries de liste de polygones de type earcut en .obj */
    void WriteLayersObj(std::vector<earcutLayer>& layers, const char* filename);

    /* Sauvegarde des séries de liste de polygones de type gdstk en .obj */
    void WriteLibraryToObj(const std::vector<Library>& layers, const char* filename);

    void writeObj(const char* file_name, float2* vertices, uint3* triangles, 
        size_t nb_v, size_t nb_tris);

    /* Récupère les sommets et triangles à partir d'une liste de polygones OpenCV */
    std::pair<float3*, uint3*> GetVertexAndTriangles(std::vector<std::vector<cv::Point2f>>& polys);

    /* Charge les sommets d'un .obj */
    std::pair<std::vector<cv::Point2f>, std::vector<uint3>> LoadObjVerticesTriangles(const char* filename);
}