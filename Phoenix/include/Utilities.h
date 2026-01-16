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

    /* Sauvegarde des séries de liste de polygones de type earcut en .obj */
    void WriteLayersObj(std::vector<earcutLayer>& layers, const char* filename);

    /* Sauvegarde des séries de liste de polygones de type gdstk en .obj */
    void WriteLibraryToObj(const std::vector<Library>& layers, const char* filename);

    /* Récupère les sommets et triangles à partir d'une liste de polygones OpenCV */
    std::pair<float3*, uint3*> GetVertexAndTriangles(std::vector<std::vector<cv::Point2f>>& polys);

    /* Charge les sommets d'un .obj */
    std::pair<std::vector<cv::Point2f>, std::vector<uint3>> LoadObjVerticesTriangles(const char* filename);
}