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


namespace Utils
{
    /* Applique la triangulation earcut à une série de liste de polygones de type gdstk */
    std::vector<earcutLayer> EarcutTriangulation(std::vector<Library>& layers);

    /* Sauvegarde des séries de liste de polygones de type earcut en .obj */
    void WriteLayersObj(std::vector<earcutLayer>& layers, const char* filename);

    /* Sauvegarde des séries de liste de polygones de type gdstk en .obj */
    void WriteLibraryToObj(const std::vector<Library>& layers, const char* filename);
}