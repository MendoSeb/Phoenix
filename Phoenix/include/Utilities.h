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
    std::vector<earcutLayer> EarcutTriangulation(std::vector<Library>& layers);

    void WriteLayersObj(std::vector<earcutLayer>& layers, const char* filename);

    void WriteLibraryToObj(const std::vector<Library>& layers, const char* filename);
}