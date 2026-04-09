#pragma once
#include "vector_types.h"


struct SimulationParams
{
    int dmd_width = 4096;
    int dmd_height = 2176;

    int img_width = 4096;
    int img_height = 2176;

    int sp = 8 * 8;
    int resolution = 16;
};