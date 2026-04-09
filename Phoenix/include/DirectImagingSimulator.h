#pragma once
#include <SimulationParams.h>
#include <TriangulationUtils.h>


class DirectImagingSimulator
{
private:
    SimulationParams sp;

public:

    DirectImagingSimulator();
    ~DirectImagingSimulator();

    float* CreateLuminanceMatrix(int width, int height);

    unsigned char* CreateLuminanceCorrectionMatrix(int width, int height, float* luminance_matrix);

    void SimulateLithography();
};