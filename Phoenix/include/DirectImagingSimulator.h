#pragma once
#include <SimulationParams.h>
#include <TriangulationUtils.h>
#include <Optix.h>


class DirectImagingSimulator
{
private:
    SimulationParams sp;

public:

    DirectImagingSimulator();
    ~DirectImagingSimulator();

    float* CreateLuminanceMatrix(int width, int height);

    unsigned char* CreateLuminanceCorrectionMatrix(int width, int height, float* luminance_matrix);

    void CreateImagesBuffer(
        float*& sim_img,
        float*& dimg,
        float*& dluminance_matrix,
        unsigned char*& dluminance_correction
    );

    void SetOptixParams(
        CUdeviceptr& dparam,
        Params& hparam,
        float* dimg,
        float* dluminance_matrix,
        unsigned char* dluminance_correction,
        TrisUtils::Triangulation& tris,
        Optix& o
    );

    void CudaWriteImage(
        Params& hparam,
        int& dmd_width,
        int& dmd_height,
        float* dimg,
        float* dsim_img,
        int& shift
    );

    void SaveSimImageAsBmp(
        Optix& o,
        int& img_width,
        int& img_height,
        float* dsim_img
    );

    void SimulateLithography();
};