#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <DirectImagingSimulator.h>
#include <vector_types.h>

//attention à ne pas passer en référence des paramètres vers le gpu car l'adresse est sur cpu!
__global__ void WriteSimulationImage(Params hparam, float* dimg, float* dsim_img, int shift)
{
    // 1. Coordonnées de pixel
    const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    float3 ray_origin {
        hparam.cam_position.x + (float)x,
        hparam.cam_position.y + (float)y,
    };

    int res = 7;
    int start = -(float)res / 2.0f;
    int end = (float)res / 2.0f + 1;

    float coef_luminance = hparam.luminance_matrix[y * hparam.dmd_width + x];
    unsigned char activated = hparam.luminance_correction[y * hparam.dmd_width + x];

    int temp_x_dmd = x;
    int temp_y_dmd = y + shift;
    int pixel_index_dmd = temp_y_dmd * hparam.dmd_width + temp_x_dmd;

    for (int lx = start; lx < end; lx++) {
        for (int ly = start; ly < end; ly++) {

            int temp_x_img = std::round(ray_origin.x * res) + lx;
            int temp_y_img = std::round(ray_origin.y * res) + ly;

            // modifier le pixel que si on est dans l'image
            if (temp_x_img >= 0 && temp_x_img < hparam.img_width
                && temp_y_img >= 0 && temp_y_img < hparam.img_height
                && temp_x_dmd >= 0 && temp_x_dmd < hparam.dmd_width
                && temp_y_dmd >= 0 && temp_y_dmd < hparam.dmd_height)
            {
                int pixel_index = temp_y_img * hparam.img_width + temp_x_img;
                dsim_img[pixel_index] += dimg[pixel_index_dmd] * coef_luminance * activated;
            }
        }
    }
}


void DirectImagingSimulator::CudaWriteImage(
    Params& hparam,
    int& dmd_width,
    int& dmd_height,
    float* dimg,
    float* dsim_img,
    int& shift
)
{
    int nb_blocks_x = std::ceil((float)dmd_width / 32.0f);
    int nb_blocks_y = std::ceil((float)dmd_height / 32.0f);
    WriteSimulationImage <<< dim3(nb_blocks_x, nb_blocks_y, 1), dim3(32, 32, 1) >>> (hparam, dimg, dsim_img, shift);
}