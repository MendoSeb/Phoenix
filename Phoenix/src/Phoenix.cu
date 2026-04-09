#include <optix.h>
#include <optix_device.h> // AJOUTÉ : Nécessaire pour optixGetLaunchIndex, optixTrace, etc.
#include <vector_types.h>
#include "vec_math.h"


extern "C" {
    __constant__ Params params; // données à transmettre aux gpu (uniforms)
}


extern "C" __global__ void __raygen__rg() 
{
    // 1. Coordonnées de pixel
    const unsigned int x = optixGetLaunchIndex().x;
    const unsigned int y = optixGetLaunchIndex().y;
    const unsigned int launch_dim_x = optixGetLaunchDimensions().x;
    const unsigned int launch_dim_y = optixGetLaunchDimensions().y;

    float3 ray_origin = float3{
        params.cam_position.x + (float)x, 
        params.cam_position.y + (float)y,
        params.cam_position.z
    };

    float3 ray_direction = float3{ 0.0f, 0.0f, -1.0f };

    // ajout de la distorsion
    /* float norme = sqrt(pow(params.image_width / 2.0f, 2) + pow(params.image_height / 2.0f, 2));
    float3 distorsion = ray_origin - float3{(float)params.image_width/2.0f, (float)params.image_height/2.0f, 10.0f};
    
    float l = length(distorsion) / norme + 1.0f;
    distorsion = (distorsion / norme) * pow(l, 8); // offset entre 0 et 1 de norme

    ray_direction += distorsion;

    // correction de la distorsion
    ray_direction.x -= params.distorsion[y * params.image_width + x].x;
    ray_direction.y -= params.distorsion[y * params.image_width + x].y;

    ray_direction = normalize(ray_direction); */

    float t_min = 0.0f;
    float t_max = 1e16f;
    unsigned int p0 = 0;
        
    optixTrace(
        params.handle,      // Les paramètres de la caméra (pourrait être un espace vide)
        ray_origin,         // Point d'origine du rayon
        ray_direction,      // Direction du rayon (avec distorsion)
        t_min,
        t_max,
        0.0f,               // Temps (pour le flou de mouvement)
        OptixVisibilityMask(255),
        OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT,
        0,                  // ID du pipeline
        1,                  // ID du hitgroup
        0,                  // ID du programme de miss
        p0                  // Pointeur de données de payload
    );

    int res = 4;

    for (int lx = 0; lx < res; lx++) {
        for (int ly = 0; ly < res; ly++)
        {
            int temp_y = std::floor(ray_origin.y) * res + ly;
            int temp_x = std::floor(ray_origin.x) * res + lx;
            int pixel_index = temp_y * params.image_width + temp_x;

            // modifier le pixel que si on est dans l'image
            if (temp_x >= 0 && temp_x < params.image_width
                && temp_y >= 0 && temp_y < params.image_height)
            {
                float coef_luminance = params.luminance_matrix[y * 4096 + x];
                unsigned char activated = params.luminance_correction[y * 4096 + x];

                params.image[pixel_index] += (p0 / 255.0f) * coef_luminance * activated;
            }
        }
    }
}


extern "C" __global__ void __closesthit__ch() {

    const int tri_id = optixGetPrimitiveIndex();
    optixSetPayload_0(params.polarity[tri_id]);
}


extern "C" __global__ void __miss__ms() {
    optixSetPayload_0(0);
} 