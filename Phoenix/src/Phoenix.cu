#include <optix.h>
#include <optix_device.h>
#include <vector_types.h>
#include "vec_math.h"


extern "C" 
{
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

    int pixel_index = y * params.dmd_width + x;

    // modifier le pixel que si on est dans l'image
    if (x >= 0 && x < params.dmd_width
        && y >= 0 && y < params.dmd_height)
    {
        params.image[pixel_index] = p0;
    }
}


extern "C" __global__ void __closesthit__ch() 
{
    const int tri_id = optixGetPrimitiveIndex();
    optixSetPayload_0(params.polarity[tri_id]);
}


extern "C" __global__ void __miss__ms() 
{
    optixSetPayload_0(0);
}