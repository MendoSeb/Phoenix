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
        params.cam_eye.x + x,
        params.cam_eye.y + y, 
        params.cam_eye.z 
    };

    float3 ray_direction = float3{ 0.0f, 0.0f, -1.0f };

    float t_min = 0.0f;
    float t_max = 1e16f;
    unsigned int p0;
        
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

    for (int ix = 0; ix < params.sp; ix++)
    {
        for (int iy = 0; iy < params.sp; iy++)
        {
            int pixel_index = ((int)(ray_origin.y + params.min_y) * params.sp + iy + params.y_sp_index) * params.image_width
                + ((int)(ray_origin.x + params.min_x) * params.sp + ix + params.x_sp_index);

            if (pixel_index >= 0 && pixel_index < params.total_pixels)
                params.image[pixel_index] |= p0;
        }
    }
}


extern "C" __global__ void __closesthit__ch() {
    //const HitGroupData* sbt_data = (const HitGroupData*)optixGetSbtDataPointer();
    //const int tri_id = optixGetPrimitiveIndex();
    //optixSetPayload_0(sbt_data->triangles_type[tri_id]);     // pour l'obj multi couche

    optixSetPayload_0(255);    // pour l'obj une seule couche
}


extern "C" __global__ void __miss__ms() {
    optixSetPayload_0(0);
} 