#include <optix.h>
#include <optix_device.h> // AJOUTÉ : Nécessaire pour optixGetLaunchIndex, optixTrace, etc.
#include <vector_types.h>
#include "vec_math.h"
#include "Optix.h"


extern "C" {
    __constant__ Params params; // données à transmettre aux gpu (uniforms)
}


extern "C" __global__ void __raygen__rg() {
    // Obtenir les dimensions de la grille et de la scène
// Accéder aux données de la camédra

    // 1. Coordonnées de pixel
    const unsigned int x = optixGetLaunchIndex().x;
    const unsigned int y = optixGetLaunchIndex().y;
    const unsigned int launch_dim_x = optixGetLaunchDimensions().x;
    const unsigned int launch_dim_y = optixGetLaunchDimensions().y;

    // 2. Coordonnées normalisées dans l'espace de l'image (-1 à 1)
    float ratio = 2176.0f / 4096.0f;

    float3 ray_origin = float3{ 
        params.cam_eye.x + x,
        params.cam_eye.y + y, 
        params.cam_eye.z 
    };

    float3 ray_direction = float3{ 0.0f, 0.0f, -1.0f };

    float t_min = 0.0f;
    float t_max = 1e16f;

    int x2 = ray_direction.x;
    int y2 = ray_direction.y;

    unsigned int visibility_mask = 1;
    unsigned int Val = 0; 
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
        p0           // Pointeur de données de payload
    );
    Val |= p0;

    int pixel_index = (int)ray_origin.y * params.image_width + (int)ray_origin.x;

    if (pixel_index >= 0 && pixel_index < params.total_pixels)
        params.image[pixel_index] = Val;
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