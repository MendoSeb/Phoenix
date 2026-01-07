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
    float ratio = 4096.0f / 2176.0f;
    float zoom = 0.0f;

    float u_normalized = (((float)x / (float)launch_dim_x * 2.0f - 1.0f) * ratio) / zoom;
    float v_normalized = (((float)y / (float)launch_dim_y * 2.0f - 1.0f)) / zoom;

    float3 ray_origin = float3{ u_normalized + params.cam_eye.x, v_normalized + params.cam_eye.y, params.cam_eye.z };
    //float3 ray_direction = params.cam_w + u_normalized * params.cam_u + v_normalized * params.cam_v;
    float3 ray_direction = float3{ 0.0f, 0.0f, -1.0f };

    float t_min = 0.0f;
    float t_max = 1e16f;

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

    params.image[y * params.image_width + x] = Val;

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