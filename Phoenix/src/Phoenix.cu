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

    //int sp = params.sp;
    unsigned int sp = 1;

    for (int ix = 0; ix < sp; ix++)
    {
        for (int iy = 0; iy < sp; iy++)
        {
            int pixel_index =
                y * sp // calcul l'emplacement du pixel de taille: sp X sp (en y)
                // ajoute le décalage pour parcourir les voisins (en y)
                * params.image_width // pour enregistrer dans le tableau 1D
                + x * sp;  // calcul l'emplacement du pixel de taille: sp X sp (en x)
                 // ajoute le décalage pour parcourir les voisins (en x)

            // modifier le pixel que si on est dans l'image
            if (pixel_index >= 0 && pixel_index < params.total_pixels)
                params.image[pixel_index] = p0;
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