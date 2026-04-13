#include <optix.h>
#include <optix_device.h>
#include <vector_types.h>
#include "vec_math.h"


extern "C"
{
	__constant__ Params dparam; // données à transmettre aux gpu (uniforms)
}


extern "C" __global__ void __raygen__rg()
{
	// 1. Coordonnées de pixel
	const unsigned int x = optixGetLaunchIndex().x;
	const unsigned int y = optixGetLaunchIndex().y;
	const unsigned int launch_dim_x = optixGetLaunchDimensions().x;
	const unsigned int launch_dim_y = optixGetLaunchDimensions().y;

	float3 ray_origin = float3{
		dparam.cam_position.x + (float)x,
		dparam.cam_position.y + (float)y,
		dparam.cam_position.z
	};

	float3 ray_direction { 0.0f, 0.0f, -1.0f };

	float t_min = 0.0f;
	float t_max = FLT_MAX;
	unsigned int p0 = 0; // payload 0

	optixTrace(
		dparam.handle,      // Les paramètres de la caméra (pourrait être un espace vide)
		ray_origin,         // Point d'origine du rayon
		ray_direction,      // Direction du rayon (avec distorsion)
		t_min,
		t_max,
		0.0f,               // Temps (pour le flou de mouvement)
		OptixVisibilityMask(255),
		OPTIX_RAY_FLAG_NONE, // fragment shader pour le triangle le plus proche
		0,                  // ID du pipeline
		1,                  // ID du hitgroup
		0,                  // ID du programme de miss
		p0                  // Pointeur de données de payload
	);

	int pixel_index = y * dparam.dmd_width + x;

	// modifier le pixel que si on est dans l'image
	if (x >= 0 && x < dparam.dmd_width
		&& y >= 0 && y < dparam.dmd_height + dparam.dmd_height_spacing)
	{
		dparam.image[pixel_index] = p0;
	}
}


extern "C" __global__ void __closesthit__ch()
{
	const int tri_id = optixGetPrimitiveIndex();
	optixSetPayload_0(dparam.polarity[tri_id]);
}


extern "C" __global__ void __miss__ms()
{
	optixSetPayload_0(100);
}