#pragma once
#include <string>
#include <vector_types.h>


namespace CalibrationImageGenerator
{
	bool IsPointInTriangle(const float2& pixel, float2(&tri)[3]);


	void GenerateFocusImage(
		int nb_marker_y, 
		int img_width, 
		int img_height, 
		int nb_star_branch
	);


	unsigned char* GenerateDistorsionMarker(
		int marker_diameter
	);


	void GenerateDistorsionImage(
		int nb_marker_y,
		int img_width,
		int img_height
	);


	void GenerateCameraAngleImage(
		int camera_width,
		int camera_height,
		int img_width,
		int img_height
	);

	void saveToBmp(const std::string& filename, int width, int height, unsigned char* hostData);
}