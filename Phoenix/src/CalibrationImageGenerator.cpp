#include "CalibrationImageGenerator.h"
#include <cmath>
#include <corecrt_math_defines.h>
#include <Optix.h>


bool CalibrationImageGenerator::IsPointInTriangle(const float2& pixel, float2(&tri)[3])
{
	auto sign = [](float x1, float y1, float x2, float y2, float x3, float y3)
		{
			return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
		};

	float d1 = sign(pixel.x, pixel.y, tri[0].x, tri[0].y, tri[1].x, tri[1].y);
	float d2 = sign(pixel.x, pixel.y, tri[1].x, tri[1].y, tri[2].x, tri[2].y);
	float d3 = sign(pixel.x, pixel.y, tri[2].x, tri[2].y, tri[0].x, tri[0].y);

	bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
	bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

	return !(has_neg && has_pos);
}


void CalibrationImageGenerator::GenerateFocusImage(
	int nb_marker_y,
	int img_width,
	int img_height,
	int nb_star_branch
)
{
	int spacing = 10;
	int marker_diameter = std::floor((img_height - nb_marker_y * spacing) / nb_marker_y);
	float marker_radius = marker_diameter / 2.0f;

	int nb_marker_x = img_width / (marker_diameter + spacing);

	int pixels_left_height = img_height - (nb_marker_y * marker_diameter);
	int pixels_left_width = img_width - (nb_marker_x * marker_diameter);
	int spacing_x = pixels_left_width / nb_marker_x;
	int spacing_y = pixels_left_height / nb_marker_y;

	// create the siemens star
	unsigned char* marker = new unsigned char[marker_diameter * marker_diameter]();
	float step_angle = 2.0 * M_PI / (nb_star_branch * 2.0);
	int nb_pixels_outer_branch = std::ceil(2.0 * M_PI * marker_radius) / (nb_star_branch * 2.0f);
	float2 center{ (float)marker_diameter / 2.0f, (float)marker_diameter / 2.0f };

	unsigned char* img = new unsigned char[img_width * img_height](0);
	float2* vertices = new float2[nb_star_branch * 2 + 1];
	uint3* triangles = new uint3[nb_star_branch];

	// créer les triangles correspondants aux branches
	int v_counter = 1;
	int t_counter = 0;
	vertices[0] = center;

	for (float a = 0.0f; a < 2 * M_PI; a += step_angle * 2)
	{
		float2 p1{ center.x + cos(a) * marker_radius, center.y + sin(a) * marker_radius };
		float2 p2{ center.x + cos(a + step_angle) * marker_radius, center.y + sin(a + step_angle) * marker_radius };

		vertices[v_counter] = p1;
		vertices[v_counter + 1] = p2;

		triangles[t_counter].x = 0;
		triangles[t_counter].y = v_counter;
		triangles[t_counter].z = v_counter + 1;

		v_counter += 2;
		t_counter++;
	}

	// draw the marker with the triangles
	for (int x = 0; x < marker_diameter; x++) {
		for (int y = 0; y < marker_diameter; y++)
		{
			float2 pixel{ x, y };

			for (int i = 0; i < nb_star_branch; i++)
			{
				float2 tri[3] = {
					vertices[triangles[i].x],
					vertices[triangles[i].y],
					vertices[triangles[i].z]
				};

				if (IsPointInTriangle(pixel, tri))
				{
					marker[y * marker_diameter + x] = 255;
					break;
				}
			}
		}
	}

	// place markers in the image
	for (int x = 0; x < nb_marker_x; x++) {
		for (int y = 0; y < nb_marker_y; y++)
		{
			int offset_x = x * (marker_diameter + spacing_x) + spacing_x / 2.0;
			int offset_y = y * (marker_diameter + spacing_y) + spacing_y / 2.0;

			for (int lx = 0; lx < marker_diameter; lx++) {
				for (int ly = 0; ly < marker_diameter; ly++)
				{
					float2 pixel{ offset_x + lx, offset_y + ly };

					img[(int)pixel.y * img_width + (int)pixel.x] = marker[ly * marker_diameter + lx];
				}
			}
		}
	}

	CalibrationImageGenerator::saveToBmp("C:/Users/PC/Desktop/poc/FocusImage.bmp", img_width, img_height, img);

	delete[] vertices;
	delete[] triangles;
	delete[] marker;
	delete[] img;
}


unsigned char* CalibrationImageGenerator::GenerateDistorsionMarker(
	int marker_diameter
)
{
	int marker_radius = (float)marker_diameter / 2.0f;
	float2 center{ (float)marker_diameter / 2.0f, (float)marker_diameter / 2.0f };
	int nb_points_circle = 100;
	float step_angle = 2.0 * M_PI / nb_points_circle;

	unsigned char* marker = new unsigned char[marker_diameter * marker_diameter]();
	std::vector<float2> vertices;
	std::vector<uint3> triangles;

	for (float a = 0.0f; a < 2.0 * M_PI; a += step_angle)
	{
		// outer torus
		float2 p1{ center.x + cos(a) * marker_radius, center.y + sin(a) * marker_radius };
		float2 p2{ center.x + cos(a + step_angle) * marker_radius, center.y + sin(a + step_angle) * marker_radius };
		float2 p3{ center.x + cos(a) * marker_radius * 0.8, center.y + sin(a) * marker_radius * 0.8 };
		float2 p4{ center.x + cos(a + step_angle) * marker_radius * 0.8, center.y + sin(a + step_angle) * marker_radius * 0.8 };

		triangles.push_back(uint3{ (uint)vertices.size(), (uint)vertices.size() + 1, (uint)vertices.size() + 2 });
		vertices.push_back(p1);
		vertices.push_back(p2);
		vertices.push_back(p3);

		triangles.push_back(uint3{ (uint)vertices.size(), (uint)vertices.size() + 1, (uint)vertices.size() + 2 });
		vertices.push_back(p2);
		vertices.push_back(p3);
		vertices.push_back(p4);

		// inner torus
		float2 pi1{ center.x + cos(a) * marker_radius * 0.6, center.y + sin(a) * marker_radius * 0.6 };
		float2 pi2{ center.x + cos(a + step_angle) * marker_radius * 0.6, center.y + sin(a + step_angle) * marker_radius * 0.6 };
		float2 pi3{ center.x + cos(a) * marker_radius * 0.4, center.y + sin(a) * marker_radius * 0.4 };
		float2 pi4{ center.x + cos(a + step_angle) * marker_radius * 0.4, center.y + sin(a + step_angle) * marker_radius * 0.4 };

		triangles.push_back(uint3{ (uint)vertices.size(), (uint)vertices.size() + 1, (uint)vertices.size() + 2 });
		vertices.push_back(pi1);
		vertices.push_back(pi2);
		vertices.push_back(pi3);

		triangles.push_back(uint3{ (uint)vertices.size(), (uint)vertices.size() + 1, (uint)vertices.size() + 2 });
		vertices.push_back(pi2);
		vertices.push_back(pi3);
		vertices.push_back(pi4);
	}

	// draw the marker with the triangles
	for (int x = 0; x < marker_diameter; x++) {
		for (int y = 0; y < marker_diameter; y++)
		{
			float2 pixel{ x, y };

			for (int i = 0; i < triangles.size(); i++)
			{
				float2 tri[3] = {
					vertices[triangles[i].x],
					vertices[triangles[i].y],
					vertices[triangles[i].z]
				};

				if (IsPointInTriangle(pixel, tri))
				{
					marker[y * marker_diameter + x] = 255;
					break;
				}
			}
		}
	}

	return marker;
}

void CalibrationImageGenerator::GenerateDistorsionImage(
	int nb_marker_y,
	int img_width,
	int img_height
)
{
	int spacing = 10;
	int marker_diameter = std::floor((img_height - (nb_marker_y - 1) * spacing) / (nb_marker_y - 1));
	int marker_radius = (float)marker_diameter / 2.0f;

	int nb_marker_x = std::floor(img_width / (marker_diameter + spacing));

	int pixels_left_height = img_height - ((nb_marker_y - 1) * marker_diameter);
	int pixels_left_width = img_width - ((nb_marker_x - 1) * marker_diameter);
	int spacing_x = pixels_left_width / (nb_marker_x - 1);
	int spacing_y = pixels_left_height / (nb_marker_y - 1);

	unsigned char* img = new unsigned char[img_width * img_height]();
	unsigned char* marker = GenerateDistorsionMarker(marker_diameter);

	// place markers in the image
	for (int x = 0; x < nb_marker_x; x++) {
		for (int y = 0; y < nb_marker_y; y++)
		{
			int offset_x = x * (marker_diameter + spacing_x) - marker_radius;
			int offset_y = y * (marker_diameter + spacing_y) - marker_radius;

			for (int lx = 0; lx < marker_diameter; lx++) {
				for (int ly = 0; ly < marker_diameter; ly++)
				{
					float2 pixel{ offset_x + lx, offset_y + ly };
					int pixel_index_img = (int)pixel.y * img_width + (int)pixel.x;
					int pixel_index_marker = ly * marker_diameter + lx;

					if (pixel.x >= 0 && pixel.x < img_width
						&& pixel.y >= 0 && pixel.y < img_height)
					{
						img[pixel_index_img] = marker[pixel_index_marker];
					}
				}
			}
		}
	}

	saveToBmp("C:/Users/PC/Desktop/poc/DistorsionImage.bmp", img_width, img_height, img);

	delete[] marker;
	delete[] img;
}


void CalibrationImageGenerator::GenerateCameraAngleImage(
	int camera_width,
	int camera_height,
	int img_width,
	int img_height
)
{
	int marker_diameter = (camera_height * 1.85) / 30.0;
	int spacing_x = ((camera_width - camera_height) * 1.85) / 30.0;

	unsigned char* marker = GenerateDistorsionMarker(marker_diameter);
	unsigned char* img = new unsigned char[img_width * img_height]();

	// place markers in the image
	float2 markers_pos[2] = {
		{img_width / 2.0 - spacing_x / 2.0 - marker_diameter, img_height / 2.0 - marker_diameter / 2.0},
		{img_width / 2.0 + spacing_x / 2.0, img_height / 2.0 - marker_diameter / 2.0}
	};

	for (float2& pos : markers_pos) {
		for (int lx = 0; lx < marker_diameter; lx++) {
			for (int ly = 0; ly < marker_diameter; ly++)
			{
				float2 pixel{ pos.x + lx, pos.y + ly };

				if (pixel.x >= 0 && pixel.x < img_width && pixel.y >= 0 && pixel.y < img_height)
				{
					img[(int)pixel.y * img_width + (int)pixel.x] = marker[ly * marker_diameter + lx];
				}
			}
		}
	}

	saveToBmp("C:/Users/PC/Desktop/poc/CameraAngleImage.bmp", img_width, img_height, img);

	delete[] marker;
	delete[] img;
}


void CalibrationImageGenerator::saveToBmp(const std::string& filename, int width, int height,
	unsigned char* hostData)
{
	// 2. Prepare BMP Headers and Palette
	BitmapFileHeader fileHeader;
	BitmapInfoHeader infoHeader;

	infoHeader.width = width;
	infoHeader.height = height;
	infoHeader.bit_count = 8;
	infoHeader.size = sizeof(BitmapInfoHeader);

	uint32_t palette_size = 256 * 4; // 256 grayscale entries, 4 bytes each
	fileHeader.offset_data = sizeof(BitmapFileHeader)
		+ sizeof(BitmapInfoHeader) + palette_size;

	fileHeader.file_size = fileHeader.offset_data + (width * height);

	std::vector<char> palette(palette_size);
	for (int i = 0; i < 256; ++i) {
		palette[i * 4 + 0] = i; // Blue
		palette[i * 4 + 1] = i; // Green
		palette[i * 4 + 2] = i; // Red
		palette[i * 4 + 3] = 0;  // Reserved
	}

	// 3. Write data to the file
	std::ofstream file(filename, std::ios::binary);
	if (!file.is_open()) {
		std::cerr << "Error: Could not open file for writing." << std::endl;
		delete[] hostData;
		return;
	}

	file.write(reinterpret_cast<const char*>(&fileHeader), sizeof(fileHeader));
	file.write(reinterpret_cast<const char*>(&infoHeader), sizeof(infoHeader));
	file.write(palette.data(), palette.size());
	file.write(reinterpret_cast<const char*>(hostData), width * height);

	file.close();
}
