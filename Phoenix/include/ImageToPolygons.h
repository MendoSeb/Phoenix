#pragma once


namespace ImageToPolygons
{
	void ExtractContours(const char* filepath);

	void ThresholdImage(unsigned char* img, int& width, int& height, int& pixel_component);

	int CountBlackPixels(unsigned char* img, int& width, int& height, int& threshold);

	void ConvertImageToPolygons(const char* filepath);

	void ConvertBMPToPolygons(const char* filepath);
}