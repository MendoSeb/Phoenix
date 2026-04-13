#include <stdio.h>

#include "Clipper2Utils.h"
#include "GdstkUtils.h"
#include "earcut.hpp"
#include "TriangulationUtils.h"
#include "Optix.h"
#include "ODB++Parser.h"
#include "Demo.h"
#include <FreeType.h>
#include <DirectImagingSimulator.h>
#include <CalibrationImageGenerator.h>

using namespace Demo;


int main()
{
	//clipper2Demo();
	//gdstkDemo();

	//optixDemo();

	DirectImagingSimulator dis;
	dis.SimulateLithography();

	/*CalibrationImageGenerator::GenerateFocusImage(11, 4096, 2176, 10);
	CalibrationImageGenerator::GenerateDistorsionImage(11, 4096, 2176);
	CalibrationImageGenerator::GenerateCameraAngleImage(4024, 3036, 4096, 2176);*/

	//warpingDemo1();
	//warpingDemo2();
	//warpingDemo3();

	//odbDemo();

	//video();

	//essaiClient();
	//essaiClientComparison();

	//gpu();

	/*float inch_x = (26.3f * 2.0f) / 2.54f; // pour le circuit 58a0
	float inch_y = (16.8f * 3.5f) / 2.54f;

	inch_x = 9.5;
	inch_y = 9.5;
	MultiLayerRasterization({ inch_x, inch_y }, 300);*/

	//FreeType ft;
	//ft.StringToPolygons("abcdefghijklmnopqrstuvwxyz\nABCDEFGHIJKLMNOPQRSTUVWXYZ\n1234567890\nun espace\n?!,;./()[]<>%^кЂ&аз'~-_+*");

	//BMPToGDS();

	return 0;
}
