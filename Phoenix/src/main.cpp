#include <stdio.h>

#include "Clipper2Utils.h"
#include "GdstkUtils.h"
#include "earcut.hpp"
#include "Utilities.h"
#include "Optix.h"
#include "ODB++Parser.h"
#include "Demo.h"
#include <FreeType.h>

using namespace Demo;


int main()
{
	//clipper2Demo();
	//gdstkDemo();

	//optixDemo();

	//warpingDemo1();
	//warpingDemo2();
	//warpingDemo3();

	//odbDemo();

	//video();

	//essaiClient();
	//essaiClientComparison();

	//rasterisationStep();

	//gpu();

	/*float inch_x = (29.5f * 2.0f) / 2.54f; // pour le circuit 58a0 je crois
	float inch_y = (16.8f * 4.0f) / 2.54f;
	MultiLayerRasterization({ inch_x, inch_y }, 1200); */

	float2 circuit_dim{10, 10};
	int dpi = 1200;
	MultiLayerRasterization(circuit_dim, dpi);

	//FreeType ft;
	//ft.StringToPolygons("MGI test\nenorme");

	//ImageToSVG();
	
	return 0;
}
