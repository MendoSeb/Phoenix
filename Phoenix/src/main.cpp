#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>  

#include "BoostUtils.h"
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
	//boostGeometryDemo();
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

	float inch_x = (29.5f * 2.0f) / 2.54f;
	float inch_y = (16.8f * 4.0f) / 2.54f;
	MonoLayerRasterization({inch_x, inch_y}, 1200);
	//MultiLayerRasterization({ inch_x, inch_y }, 100);

	//FreeType ft;
	//ft.StringToPolygons("MGI test\nenorme");

	//ImageToSVG();
	
	return 0;
}
