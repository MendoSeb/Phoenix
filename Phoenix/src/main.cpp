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

	//gpu();

	//float inch_x = (24.7f * 2.0f) / 2.54f; // pour le circuit 58a0 je crois
	//float inch_y = (16.8f * 3.5f) / 2.54f;
	//MultiLayerRasterization({ inch_x, inch_y }, 1200);

	FreeType ft;
	ft.StringToPolygons("abcdefghijklmnopqrstuvwxyz\nABCDEFGHIJKLMNOPQRSTUVWXYZ\n1234567890");

	//BMPToGDS();
	
	return 0;
}
