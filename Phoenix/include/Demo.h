#pragma once
#include <GdstkUtils.h>
#include <mutex>
#include <vector_types.h>


namespace Demo
{
	// Démo d'opérations binaires BoostGeometry sur les polygones
	void boostGeometryDemo();

	// Démo d'opérations binaires Clipper2 sur les polygones
	void clipper2Demo();

	// Démo d'opérations binaires gdstk sur les polygones
	void gdstkDemo();

	// Démonstration du déplacement du dmd (caméra) et lancer de rayons sur un circuit triangulé
	void optixDemo();

	// déformation sur des polygones triangulés .gds
	void warpingDemo1();

	// déformation puis triangulation des polygones .gds
	void warpingDemo2();

	// déformation d'un .obj
	void warpingDemo3();

	// parsing d'un dossier odb++ et enregistrement des résultats en .gds
	void odbDemo();

	// essai avec Clipper2 des opérations binaires faites avec Artwork auparavant
	void essaiClient();

	void essaiClientComparison();

	void rasterisationStep();

	void AltijetRasterization(float2 circuit_inch_size, int dpi);
}