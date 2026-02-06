#pragma once
#include <GdstkUtils.h>
#include <mutex>


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

	// Accélération de la transformation des polygones
	void threadWarpingDemo
	(
		Library& lib,
		int index_box,
		std::vector<std::vector<cv::Point2f>>& polys_in_box,
		std::mutex& mutex
	);

	// déformation sur des polygones triangulés .gds
	void warpingDemo1();

	// déformation puis triangulation des polygones .gds
	void warpingDemo2();

	// déformation d'un .obj
	void warpingDemo3();

	// parsing d'un dossier odb++ et enregistrement des résultats en .gds
	void odbDemo();

	// trouve les bornes des cibles selon la position du nombre de la cible
	std::vector<double> findMinMax(cv::Mat& frame, bool text_down);

	// extrait les images des cibles de la vidéo
	void video();

	// essai avec Clipper2 des opérations binaires faites avec Artwork auparavant
	void essaiClient();

	void essaiClientComparison();
}