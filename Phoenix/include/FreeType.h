#pragma once
#include <ft2build.h>
#include FT_FREETYPE_H
#include <string>
#include <clipper2/clipper.engine.h>
#include <Clipper2Utils.h>
#include <vector_types.h>


using namespace Clipper2Lib;


class FreeType
{
private:
	FT_Library library;
	FT_Face     face;

public:
	FreeType();

	void StringToPolygons(std::string text);
	PathsD CharacterToPolygons(const char c);

	static int MoveTo(const FT_Vector* to, void* poly);
	static int LineTo(const FT_Vector* to, void* poly);
	static int ConicTo(const FT_Vector* control, const FT_Vector* to, void* poly);
	static int CubicTo(const FT_Vector* control1, const FT_Vector* control2, const FT_Vector* to, void* poly);
};