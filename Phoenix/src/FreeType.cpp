#include "FreeType.h"
#include <iostream>
#include "RasterizationStep.h"
#include <freetype/ftbitmap.h>
#include <freetype/ftoutln.h>


FreeType::FreeType()
{
	FT_Error error = FT_Init_FreeType(&library);

	if (error)
		printf("erreur initialisation freetype 1\n");

	error = FT_New_Face(library,
		//"C:/Users/PC/Downloads/jurassic-park/Jurassic_Park.ttf",
		"C:/Users/PC/Downloads/arial/ARIAL.TTF",
		0,
		&face);

	if (error == FT_Err_Unknown_File_Format)
		printf("erreur initialisation freetype 2\n");

	else if (error)
		printf("erreur initialisation freetype 3\n");

	error = FT_Set_Pixel_Sizes(
		face,   /* handle to face object */
		0,      /* pixel_width           */
		10);   /* pixel_height          */
}


void FreeType::StringToPolygons(std::string text)
{
	PathsD paths;
	PointD char_offset(0, 0);

	for (int i = 0; i < text.size(); i++)
	{
		char c = text[i];

		if (c == '\n')
		{
			char_offset.x = 0;
			char_offset.y -= face->size->metrics.height;
		}
		else
		{
			PathsD char_paths = CharacterToPolygons(c);

			for (PathD& path : char_paths) {
				for (PointD& point : path)
				{
					point.x += char_offset.x;
					point.y += char_offset.y;
				}
			}

			for (PathD& path : char_paths) {
				paths.push_back(path);
			}

			char_offset.x += face->glyph->advance.x;
		}
	}

	PolyTreeD out;
	Clipper2Utils::MakeUnion(paths, out);

	Library out_lib = {};
	Clipper2Utils::ConvertPolyTreeDToGdsiiPath(out, out_lib);
	GdstkUtils::SaveToGdsii(out_lib, "C:/Users/PC/Desktop/poc/freetype.gds", false);
}


PathsD FreeType::CharacterToPolygons(const char c)
{
	FT_UInt glyph_index = FT_Get_Char_Index(face, c);

	FT_Error error = FT_Load_Glyph(
		face,          /* handle to face object */
		glyph_index,   /* glyph index           */
		FT_LOAD_NO_HINTING); // poiur indiquer de ne pas placer les points par rapport aux pixels

	// functions for decomposition
	FT_Outline_Funcs funcs;
	funcs.move_to = MoveTo; // doit être statique pour que la fonction puisse être appelée quand on veut
	funcs.line_to = LineTo;
	funcs.conic_to = ConicTo;
	funcs.cubic_to = CubicTo;
	funcs.shift = 0;
	funcs.delta = 0;
	PathsD paths;

	if (FT_Outline_Decompose(&face->glyph->outline, &funcs, &paths))
		std::cout << "erreur\n";

	return paths;
}

int FreeType::MoveTo(const FT_Vector* to, void* poly)
{
	PathsD* paths = static_cast<PathsD*>(poly);
	PathD path;

	path.push_back(PointD{to->x, to->y});
	paths->push_back(path);

	//printf("dans move to x: %i, x: %i\n", to->x, to->y);
	return 0;
}


int FreeType::LineTo(const FT_Vector* to, void* poly)
{
	PathD* path = &static_cast<PathsD*>(poly)->back();
	path->push_back(PointD{ to->x, to->y });

	return 0;
}


int FreeType::ConicTo(const FT_Vector* control, const FT_Vector* to, void* poly)
{
	//printf("In degree two bezier curve\n");
	PathD* path = &static_cast<PathsD*>(poly)->back();

	PointD p0 = path->back();
	PointD p1 { control->x, control->y };
	PointD p2 { to->x, to->y };
	float t_step = 0.05f;

	for (float t = t_step; t <= 1.0f; t += t_step)
	{
		PointD temp;
		temp.x = p0.x * std::pow(1.0f - t, 2) + 2.0f * (1.0f - t) * t * p1.x + t*t*p2.x;
		temp.y = p0.y * std::pow(1.0f - t, 2) + 2.0f * (1.0f - t) * t * p1.y + t*t*p2.y;
		path->push_back(temp);
	}

	return 0;
}


int FreeType::CubicTo(const FT_Vector* control1, const FT_Vector* control2, const FT_Vector* to, void* poly)
{
	//printf("In degree three bezier curve\n");
	PathD* path = &static_cast<PathsD*>(poly)->back();

	PointD p0 = path->back();
	PointD p1{ control1->x, control1->y };
	PointD p2{ control2->x, control2->y };
	PointD p3{ to->x, to->y };
	float t_step = 0.05f;

	for (float t = t_step; t <= 1.0f; t += t_step)
	{
		PointD temp;
		temp.x = std::pow(1.0f - t, 3) * p0.x + 3 * std::pow(1.0f - t, 2) * t * p1.x
			+ 3 * (1.0f - t) * t * t * p2.x + t * t * t * p3.x;

		temp.y = std::pow(1.0f - t, 3) * p0.y + 3 * std::pow(1.0f - t, 2) * t * p1.y
			+ 3 * (1.0f - t) * t * t * p2.y + t * t * t * p3.y;

		path->push_back(temp);
	}

	return 0;
}