#include "GdstkOperators.h"
#include <iostream>
#include <__msvc_chrono.hpp>


GdstkOperators::GdstkOperators()
{
}


Library GdstkOperators::MakeUnion(Library& lib)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    gdstk::Library out_lib = {};
    out_lib.init("library", 1e-6, 1e-9);

    for (size_t i = 0; i < lib.cell_array.count; i++)
    {
        Cell* out_cell = (Cell*)allocate_clear(sizeof(Cell));
        out_cell->name = copy_string("cellule", NULL);

        boolean(lib.cell_array[i]->polygon_array, out_cell->polygon_array, Operation::Or, 1e3, out_cell->polygon_array);
        out_lib.cell_array.append(out_cell);
    }
    
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Union faite en "  << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count()  << std::endl;
    return out_lib;
}


Library GdstkOperators::MakeDegraissement(Library lib, double dist)
{;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    gdstk::Library out_lib = {};
    out_lib.init("library", 1e-6, 1e-9);

    for (size_t i = 0; i < lib.cell_array.count; i++)
    {
        Cell* out_cell = (Cell*)allocate_clear(sizeof(Cell));
        out_cell->name = copy_string("cellule", NULL);

        // dégraissement
        gdstk::offset(
            lib.cell_array[i]->polygon_array,
            dist,
            OffsetJoin::Miter,
            2,
            1e3,
            false,
            out_cell->polygon_array
        );
        
        out_lib.cell_array.append(out_cell);
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Dégraissement fait en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << std::endl;
    return out_lib;
}


Library GdstkOperators::MakeDifference(Library& lib1, Library& lib2)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    gdstk::Library out_lib = {};
    out_lib.init("library", 1e-6, 1e-9);

    assert(lib1.cell_array.count == lib2.cell_array.count);

    for (size_t i = 0; i < lib1.cell_array.count; i++)
    {
        Cell* out_cell = (Cell*)allocate_clear(sizeof(Cell));
        out_cell->name = copy_string("cellule", NULL);

        boolean(
            lib1.cell_array[i]->polygon_array,
            lib2.cell_array[i]->polygon_array,
            Operation::Not,
            1e3,
            out_cell->polygon_array
        );

        out_lib.cell_array.append(out_cell);
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Différence faite en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << std::endl;
    return out_lib;
}