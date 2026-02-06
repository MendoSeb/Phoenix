#pragma once
#include <optix.h>
#include <cuda_runtime_api.h>
#include <string>
#include <vector>
#include <driver_types.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <optix_types.h>
#include <cuda.h>


#pragma pack(push, 1)
struct BitmapFileHeader {
    uint16_t file_type{ 0x4D42 }; // 'BM'
    uint32_t file_size{ 0 };
    uint16_t reserved1{ 0 };
    uint16_t reserved2{ 0 };
    uint32_t offset_data{ 0 };
};

struct BitmapInfoHeader {
    uint32_t size{ 0 };
    int32_t width{ 0 };
    int32_t height{ 0 };
    uint16_t planes{ 1 };
    uint16_t bit_count{ 0 };
    uint32_t compression{ 0 };
    uint32_t size_image{ 0 };
    int32_t x_pixels_per_meter{ 0 };
    int32_t y_pixels_per_meter{ 0 };
    uint32_t colors_used{ 0 };
    uint32_t colors_important{ 0 };
};
#pragma pack(pop)


struct Vertex {
    float x, y, z;
};

struct Triangle {
    size_t v1, v2, v3;
};

struct RayGenData {
};


struct MissData
{
    // No data needed
};

/* Contient les données pour chaque triangle */
struct HitGroupData
{
    // No data needed
    unsigned char color = 128;
    int* triangles_type; // full or hole
};


struct Params
{
    unsigned char*         image;
    unsigned int           image_width;
    unsigned int           image_height;
    unsigned int           total_pixels;
    unsigned int           sp;
    unsigned int           y_sp_index;
    unsigned int           x_sp_index;
    float                  min_x;
    float                  min_y;
    float3                 cam_eye; // position de la caméra
    float3                 cam_u, cam_v, cam_w; // orientation de la caméra
    OptixTraversableHandle handle;
};


template <typename T>
struct SbtRecord
{
    __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    T data;
};

typedef SbtRecord<RayGenData>     RayGenSbtRecord;
typedef SbtRecord<MissData>       MissSbtRecord;
typedef SbtRecord<HitGroupData>   HitGroupSbtRecord;

class Exception : public std::runtime_error
{
public:
    Exception(const char* msg)
        : std::runtime_error(msg)
    {
    }

    Exception(OptixResult res, const char* msg)
        : std::runtime_error(createMessage(res, msg).c_str())
    {
    }

private:
    std::string createMessage(OptixResult res, const char* msg)
    {
        std::ostringstream out;
        out << res << ": " << msg;
        return out.str();
    }
};


#define OPTIX_CHECK( call )                                                    \
    optixCheck( call, #call, __FILE__, __LINE__ )


#define OPTIX_CHECK_LOG( call )                                                \
    do                                                                         \
    {                                                                          \
        char   LOG[2048];                                                      \
        size_t LOG_SIZE = sizeof( LOG );                                       \
        optixCheckLog( call, LOG, sizeof( LOG ), LOG_SIZE, #call,     \
                                __FILE__, __LINE__ );                          \
    } while( false )

#define OPTIX_CHECK_NOTHROW( call )                                            \
    optixCheckNoThrow( call, #call, __FILE__, __LINE__ )


inline void optixCheck(OptixResult res, const char* call, const char* file, unsigned int line)
{
    if (res != OPTIX_SUCCESS)
    {
        std::stringstream ss;
        ss << "Optix call '" << call << "' failed: " << file << ':' << line << ")\n";
        throw Exception(res, ss.str().c_str());
    }
}

inline void optixCheckLog(OptixResult  res,
    const char* log,
    size_t       sizeof_log,
    size_t       sizeof_log_returned,
    const char* call,
    const char* file,
    unsigned int line)
{
    if (res != OPTIX_SUCCESS)
    {
        std::stringstream ss;
        ss << "Optix call '" << call << "' failed: " << file << ':' << line << ")\nLog:\n"
            << log << (sizeof_log_returned > sizeof_log ? "<TRUNCATED>" : "") << '\n';
        throw Exception(res, ss.str().c_str());
    }
}

inline void optixCheckNoThrow(OptixResult res, const char* call, const char* file, unsigned int line) noexcept
{
    if (res != OPTIX_SUCCESS)
    {
        try
        {
            std::cerr << "Optix call '" << call << "' failed: " << file << ':'
                << line << ")\n";
        }
        catch (...)
        {
        }
        std::terminate();
    }
}


class Optix
{
private:
    int width, height; // dimension de l'image à calculer
    std::pair<float, float> x_limit;
    std::pair<float, float> y_limit;

    OptixDeviceContext context = nullptr;
    OptixModule module = nullptr;
    OptixPipeline pipeline = nullptr;

    // Définition du programme de lancer de rayons (raygen)
    OptixProgramGroup raygen_program_group = nullptr;
    OptixProgramGroup miss_program_group = nullptr;
    OptixProgramGroup hit_program_group = nullptr;

    OptixModuleCompileOptions module_options = {};
    OptixPipelineCompileOptions pipeline_options = {};

    OptixTraversableHandle gas_handle;
    CUdeviceptr            d_gas_output_buffer;

    OptixShaderBindingTable sbt = {};

public:
	Optix(int width, int height);

    ~Optix();

    /* Charge un fichier .obj en retourne une liste de sommets et de triangles */
    void loadObj(
        const std::string& filename,
        float3*& vertices,
        uint3*& triangles,
        int& nb_v,
        int& nb_f
    );

    /* Initialise le contexte d'optix */
    int init();

    /* Charge le shader en mémoire */
    void loadShaders();

    /* Initialise le pipeline d'optix */
    void initPipeline(CUdeviceptr d_tris);

    /* Charge un .obj et l'envoie sur GPU */
    CUdeviceptr initScene();

    /* Rendu de l'image */
    void render();

    /* Simule le déplacement d'un DMD pour faire la sous-pixelisation */
	void DMDSimulation();

    // Deplacement du dmd (de la caméra) et renvoie à chaque lancer de rayon le résultat
    // avec du double buffering
    void DMDSimulationV2();

    /* Sauvegarde l'image en .bmp */
    void saveGrayscaleBitmapCuda(const std::string& filename, int width, int height, unsigned char* img, CUstream& stream);
};