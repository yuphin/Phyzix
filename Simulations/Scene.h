#pragma once
#include <string>
#include <vector>
#include <map>
#include <bvh.h>
#include <bvh_translator.h>
#include "DrawingUtilitiesClass.h"
#include "Camera.h"
#include "Material.h"
#include "Mesh.h"
#include "util/vector4d.h"
// Note : Scene system adapted from https://github.com/knightcrawler25/GLSL-PathTracer
// Including meshes, loading etc.
//using Vec3f = vector3Dim<Real>;
class Camera;
enum LightType {
	TYPE_SPHERE,
    TYPE_QUAD
};
struct Light {
   
	Vec3f position;
    Vec3f emission;
    Vec3f u;
    Vec3f v;
	float radius;
	float area;
	float type;
};
struct Indices {
	int x, y, z;
};
class Scene {
public:
    Scene() : camera(nullptr) {
        scene_bvh = std::make_unique<RadeonRays::Bvh>(10.0f, 64, false);
    }
    int add_mesh(const std::string& filename);
    int add_material(const Material& material);
    int add_mesh_instance(const MeshInstance& meshInstance);
    int add_light(const Light& light);

    void add_camera(Vec3f eye, Vec3f lookat, float fov);

    void create_acceleration_structures();
    void rebuild();

    //Options
    //RenderOptions renderOptions;

    //Meshs
    std::vector<Mesh*> meshes;
    matrix4x4<float> asda;
    // Scene Mesh Data 
    std::vector<Indices>   vert_indices;
    std::vector<Vec4> vertices_uvx; // Vertex Data + x coord of uv 
    std::vector<Vec4> normals_uvy;  // Normal Data + y coord of uv
    std::vector<matrix4x4<float>> transforms;
    int indices_tex_width;
    int vert_tex_width;

    //Instances
    std::vector<Material> materials;
    std::vector<MeshInstance> mesh_instances;
    bool instancesModified = false;

    //Lights
    std::vector<Light> lights;

    //Camera
    std::unique_ptr<Camera> camera = nullptr;

    //Bvh
    RadeonRays::BvhTranslator bvhTranslator; // Produces a flat bvh array for GPU consumption
    RadeonRays::bbox sceneBounds;

    //Texture Data
    //std::vector<Texture*> textures;
    //std::vector<unsigned char> textureMapsArray;
    int texWidth;
    int texHeight; // TODO: allow textures of different sizes

private:
    std::unique_ptr<RadeonRays::Bvh> scene_bvh = nullptr;
    void create_blas();
    void create_tlas();
};
