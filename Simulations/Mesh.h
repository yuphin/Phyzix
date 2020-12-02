#pragma once
#include <bvh.h>
#include "DrawingUtilitiesClass.h"
#include "util/vector4d.h"
class Mesh {
public:
    Mesh() {
        //bvh = new RadeonRays::SplitBvh(2.0f, 64, 0, 0.001f, 2.5f); 
        bvh = std::make_unique<RadeonRays::Bvh>(2.0f, 64, false);
    }
    void build_bvh();
    bool load(const std::string& filename);

    std::vector<Vec4> vert_uvx; // Vertex Data + x coord of uv 
    std::vector<Vec4> normals_uvy;  // Normal Data + y coord of uv

    std::unique_ptr<RadeonRays::Bvh> bvh = nullptr;
    std::string name;
};

class MeshInstance {

public:
    MeshInstance(const std::string& name, int mesh_id, matrix4x4<float>& transform, int mat_id)
        : name(name)
        , mesh_id(mesh_id)
        , transform(transform)
        , material_id(mat_id) {}
    ~MeshInstance() {}

    matrix4x4<float> transform;
    std::string name;

    int material_id;
    int mesh_id;
};

