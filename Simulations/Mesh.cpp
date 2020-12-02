#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>
#include "Mesh.h"
#include <iostream>

bool Mesh::load(const std::string& filename) {
	name = filename;
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	std::string err;
	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename.c_str(), 0, true);

	if(!ret) {
		printf("Unable to load model\n");
		return false;
	}

	// Loop over shapes
	for(size_t s = 0; s < shapes.size(); s++) {
		// Loop over faces(polygon)
		size_t index_offset = 0;

		for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
			int fv = shapes[s].mesh.num_face_vertices[f];
			// Loop over vertices in the face.
			for(size_t v = 0; v < fv; v++) {
				// access to vertex
				tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
				tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
				tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
				tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];
				tinyobj::real_t nx = attrib.normals[3 * idx.normal_index + 0];
				tinyobj::real_t ny = attrib.normals[3 * idx.normal_index + 1];
				tinyobj::real_t nz = attrib.normals[3 * idx.normal_index + 2];

				tinyobj::real_t tx, ty;

				// temporary fix
				if(!attrib.texcoords.empty()) {
					tx = attrib.texcoords[2 * idx.texcoord_index + 0];
					ty = attrib.texcoords[2 * idx.texcoord_index + 1];
				} else {
					tx = ty = 0;
				}
				vert_uvx.push_back(Vec4(vx, vy, vz, tx));
				normals_uvy.push_back(Vec4(nx, ny, nz, ty));
			}

			index_offset += fv;
		}
	}

	return true;
}

void Mesh::build_bvh() {
	const int num_tris = vert_uvx.size() / 3;
	std::vector<RadeonRays::bbox> bounds(num_tris);

#pragma omp parallel for
	for(int i = 0; i < num_tris; ++i) {
		const Vec3f v1 = Vec3f(vert_uvx[i * 3 + 0].x,
							 vert_uvx[i * 3 + 0].y,
							 vert_uvx[i * 3 + 0].z);
		const Vec3f v2 = Vec3f(vert_uvx[i * 3 + 1].x,
							 vert_uvx[i * 3 + 1].y,
							 vert_uvx[i * 3 + 1].z);
		const Vec3f v3 = Vec3f(vert_uvx[i * 3 + 2].x,
							 vert_uvx[i * 3 + 2].y,
							 vert_uvx[i * 3 + 2].z);
		bounds[i].grow(v1);
		bounds[i].grow(v2);
		bounds[i].grow(v3);
	}
	bvh->Build(&bounds[0], num_tris);
}
