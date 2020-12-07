#include "Scene.h"

void Scene::add_camera(Vec3f pos, Vec3f lookat, float fov) {
	camera = std::make_unique<Camera>(pos, lookat, fov);
}

int Scene::add_mesh(const std::string& filename) {
	int id = -1;
	// Check if mesh was already loaded
	for(int i = 0; i < meshes.size(); i++)
		if(meshes[i]->name == filename)
			return i;

	id = meshes.size();
	Mesh* mesh = new Mesh;
	if(mesh->load(filename)) {
		meshes.push_back(mesh);
		printf("Model %s loaded\n", filename.c_str());
	} else
		id = -1;
	return id;
}


int Scene::add_material(const Material& material) {
	int id = materials.size();
	materials.push_back(material);
	return id;
}


int Scene::add_mesh_instance(const MeshInstance& meshInstance) {
	int id = mesh_instances.size();
	mesh_instances.push_back(meshInstance);
	return id;
}

int Scene::add_light(const Light& light) {
	int id = lights.size();
	lights.push_back(light);
	return id;
}

void Scene::create_tlas() {
	// Loop through all the mesh Instances and build a Top Level BVH
	std::vector<RadeonRays::bbox> bounds;
	bounds.resize(mesh_instances.size());

#pragma omp parallel for
	for(int i = 0; i < mesh_instances.size(); i++) {
		RadeonRays::bbox bbox = meshes[mesh_instances[i].mesh_id]->bvh->Bounds();
		Mat4f matrix = mesh_instances[i].transform;

		Vec3f minBound = bbox.pmin;
		Vec3f maxBound = bbox.pmax;

		Vec3f right = Vec3f(matrix.value[0][0], matrix.value[0][1], matrix.value[0][2]);
		Vec3f up = Vec3f(matrix.value[1][0], matrix.value[1][1], matrix.value[1][2]);
		Vec3f forward = Vec3f(matrix.value[2][0], matrix.value[2][1], matrix.value[2][2]);
		Vec3f translation = Vec3f(matrix.value[3][0], matrix.value[3][1], matrix.value[3][2]);

		Vec3f xa = right * minBound.x;
		Vec3f xb = right * maxBound.x;

		Vec3f ya = up * minBound.y;
		Vec3f yb = up * maxBound.y;

		Vec3f za = forward * minBound.z;
		Vec3f zb = forward * maxBound.z;

		minBound = Vec3f_min(xa, xb) + Vec3f_min(ya, yb) + Vec3f_min(za, zb) + translation;
		maxBound = Vec3f_max(xa, xb) + Vec3f_max(ya, yb) + Vec3f_max(za, zb) + translation;

		RadeonRays::bbox bound;
		bound.pmin = minBound;
		bound.pmax = maxBound;

		bounds[i] = bound;
	}
	scene_bvh->Build(&bounds[0], bounds.size());
	sceneBounds = scene_bvh->Bounds();
}

void Scene::create_blas() {
	// Loop through all meshes and build BVHs
#pragma omp parallel for
	for(int i = 0; i < meshes.size(); i++) {
		printf("Building BVH for %s\n", meshes[i]->name.c_str());
		meshes[i]->build_bvh();
	}
}

void Scene::rebuild() {
	scene_bvh.reset();
	scene_bvh = std::make_unique<RadeonRays::Bvh>(10.0f, 64, false);

	create_tlas();
	bvhTranslator.UpdateTLAS(scene_bvh.get(), mesh_instances);

	//Copy transforms
	for(int i = 0; i < mesh_instances.size(); i++)
		transforms[i] = mesh_instances[i].transform;

	instancesModified = true;
}

void Scene::create_acceleration_structures() {
	create_blas();

	printf("Building scene BVH\n");
	create_tlas();

	// Flatten BVH
	bvhTranslator.Process(scene_bvh.get(), meshes, mesh_instances);

	int verticesCnt = 0;

	//Copy mesh data
	for(int i = 0; i < meshes.size(); i++) {
		// Copy indices from BVH and not from Mesh
		int num_indices = meshes[i]->bvh->GetNumIndices();
		const int* tri_indices = meshes[i]->bvh->GetIndices();

		for(int j = 0; j < num_indices; j++) {
			int index = tri_indices[j];
			int v1 = (index * 3 + 0) + verticesCnt;
			int v2 = (index * 3 + 1) + verticesCnt;
			int v3 = (index * 3 + 2) + verticesCnt;

			vert_indices.push_back(Indices{ v1, v2, v3 });
		}

		vertices_uvx.insert(vertices_uvx.end(), meshes[i]->vert_uvx.begin(), meshes[i]->vert_uvx.end());
		normals_uvy.insert(normals_uvy.end(), meshes[i]->normals_uvy.begin(), meshes[i]->normals_uvy.end());

		verticesCnt += meshes[i]->vert_uvx.size();
	}

	// Resize to power of 2
	indices_tex_width = (int) (sqrt(vert_indices.size()) + 1);
	vert_tex_width = (int) (sqrt(vertices_uvx.size()) + 1);

	vert_indices.resize(indices_tex_width * indices_tex_width);
	vertices_uvx.resize(vert_tex_width * vert_tex_width);
	normals_uvy.resize(vert_tex_width * vert_tex_width);

	for(int i = 0; i < vert_indices.size(); i++) {
		vert_indices[i].x = ((vert_indices[i].x % vert_tex_width) << 12) | (vert_indices[i].x / vert_tex_width);
		vert_indices[i].y = ((vert_indices[i].y % vert_tex_width) << 12) | (vert_indices[i].y / vert_tex_width);
		vert_indices[i].z = ((vert_indices[i].z % vert_tex_width) << 12) | (vert_indices[i].z / vert_tex_width);
	}

	//Copy transforms
	transforms.resize(mesh_instances.size());
#pragma omp parallel for
	for(int i = 0; i < mesh_instances.size(); i++)
		transforms[i] = mesh_instances[i].transform;

	////Copy Textures
	//for(int i = 0; i < textures.size(); i++) {
	//	texWidth = textures[i]->width;
	//	texHeight = textures[i]->height;
	//	int texSize = texWidth * texHeight;
	//	textureMapsArray.insert(textureMapsArray.end(), &textures[i]->texData[0], &textures[i]->texData[texSize * 3]);
	//}
}
