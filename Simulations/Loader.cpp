/*
	This is a modified version of the original code
	Link to original code: https://github.com/mmacklin/tinsel
*/

#include "Loader.h"
#include <tiny_obj_loader.h>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <stdio.h>
#include "Options.h"

static const int k_max_line_len = 2048;
int(*Log)(const char* szFormat, ...) = printf;

bool load_scene(const std::string& filename, Scene* scene, const Options& render_options) {
	FILE* file;
	file = fopen(filename.c_str(), "r");

	if(!file) {
		Log("Couldn't open %s for reading\n", filename.c_str());
		return false;
	}

	Log("Loading Scene..\n");

	struct MaterialData {
		Material mat;
		int id;
	};

	std::map<std::string, MaterialData> material_map;
	std::vector<std::string> albedo_tex;
	std::vector<std::string> metallic_roughness_tex;
	std::vector<std::string> normal_tex;
	std::string path = filename.substr(0, filename.find_last_of("/\\")) + "/";

	int material_cnt = 0;
	char line[k_max_line_len];

	//Defaults
	Material default_mat;
	scene->add_material(default_mat);

	bool cameraAdded = false;

	while(fgets(line, k_max_line_len, file)) {
		// skip comments
		if(line[0] == '#')
			continue;

		// name used for materials and meshes
		char name[k_max_line_len] = { 0 };

		//--------------------------------------------
		// Material

		if(sscanf(line, " material %s", name) == 1) {
			Material material;
			char albedo_tex_name[100] = "None";
			char metallic_rougness_tex_name[100] = "None";
			char normal_tex_name[100] = "None";

			while(fgets(line, k_max_line_len, file)) {
				// end group
				if(strchr(line, '}'))
					break;

				sscanf(line, " name %s", name);
				sscanf(line, " color %f %f %f", &material.albedo.x, &material.albedo.y, &material.albedo.z);
				sscanf(line, " emission %f %f %f", &material.emission.x, &material.emission.y, &material.emission.z);
				sscanf(line, " materialType %f", &material.mat_type);
				sscanf(line, " metallic %f", &material.metallic);
				sscanf(line, " roughness %f", &material.roughness);
				sscanf(line, " ior %f", &material.ior);
				//sscanf(line, " transmittance %f", &material.transmittance);

				sscanf(line, " albedoTexture %s", albedo_tex_name);
				sscanf(line, " metallicRoughnessTexture %s", metallic_rougness_tex_name);
				sscanf(line, " normalTexture %s", normal_tex_name);
			}

			//// Albedo Texture
			//if(strcmp(albedo_tex_name, "None") != 0)
			//	material.albedo_texid = scene->AddTexture(path + albedo_tex_name);

			//// MetallicRoughness Texture
			//if(strcmp(metallic_rougness_tex_name, "None") != 0)
			//	material.metallicRoughnessTexID = scene->AddTexture(path + metallic_rougness_tex_name);

			//// Normal Map Texture
			//if(strcmp(normal_tex_name, "None") != 0)
			//	material.normalmapTexID = scene->AddTexture(path + normal_tex_name);

			// add material to map
			if(material_map.find(name) == material_map.end()) // New material
			{
				int id = scene->add_material(material);
				material_map[name] = MaterialData{ material, id };
			}
		}

		//--------------------------------------------
		// Light

		if(strstr(line, "light")) {
			Light light;
			Vec3f v1, v2;
			char light_type[20] = "None";

			while(fgets(line, k_max_line_len, file)) {
				// end group
				if(strchr(line, '}'))
					break;

				sscanf(line, " position %f %f %f", &light.position.x, &light.position.y, &light.position.z);
				sscanf(line, " emission %f %f %f", &light.emission.x, &light.emission.y, &light.emission.z);

				sscanf(line, " radius %f", &light.radius);
				sscanf(line, " v1 %f %f %f", &v1.x, &v1.y, &v1.z);
				sscanf(line, " v2 %f %f %f", &v2.x, &v2.y, &v2.z);
				sscanf(line, " type %s", light_type);
			}

			if(strcmp(light_type, "Quad") == 0) {
				light.type = LightType::TYPE_QUAD;
				light.u = v1 - light.position;
				light.v = v2 - light.position;
				light.area = norm(cross(light.u, light.v));
			} else if(strcmp(light_type, "Sphere") == 0) {
				light.type = LightType::TYPE_SPHERE;
				light.area = 4.0f * M_PI * light.radius * light.radius;
			}
			scene->add_light(light);
		}

		//--------------------------------------------
		// Camera
		// TODO: Finish camera
		if(strstr(line, "Camera")) {
			Vec3f position;
			Vec3f lookat;
			float fov;
			float aperture = 0, focal_dist = 1;

			while(fgets(line, k_max_line_len, file)) {
				// end group
				if(strchr(line, '}'))
					break;

				sscanf(line, " position %f %f %f", &position.x, &position.y, &position.z);
				sscanf(line, " lookAt %f %f %f", &lookat.x, &lookat.y, &lookat.z);
				sscanf(line, " aperture %f ", &aperture);
				sscanf(line, " focaldist %f", &focal_dist);
				sscanf(line, " fov %f", &fov);
			}

			scene->camera.reset();
			scene->add_camera(position, lookat, fov);
			//scene->camera->aperture = aperture;
			//scene->camera->focalDist = focal_dist;
			cameraAdded = true;
		}

		//--------------------------------------------
		// Renderer

		if(strstr(line, "Renderer")) {
			char envMap[200] = "None";

			while(fgets(line, k_max_line_len, file)) {
				// end group
				if(strchr(line, '}'))
					break;

				sscanf(line, " envMap %s", envMap);
				sscanf(line, " resolution %d %d", &render_options.resolution.x, &render_options.resolution.y);
				//sscanf(line, " hdrMultiplier %f", &render_options.hdrMultiplier);
				sscanf(line, " maxDepth %i", &render_options.max_depth);
				sscanf(line, " tileWidth %i", &render_options.tile_width);
				sscanf(line, " tileHeight %i", &render_options.tile_height);
			}

		/*	if(strcmp(envMap, "None") != 0) {
				scene->AddHDR(path + envMap);
				render_options.useEnvMap = true;
			}*/
		}


		//--------------------------------------------
		// Mesh

		if(strstr(line, "mesh")) {
			std::string filename;
			Vec3f pos;
			Vec3f scale;
			Mat4f transform;
			int material_id = 0; // Default Material ID
			char mesh_name[200] = "None";

			while(fgets(line, k_max_line_len, file)) {
				// end group
				if(strchr(line, '}'))
					break;

				char file[2048];
				char mat_name[100];

				sscanf(line, " name %[^\t\n]s", mesh_name);

				if(sscanf(line, " file %s", file) == 1) {
					filename = path + file;
				}

				if(sscanf(line, " material %s", mat_name) == 1) {
					// look up material in dictionary
					if(material_map.find(mat_name) != material_map.end()) {
						material_id = material_map[mat_name].id;
					} else {
						Log("Could not find material %s\n", mat_name);
					}
				}

				sscanf(line, " position %f %f %f", 
					   &transform.value[3][0], 
					   &transform.value[3][1], 
					   &transform.value[3][2]);
				sscanf(line, " scale %f %f %f", 
					   &transform.value[0][0], 
					   &transform.value[1][1], 
					   &transform.value[2][2]);
			}
			if(!filename.empty()) {
				int mesh_id = scene->add_mesh(filename);
				if(mesh_id != -1) {
					std::string instance_name;

					if(strcmp(mesh_name, "None") != 0) {
						instance_name = std::string(mesh_name);
					} else {
						std::size_t pos = filename.find_last_of("/\\");
						instance_name = filename.substr(pos + 1);
					}

					MeshInstance instance1(instance_name, mesh_id, transform, material_id);
					scene->add_mesh_instance(instance1);
				}
			}
		}
	}

	fclose(file);

	if(!cameraAdded)
		scene->add_camera(Vec3f(0.0f, 0.0f, 10.0f), Vec3f(0.0f, 0.0f, -10.0f), 35.0f);

	scene->create_acceleration_structures();

	return true;
}
