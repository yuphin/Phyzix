#pragma once
#include <vector>
#include "DrawingUtilitiesClass.h"

enum class MaterialType {
	MICROFACET,
	SMOOTH_GLASS,
	DIFFUSE,
	MIRROR,
	DIELECTRIC
};

class Material {
public:
	Material() {
		albedo = Vec3f(1.0f, 1.0f, 1.0f);
		emission = Vec3f(0.0f, 0.0f, 0.0f);
		mat_type = float(MaterialType::MICROFACET);
		unused = 0;
		metallic = 0.0f;
		roughness = 0.5f;
		ior = 1.45f;
		albedo_texid = -1.0f;
		metallic_rougness_texid = -1.0f;
		normalmap_texid = -1.0f;
	};
	Vec3f albedo;
	float mat_type;
	Vec3f emission;
	float unused;
	float metallic;
	float roughness;
	float ior;
	float albedo_texid;
	float metallic_rougness_texid;
	float normalmap_texid;
	float padding[2];
};