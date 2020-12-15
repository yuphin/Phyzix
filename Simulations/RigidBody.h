#pragma once
#include "util/vectorbase.h"
#include "DrawingUtilitiesClass.h"

static constexpr int MAX_CONTACT_POINT = 16;
enum class RigidBodyType {
	CUBOID = 1 << 0,
	SPHERE = 1 << 1,
	PLANE = 1 << 2
};
class RigidBody {
public:
	// Subjected to change!
	RigidBody(float offset, const Vec3& normal,
		RigidBodyType type = RigidBodyType::PLANE) {
		this->offset = offset;
		this->normal = normal;
		this->inv_mass = 0;
		this->inv_inertia_0;
		this->type = type;
		movable = false;
	}
	RigidBody(float radius, const Vec3& pos, int mass, 
		RigidBodyType type = RigidBodyType::SPHERE) {
		this->offset = radius;
		this->inv_mass = 1.0f / mass;
		this->mass = mass;
		this->position = pos;
		this->type = type;
		calc_inv_inertia_tensor();
	}
	RigidBody(const Vec3& position, const Vec3& size, int mass,
		RigidBodyType type = RigidBodyType::CUBOID) {
		this->position = position;
		this->size = size;
		this->mass = mass;
		this->type = type;
		inv_mass = 1.0f / mass;
		calc_inv_inertia_tensor();
	}
	Mat4 obj_to_world();
	Mat4 obj_to_world_plane_rendering();
	Mat4 get_transformed_inertia(const Mat4& inertia);

	Vec3 position;
	Vec3 size;
	Vec3 linear_velocity;
	Vec3 angular_vel;
	Vec3 angular_momentum;
	Vec3 torque = Vec3();
	Vec3 force = Vec3();
	Vec3 normal;
	int mass;
	double inv_mass;
	float offset;
	bool movable = true;
	Mat4 inv_inertia_0;
	Quat orientation = { 0,0,0,1 };
	RigidBodyType type;
private:
	void calc_inv_inertia_tensor();
};

class Plane {
public:
	Vec3 normal;
	float offset;
	Quat orientation = { 0,0,0,1 };
};


