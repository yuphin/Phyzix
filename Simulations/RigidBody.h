#pragma once
#include "util/vectorbase.h"
#include "DrawingUtilitiesClass.h"

enum class RigidBodyType {
	CUBOID
};
class RigidBody {
public:
	// Subjected to change!
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
	Mat4 get_transformed_inertia(const Mat4& inertia);

	Vec3 position;
	Vec3 size;
	Vec3 linear_velocity;
	Vec3 angular_vel;
	Vec3 angular_momentum;
	Vec3 torque;
	Vec3 force;
	int mass;
	double inv_mass;
	Mat4 inv_inertia_0;
	Quat orientation = {0,0,0,1};
	RigidBodyType type;
	bool is_kinematic = false;
private:
	void calc_inv_inertia_tensor();
};
