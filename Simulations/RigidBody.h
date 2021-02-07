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
	Mat4 obj_to_world();
	Mat4 obj_to_world_plane_rendering();
	Mat4 get_transformed_inertia(const Mat4& inertia);
	void make_plane(float offset, const Vec3& normal);
	void make_sphere(float radius, const Vec3& pos, int mass);
	void make_box(const Vec3& position, const Vec3& size, int mass);
	void apply_force(const Vec3& loc, const Vec3& force);
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
	std::vector<Vec3> samples;
private:
	void calc_inv_inertia_tensor();
};

class Plane {
public:
	Vec3 normal;
	float offset;
	Quat orientation = { 0,0,0,1 };
};


