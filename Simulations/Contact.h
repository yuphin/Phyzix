#pragma once
#include "RigidBody.h"
struct Contact {
	RigidBody* bodies[2] = {nullptr, nullptr};
	Vec3 cp_rel[2];
	int cp_idx;
	Vec3 normal;
	Vec3 collision_point;
	float relative_vel;
	float expected_vel = 0;
	float penetration;
	bool is_valid;
};

struct CollisionData {
	CollisionData() {
		contacts.resize(16);
		num_contacts = 0;
	}
	void reset() {
		contacts.resize(16);
		num_contacts = 0;
	}
	std::vector<Contact> contacts;
	int num_contacts;
};

Contact* collision_box_plane(RigidBody* rb1, RigidBody* plane, Mat4& trs_a,
	CollisionData& c_data);