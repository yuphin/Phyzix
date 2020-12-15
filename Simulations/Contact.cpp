#include "Contact.h"
#include "util/util.h"

Contact* collision_box_plane(RigidBody* rb1, RigidBody* plane, Mat4& trs_a,
	CollisionData& c_data) {
	Contact* res = nullptr;
	static Vec3 vtxs[8] = {
	{0.5,0.5,0.5},{-0.5,0.5,0.5},{0.5,-0.5,0.5},{-0.5,-0.5,0.5},
	{0.5,0.5,-0.5},{-0.5,0.5,-0.5},{0.5,-0.5,-0.5},{-0.5,-0.5,-0.5}
	};
	float max_pen = -10000.0f;
	static constexpr float EPS = 0.01f;
	for (int i = 0; i < 8; i++) {
		auto vtx_pos = vtxs[i];
		vtx_pos = vtx_pos * trs_a;
		auto vtx_dist = dot(vtx_pos, plane->normal);
		if (vtx_dist <= plane->offset) {
			//printf("VTX DIST %f\n", vtx_dist);
		}
		if (vtx_dist <= plane->offset) {
			Contact& contact = c_data.contacts[c_data.num_contacts++];
			Vec3 collision_point = plane->normal;
			collision_point *= (vtx_dist - plane->offset);
			collision_point += vtx_pos;
			contact.collision_point = collision_point;
			contact.normal = plane->normal;
			contact.bodies[0] = rb1;
			contact.cp_rel[0] = collision_point - rb1->position;
			contact.penetration = plane->offset - vtx_dist;
			contact.is_valid = true;
			contact.cp_idx = i;
			if (contact.penetration > max_pen) {
				res = &contact;

			}
		}
	}
	return res;
}

Contact* collision_sphere_plane(RigidBody* sphere, RigidBody* plane, Mat4& trs_sphere, CollisionData& c_data) {
	// Sphere->offset == radius
	Contact* res = nullptr;
	float dist = dot(plane->normal, sphere->position) - sphere->offset - plane->offset;
	if (dist <= 0) {
		res = &c_data.contacts[c_data.num_contacts++];
		res->normal = plane->normal;
		res->penetration = -dist;
		res->collision_point = sphere->position - plane->normal * (dist + sphere->offset);
		res->is_valid = true;
		res->bodies[0] = sphere;
		res->cp_rel[0] = res->collision_point - sphere->position;

	}
	return res;
}

Contact* collision_sphere_sphere(RigidBody* s1, RigidBody* s2, Mat4& trs_s1, CollisionData& c_data) {
	Contact* res = nullptr;
	Vec3 r_to_r = s1->position - s2->position;
	auto size = norm(r_to_r);
	if (size <= 0.0f || size >= s1->offset + s2->offset) {
		return nullptr;
	}

	Vec3 normal = r_to_r * 1.0 / size;
	res = &c_data.contacts[c_data.num_contacts++];
	// TODO
	res->collision_point = s1->position + r_to_r * 0.5;
	res->normal = normal;
	res->penetration = s1->offset + s2->offset - size;
	res->is_valid = true;
	res->bodies[0] = s1;
	res->bodies[1] = s2;
	res->cp_rel[0] = res->collision_point - s1->position;
	res->cp_rel[1] = res->collision_point - s2->position;
	return res;


}

static Vec3 fabs_vec(const Vec3& vec) {
	return Vec3(fabs(vec.x), fabs(vec.y), fabs(vec.z));
}

Contact* collision_box_sphere(RigidBody* b, RigidBody* s, Mat4& trs_b, CollisionData& c_data) {
	// Reminder that s->offset is radius
	Contact* res = nullptr;
	Vec3 box_local_pos = { 0,0,0 };
	Vec3 sphere_pos_according_to_box_coord = s->position * trs_b.inverse();
	Vec3 scaled_rad = Vec3(s->offset / b->size.x, s->offset / b->size.y, s->offset / b->size.z);
	Vec3 dif = fabs_vec(sphere_pos_according_to_box_coord) - scaled_rad;
	if (dif.x > 0.5 || dif.y > 0.5 || dif.z > 0.5) {
		return nullptr;
	}
	Vec3 closest = Vec3();
	float dist = sphere_pos_according_to_box_coord.x;
	if (dist > 0.5) dist = 0.5;
	if (dist < -0.5) dist = -0.5;
	closest.x = dist;

	dist = sphere_pos_according_to_box_coord.y;
	if (dist > 0.5) dist = 0.5;
	if (dist < -0.5) dist = -0.5;
	closest.y = dist;

	dist = sphere_pos_according_to_box_coord.z;
	if (dist > 0.5) dist = 0.5;
	if (dist < -0.5) dist = -0.5;
	closest.z = dist;

	auto tmp = closest - sphere_pos_according_to_box_coord;
	dist = dot(tmp, tmp);
	if (dist > dot(scaled_rad, scaled_rad)) return nullptr;

	Vec3 point_world = closest * trs_b;
	res = &c_data.contacts[c_data.num_contacts++];

	res->is_valid = true;
	res->normal = point_world - s->position;
	normalize(res->normal);
	res->collision_point = point_world;
	res->penetration = s->offset - sqrt(dist);
	res->bodies[0] = b;
	res->bodies[1] = s;
	res->cp_rel[0] = res->collision_point - b->position;
	res->cp_rel[1] = res->collision_point - s->position;

	return res;
}
