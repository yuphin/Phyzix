#include "Contact.h"
#include "util/util.h"

Contact* collision_box_plane(RigidBody* rb1, RigidBody* plane, Mat4& trs_a,
	CollisionData& c_data) {
	Contact* res = nullptr;
	/*static Vec3 vtxs[8] = {
		{1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},
		{1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1}
	};*/
	static Vec3 vtxs[8] = {
	{0.5,0.5,0.5},{-0.5,0.5,0.5},{0.5,-0.5,0.5},{-0.5,-0.5,0.5},
	{0.5,0.5,-0.5},{-0.5,0.5,-0.5},{0.5,-0.5,-0.5},{-0.5,-0.5,-0.5}
	};
	float max_pen = -10000.0f;
	static constexpr float EPS = 0.01f;
	for (int i = 0; i < 8; i++) {
		auto vtx_pos = vtxs[i];
		vtx_pos =  vtx_pos * trs_a;
		auto vtx_dist = dot(vtx_pos, plane->normal);
		if (vtx_dist  <= plane->offset) {
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
