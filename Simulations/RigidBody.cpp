#include "RigidBody.h"

Mat4 RigidBody::obj_to_world()
{
	auto scale = Mat4();
	auto rot = orientation.getRotMat();
	auto trans = Mat4();
	scale.value[0][0] = size.x;
	scale.value[1][1] = size.y;
	scale.value[2][2] = size.z;
	scale.value[3][3] = 1;
	trans.initTranslation(position[0], position[1], position[2]);
	return scale * rot * trans;
}

Mat4 RigidBody::obj_to_world_plane_rendering() {
	auto scale = Mat4();
	auto trans = Mat4();
	scale.value[0][0] = 100;
	scale.value[1][1] = 0.001;
	scale.value[2][2] = 100;
	scale.value[3][3] = 1;

	Vec3 d = Vec3(0, 1, 0);
	Vec3 a = cross(d, normal);
	Quat q = { a.x, a.y, a.z, sqrt(1) + dot(d, normal) };
	q = q.unit();

	auto rot = q.getRotMat();
	trans.initTranslation(normal.x * offset, normal.y * offset, normal.z * offset);
	return scale * rot * trans;
}

Mat4 RigidBody::get_transformed_inertia(const Mat4& inertia) {
	auto rot = orientation.getRotMat();
	auto rot_transpose = orientation.getRotMat();
	rot_transpose.transpose();
	return rot * inertia * rot_transpose;
}

void RigidBody::calc_inv_inertia_tensor() {
	switch (type) {
	case RigidBodyType::CUBOID:
	{
		constexpr float c = 1.0f / 12.0f;
		const float size_y_2 = size.y * size.y;
		const float size_x_2 = size.x * size.x;
		const float size_z_2 = size.z * size.z;
		inv_inertia_0.value[0][0] = c * mass * (size_y_2 + size_z_2);
		inv_inertia_0.value[1][1] = c * mass * (size_x_2 + size_z_2);
		inv_inertia_0.value[2][2] = c * mass * (size_x_2 + size_y_2);
		inv_inertia_0.value[3][3] = 1;
		inv_inertia_0 = inv_inertia_0.inverse();
		break;
	}
	case RigidBodyType::SPHERE:
	{
		constexpr float c = 2.0f / 5.0f;
		const float size_x_2 = c * mass * offset * offset;
		inv_inertia_0.value[0][0] = size_x_2;
		inv_inertia_0.value[1][1] = size_x_2;
		inv_inertia_0.value[2][2] = size_x_2;
		inv_inertia_0.value[3][3] = 1;
		inv_inertia_0 = inv_inertia_0.inverse();
		break;
	}
	default:
		break;
	}
}
