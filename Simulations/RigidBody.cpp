#include "RigidBody.h"

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
	default:
		break;
	}
}
