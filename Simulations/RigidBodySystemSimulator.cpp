#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iTestCase = 0;

}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1, Demo 2, Demo 3, Demo 4, Demo 5, Demo 6, Demo 7, Demo 8, Demo 9";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwAddVarCB(DUC->g_pTweakBar, "Gravity", TW_TYPE_DIR3F, setGravity, getGravity, &gravity, "");
	TwAddButton(DUC->g_pTweakBar, "Create Random Box", addBox, this, "");
}

void TW_CALL RigidBodySystemSimulator::addBox(void* value) {
	static std::mt19937 eng;
	static std::uniform_real_distribution<float> randomer(-1.5, 2);
	((RigidBodySystemSimulator*) value)->addRigidBody({ randomer(eng), 1.0, randomer(eng) }, { 0.3, 0.3, 0.3 }, 2);
}

void TW_CALL RigidBodySystemSimulator::getGravity(void* value, void* clientData) {
	static_cast<float*> (value)[0] = static_cast<const Vec3*>(clientData)->x;
	static_cast<float*> (value)[1] = static_cast<const Vec3*>(clientData)->y;
	static_cast<float*> (value)[2] = static_cast<const Vec3*>(clientData)->z;
}

void TW_CALL RigidBodySystemSimulator::setGravity(const void* value, void* clientData) {
	static_cast<Vec3*> (clientData)->x = static_cast<const float*> (value)[0];
	static_cast<Vec3*> (clientData)->y = static_cast<const float*> (value)[1];
	static_cast<Vec3*> (clientData)->z = static_cast<const float*> (value)[2];
}

void RigidBodySystemSimulator::reset() {}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* context) {
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
	for (int i = 0; i < rigid_bodies.size(); i++) {
		switch (rigid_bodies[i].type) {
		case RigidBodyType::CUBOID:
		{
			DUC->drawRigidBody(rigid_bodies[i].obj_to_world().toDirectXMatrix());

		}
		break;
		case RigidBodyType::SPHERE:
		{
			if (m_iTestCase == 4) {
				int a = 4;
			}
			const auto& rad = rigid_bodies[i].offset;
			DUC->drawSphere(rigid_bodies[i].position, {rad,rad,rad});

		}
		break;
		default:
			break;
		}
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	rigid_bodies.clear();
	running = true;
	gravity = 0;
	mouse_force = 0;


	switch (testCase) {
	case 0:
		*timestep = 2.0f;
		addRigidBody({ 0, 0, 0 }, { 1, 0.6, 0.5 }, 2);
		applyForceOnBody(0, { 0.3, 0.5, 0.25 }, { 1, 1,0 });
		break;
	case 1:
		*timestep = 0.01f;
		addRigidBody({ 0, 0, 0 }, { 1, 1, 1 }, 2);
		break;
	case 2:
		*timestep = 0.001f;
		addRigidBody({ 0.25, -0.5, 0 }, { 1, 0.5, 0.5 }, 2);
		addRigidBody({ -0.25, 1, 0 }, { 1, 0.5, 0.5 }, 2);
		applyForceOnBody(0, { 0.25, -0.5, 0 }, { 0, 10,0 });
		applyForceOnBody(1, { -0.25, 1, 0 }, { 0, -35,0 });
		break;
	case 3:
		*timestep = 0.001f;
		bounciness = 0.3;
		gravity = Vec3(0, -9.81f, 0);
		break;
	case 4:

		gravity = Vec3(0, -9.81f, 0);
		*timestep = 0.001f;
		/*	addRigidBody({ 0, 1, 0 }, { 1, 1, 1 }, 10);
		addRigidBody({ 2, 1, 0 }, { 1, 1, 1 }, 10);
		applyForceOnBody(0, { 0, 1.0, 0.0 }, { 100000,0,0 });

		applyForceOnBody(1, { 2, 1, 0.0 }, { -100000,0,0 });*/
		addRigidBody({ 0.5, 0, 0 }, { 1, 1, 1 }, 10);
		addRigidBody({ 0, 2, 0 }, { 1, 1, 1 }, 10);
		applyForceOnBody(1, { 0.0, 0.0, 0.0 }, { 0,-500,0 });
		break;
	case 5:
		*timestep = 0.001f;
		gravity = Vec3(0, -9.81f, 0);
		addRigidBody({ 0, 0, 0 }, { 1, 1, 1 }, 10);
		//applyForceOnBody(0, { 0.0, 0.5, 0.5 }, { 1,1,0 });
		setOrientationOf(0, Quat(Vec3(0.5f, 0.8f, 1.0f), (float)(M_PI) * 0.5f));
		add_torque(0, { 5000, 5000, 5000 });
		break;
	case 6:
		*timestep = 0.001f;
		gravity = Vec3(0, -9.81f, 0);
		add_sphere({ 0,0,0 }, 0.5, 10);
		break;
	case 7:
		*timestep = 0.001f;
		gravity = Vec3(0, -9.81f, 0);
		addRigidBody({ 0.5, 0, 0 }, { 1, 1, 1 }, 10);
		break;
	case 8:
		*timestep = 0.001f;
		/*	add_sphere({ 0.3,0,0 }, 0.5, 10);
		add_sphere({ 0,2,0 }, 0.5, 10);*/
		gravity = Vec3(0, -9.81f, 0);
		addRigidBody({ 0.0,0,0 }, { 1,1,1 }, 10);
		add_sphere({ 0.3,2,0 }, 0.5, 10);
		add_sphere({ 0.45,3,0 }, 0.5, 10);
		break;
	default:
		break;
	}
	// Add plane
	rigid_bodies.push_back({ -1,{0,1,0} });
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
	Point2D mouseDiff;
	mouseDiff.x = trackmouse.x - old_trackmouse.x;
	mouseDiff.y = trackmouse.y - old_trackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0) {
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.0001f;
		inputWorld = inputWorld * inputScale;
		mouse_force = inputWorld;
	}
	else {
		mouse_force = {};
	}
	for (auto& rb : rigid_bodies) {
		rb.force += mouse_force + gravity * rb.mass;
	}
}

void RigidBodySystemSimulator::handle_collisions() {
	if (rigid_bodies.size() < 2) {
		return;
	}
	bool resolve = false;
	Contact* ci = nullptr;
	CollisionData data;
	for (int i = 0; i < rigid_bodies.size() - 1; i++) {
		RigidBody& b1 = rigid_bodies[i];

		for (int j = i + 1; j < rigid_bodies.size(); j++) {
			RigidBody& b2 = rigid_bodies[j];
			// TODO: Vector is overkill here, fix
			std::vector<RigidBody*> pairs = { &b1,&b2 };
			std::sort(pairs.begin(), pairs.end(), [](RigidBody* a, RigidBody* b) {
				return a->type < b->type;
			});
			if (pairs[0]->type == RigidBodyType::CUBOID && pairs[1]->type == RigidBodyType::CUBOID) {

				ci = checkCollisionSAT(pairs[0]->obj_to_world(), pairs[1]->obj_to_world(), &data);
				resolve = ci && ci->is_valid;
				if (ci->is_valid) {
					ci->bodies[0] = &b1;
					ci->bodies[1] = &b2;
					ci->cp_rel[0] = ci->collision_point - b1.position;
					ci->cp_rel[1] = ci->collision_point - b2.position;
				}
			}
			else if (pairs[0]->type == RigidBodyType::CUBOID && pairs[1]->type == RigidBodyType::PLANE) {
				Mat4 b1_world = pairs[0]->obj_to_world();
				ci = collision_box_plane(pairs[0], pairs[1], b1_world, data);
				resolve = ci && ci->is_valid;
			}
			else if (pairs[0]->type == RigidBodyType::SPHERE && pairs[1]->type == RigidBodyType::PLANE) {
				Mat4 b1_world = pairs[0]->obj_to_world();
				ci = collision_sphere_plane(pairs[0], pairs[1], b1_world, data);
				resolve = ci && ci->is_valid;
			}
			else if (pairs[0]->type == RigidBodyType::SPHERE && pairs[1]->type == RigidBodyType::SPHERE) {
				ci = collision_sphere_sphere(pairs[0], pairs[1], data);
				resolve = ci && ci->is_valid;
			}
			else if (pairs[0]->type == RigidBodyType::CUBOID && pairs[1]->type == RigidBodyType::SPHERE) {
				Mat4 b1_world = pairs[0]->obj_to_world();
				ci = collision_box_sphere(pairs[0], pairs[1], b1_world, data);
				resolve = ci && ci->is_valid;
			}
			if (resolve) {
				// Apply position change
				if (pairs[0]->type == RigidBodyType::CUBOID && pairs[1]->type == RigidBodyType::SPHERE) {
					int a = 4;
				}
				resolve_positions(data);
				// Apply velocity change
				resolve_velocities(data, ci, pairs);
			}
			data.reset();
		}
	}
}

void RigidBodySystemSimulator::simulateTimestep(float time_step) {
	 if (!running) {
	 	return;
	 }

	if (m_iTestCase == 3) {		
		static std::mt19937 eng;
		static std::uniform_real_distribution<float> randomer(-1.5, 2);
		static int counter = 0;
		counter++;
		if (counter > 500) {
			addRigidBody({ randomer(eng), 1.0, randomer(eng) }, { 0.3, 0.3, 0.3 }, 2);
			counter = 0;
		}
	}

	handle_collisions();

	//time_step *= 0.25;
	//time_step = 0.0000001;
	for (auto& rb : rigid_bodies) {
		if (!rb.movable)
			continue;
		rb.position += time_step * rb.linear_velocity;
		rb.linear_velocity += time_step * rb.force * rb.inv_mass;
		auto ang_vel = Quat(rb.angular_vel.x, rb.angular_vel.y, rb.angular_vel.z, 0);
		rb.orientation += time_step * 0.5f * rb.orientation * ang_vel;
		rb.orientation = rb.orientation.unit();
		rb.angular_momentum += time_step * rb.torque;
		//rb.angular_momentum *= 0.999;
		auto inv_inertia = rb.get_transformed_inertia(rb.inv_inertia_0);
		rb.angular_vel = inv_inertia * rb.angular_momentum;
	}
	// Clear forces & torques
	for (auto& rb : rigid_bodies) {
		rb.force = 0;
		rb.torque = 0;
	}

	if (m_iTestCase == 0) {
		for (int i = 0; i < rigid_bodies.size(); i++) {
			auto& rb = rigid_bodies[i];
			cout << "Linear Velocity: " << rb.linear_velocity << "\n";
			cout << "Angular Velocity: " << rb.angular_vel << "\n";
		}
		running = false;
	}
}

void RigidBodySystemSimulator::onClick(int x, int y) {
	trackmouse.x = x;
	trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y) {
	old_trackmouse.x = x;
	old_trackmouse.y = y;
	trackmouse.x = x;
	trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return rigid_bodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	return rigid_bodies[i].position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return rigid_bodies[i].linear_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return rigid_bodies[i].angular_vel;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	rigid_bodies[i].force += force;
	rigid_bodies[i].torque += cross((loc - rigid_bodies[i].position), force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	rigid_bodies.push_back({ position, size, mass });
}

void RigidBodySystemSimulator::add_sphere(const Vec3& pos, float radius, int mass) {
	RigidBody sphere(radius, pos, mass);
	rigid_bodies.emplace_back(sphere);
}


void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	rigid_bodies[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	rigid_bodies[i].linear_velocity = velocity;
}

void RigidBodySystemSimulator::add_torque(int i, Vec3 ang_accelaration) {

	auto inertia = rigid_bodies[i].get_transformed_inertia(
		rigid_bodies[i].inv_inertia_0.inverse()
	);
	rigid_bodies[i].torque += inertia * ang_accelaration;
}

void RigidBodySystemSimulator::resolve_positions(CollisionData& data) {
	// Apply positional change
	auto iter_cnt = data.num_contacts == 1 ? 1 : 4 * data.num_contacts;
	Contact* collision_info;
	Vec3 angular_delta[2] = { Vec3(), Vec3() };
	Vec3 linear_delta[2] = { Vec3(), Vec3() };

	constexpr float angular_limit = 0.2f;
	for (int iter = 0; iter < iter_cnt; iter++) {
		auto max = 0.001f;
		int index = data.num_contacts;
		for (int i = 0; i < data.num_contacts; i++) {
			if (data.contacts[i].penetration > max) {
				max = data.contacts[i].penetration;
				index = i;
				collision_info = &data.contacts[i];
			}
		}
		if (index == data.num_contacts) {
			break;
		}
		float angular_mov[2];
		float linear_mov[2];
		float total_inertia = 0;
		float linear_inertia[2];
		float angular_inertia[2];

		for (int i = 0; i < 2; i++) {
			if (collision_info->bodies[i]) {
				Mat4 inv_inertia = collision_info->bodies[i]->get_transformed_inertia(
					collision_info->bodies[i]->inv_inertia_0
				);
				//Mat4 inv_inertia = collision_info->bodies[i]->inv_inertia_0;
				// -Linear inertia is proportional to inv_mass
				// -Angular inertia is proportional to linear projection of velocity along 
				// the normal induced by delta-angular velocity(impulsive torque)
				Vec3 ang_comp = cross(collision_info->cp_rel[i], collision_info->normal);
				ang_comp = inv_inertia * ang_comp;
				// Get induced linear velocity then project
				ang_comp = cross(ang_comp, collision_info->cp_rel[i]);
				angular_inertia[i] = dot(ang_comp, collision_info->normal);

				linear_inertia[i] = collision_info->bodies[i]->inv_mass;
				total_inertia += angular_inertia[i] + linear_inertia[i];
			}
		}

		for (int i = 0; i < 2; i++) {
			if (collision_info->bodies[i]) {
				auto sign = i == 0 ? 1 : -1;
				angular_mov[i] = sign * collision_info->penetration * (angular_inertia[i] / total_inertia);
				linear_mov[i] = sign * collision_info->penetration * (linear_inertia[i] / total_inertia);
				// Bigger bodies should be able to rotate more
				Vec3 proj = collision_info->cp_rel[i] - collision_info->normal * dot(collision_info->normal, collision_info->cp_rel[i]);
				auto max_angle = angular_limit * norm(proj);
				// Distribute the clamp
				if (angular_mov[i] < -max_angle) {
					auto tot = angular_mov[i] + linear_mov[i];
					angular_mov[i] = -max_angle;
					linear_mov[i] = tot - angular_mov[i];

				}
				else if (angular_mov[i] > max_angle) {
					auto tot = angular_mov[i] + linear_mov[i];
					angular_mov[i] = max_angle;
					linear_mov[i] = tot - angular_mov[i];
				}

				if (angular_mov[i]) {
					Mat4 inv_inertia = collision_info->bodies[i]->get_transformed_inertia(
						collision_info->bodies[i]->inv_inertia_0
					);
					Vec3 tmp = cross(collision_info->cp_rel[i], collision_info->normal);
					angular_delta[i] = inv_inertia * tmp * (angular_mov[i] / angular_inertia[i]);
				}
				else {
					angular_delta[i] = 0;
				}

				linear_delta[i] = collision_info->normal * linear_mov[i];

				// Apply deltas
				collision_info->bodies[i]->position += collision_info->normal * linear_mov[i];
				Quat q(angular_delta[i].x, angular_delta[i].y, angular_delta[i].z, 0);
				//q *= collision_info->bodies[i]->orientation;

				//std::cout << "Adjust pos & quat" << angular_delta[i].x <<" " <<angular_delta[i].y <<" " << angular_delta[i].z << std::endl;
				//std::cout << "Angular velocity " << collision_info->bodies[i]->angular_vel.x << " " <<
				//collision_info->bodies[i]->angular_vel.y <<" " << collision_info->bodies[i]->angular_vel.z << std::endl;
				collision_info->bodies[i]->orientation += 0.5 * collision_info->bodies[i]->orientation * q;
				collision_info->bodies[i]->orientation.x += angular_delta[i].x * 0.5;
				collision_info->bodies[i]->orientation.y += angular_delta[i].y * 0.5;
				collision_info->bodies[i]->orientation.z += angular_delta[i].z * 0.5;
				//printf("Orientation delta %f %f %f\n", angular_delta[i].x, angular_delta[i].y, angular_delta[i].z);
			}
		}
		// Propagate
		for (int i = 0; i < data.num_contacts; i++) {
			for (int j = 0; j < 2; j++) {
				if (data.contacts[i].bodies[j]) {
					for (int k = 0; k < 2; k++) {
						if (data.contacts[i].bodies[j] == data.contacts[index].bodies[k]) {
							auto sgn = j ? 1 : -1;
							Vec3 delta_pos = linear_delta[k] + cross(angular_delta[k], data.contacts[i].cp_rel[j]);
							data.contacts[i].penetration += dot(delta_pos, data.contacts[i].normal) * sgn;
						}
					}
				}

			}
		}
	}
}

void RigidBodySystemSimulator::resolve_velocities(CollisionData& data, Contact* best_col,
	const std::vector<RigidBody*>& pairs) {
	auto iter_cnt = data.num_contacts == 1 ? 1 : 4 * data.num_contacts;
	Vec3 angular_mom_delta[2] = { Vec3(), Vec3() };
	Vec3 linear_vel_delta[2] = { Vec3(), Vec3() };
	Contact* collision_info;
	// Set initial expected velocity after collision (Relative, Va - Vb)
	for (int i = 0; i < data.num_contacts; i++) {
		auto b1_collision_pos = data.contacts[i].collision_point - pairs[0]->position;
		auto b2_collision_pos = data.contacts[i].collision_point - pairs[1]->position;
		auto b1_collision_vel = pairs[0]->linear_velocity + cross(pairs[0]->angular_vel, b1_collision_pos);
		auto b2_collision_vel = pairs[1]->linear_velocity + cross(pairs[1]->angular_vel, b2_collision_pos);
		auto normal_dot_relative_vel = dot(data.contacts[i].normal, b1_collision_vel - b2_collision_vel);
		data.contacts[i].relative_vel = normal_dot_relative_vel;
		data.contacts[i].expected_vel = -(1 + bounciness) * normal_dot_relative_vel;
	}
	for (int iter = 0; iter < iter_cnt; iter++) {
		int index = data.num_contacts;
		auto max = 0.001;
		for (int i = 0; i < data.num_contacts; i++) {
			if (data.contacts[i].expected_vel > max) {
				max = data.contacts[i].expected_vel;
				index = i;
				collision_info = &data.contacts[i];
			}
		}
		if (index == data.num_contacts) {
			break;
		}

		//collision_info = best_col;
		auto b1_inv_inertia = pairs[0]->get_transformed_inertia(pairs[0]->inv_inertia_0);
		auto b2_inv_inertia = pairs[1]->get_transformed_inertia(pairs[1]->inv_inertia_0);
		auto b1_collision_pos = collision_info->collision_point - pairs[0]->position;
		auto b2_collision_pos = collision_info->collision_point - pairs[1]->position;
		auto b1_collision_vel = pairs[0]->linear_velocity + cross(pairs[0]->angular_vel, b1_collision_pos);
		auto b2_collision_vel = pairs[1]->linear_velocity + cross(pairs[1]->angular_vel, b2_collision_pos);
		auto normal_dot_relative_vel = dot(collision_info->normal, b1_collision_vel - b2_collision_vel);

		if (collision_info->relative_vel < 0) {
			auto numerator = collision_info->expected_vel;
			auto denominator = pairs[0]->inv_mass + pairs[1]->inv_mass
				+ dot(
					cross(b1_inv_inertia * cross(b1_collision_pos, collision_info->normal), b1_collision_pos) +
					cross(b2_inv_inertia * cross(b2_collision_pos, collision_info->normal), b2_collision_pos),
					collision_info->normal);

			auto impulse = numerator / denominator;
			linear_vel_delta[0] = impulse * collision_info->normal * pairs[0]->inv_mass;
			linear_vel_delta[1] = impulse * collision_info->normal * pairs[1]->inv_mass;
			angular_mom_delta[0] = cross(b1_collision_pos, impulse * collision_info->normal);
			angular_mom_delta[1] = cross(b2_collision_pos, impulse * collision_info->normal);
			pairs[0]->linear_velocity += linear_vel_delta[0];
			pairs[1]->linear_velocity -= linear_vel_delta[1];

			pairs[0]->angular_momentum += angular_mom_delta[0];
			pairs[1]->angular_momentum -= angular_mom_delta[1];
			Vec3 delta_vel[2];
			delta_vel[0] = linear_vel_delta[0] + cross(b1_inv_inertia * angular_mom_delta[0], b1_collision_pos);
			delta_vel[1] = linear_vel_delta[1] + cross(b2_inv_inertia * angular_mom_delta[1], b2_collision_pos);
			auto del = dot(delta_vel[0] - delta_vel[1], collision_info->normal);
			calc_after_col_vel(collision_info, del + collision_info->relative_vel, pairs);
		}
	}
}

void RigidBodySystemSimulator::calc_after_col_vel(Contact* contact, float delta_vel,
	const std::vector<RigidBody*>& pairs) {
	contact->relative_vel = delta_vel;
	contact->expected_vel = -(1 + bounciness) * delta_vel;
}

void RigidBodySystemSimulator::pass_time_step_variable(float& time_step) {
	timestep = &time_step;
}

