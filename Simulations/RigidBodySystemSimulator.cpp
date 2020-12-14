#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator(){ 
	m_iTestCase = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1, Demo 2, Demo 3, Demo 4";
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
		DUC->drawRigidBody(rigid_bodies[i].obj_to_world().toDirectXMatrix());
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
		*timestep = 0.01f;
		addRigidBody({ 0.25, -0.5, 0 }, { 1, 0.5, 0.5 }, 2);
		addRigidBody({ -0.25, 1, 0 }, { 1, 0.5, 0.5 }, 2);
		applyForceOnBody(0, { 0.25, -0.5, 0 }, { 0, 10,0 });
		applyForceOnBody(1, { -0.25, 1, 0 }, { 0, -35,0 });
		break;
	case 3:
		*timestep = 0.01f;
		addRigidBody({ 0, -1, 0 }, { 5, 0.1, 5 }, 100);
		rigid_bodies[0].is_kinematic = true;
		gravity = Vec3(0, -1, 0);
		break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Point2D mouseDiff;
	mouseDiff.x = trackmouse.x - old_trackmouse.x;
	mouseDiff.y = trackmouse.y - old_trackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
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
}

void RigidBodySystemSimulator::handleCollisions() {
	if (rigid_bodies.size() < 2) {
		return;
	}

	for (int i = 0; i < rigid_bodies.size(); i++) {
		RigidBody& b1 = rigid_bodies[i];

		for (int j = i + 1; j < rigid_bodies.size(); j++) {
			RigidBody& b2 = rigid_bodies[j];

			auto collision_info = checkCollisionSAT(b1.obj_to_world(), b2.obj_to_world());

			if (collision_info.isValid) {
				auto b1_collision_pos = collision_info.collisionPointWorld - b1.position;
				auto b2_collision_pos = collision_info.collisionPointWorld - b2.position;

				auto b1_collision_vel = b1.linear_velocity + cross(b1.angular_vel, b1_collision_pos);
				auto b2_collision_vel = b2.linear_velocity + cross(b2.angular_vel, b2_collision_pos);

				auto normal_dot_relative_vel = dot(collision_info.normalWorld, b1_collision_vel - b2_collision_vel);

				if (normal_dot_relative_vel <= 0) {

					auto numerator = -(1 + bounciness) * normal_dot_relative_vel;
					auto denominator = b1.inv_mass + b2.inv_mass
						+ dot(
							cross(b1.inv_inertia_0 * cross(b1_collision_pos, collision_info.normalWorld), b1_collision_pos) + 
							cross(b2.inv_inertia_0 * cross(b2_collision_pos, collision_info.normalWorld), b2_collision_pos),
							collision_info.normalWorld);

					auto impulse = numerator / denominator;

					if (!b1.is_kinematic) {
						b1.linear_velocity += impulse * collision_info.normalWorld * b1.inv_mass;
						b1.angular_momentum += cross(b1_collision_pos, impulse * collision_info.normalWorld);
					}

					if (!b2.is_kinematic) {
						b2.linear_velocity -= impulse * collision_info.normalWorld * b2.inv_mass;
						b2.angular_momentum -= cross(b2_collision_pos, impulse * collision_info.normalWorld);
					}
				}
			}
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

	handleCollisions();

	for (auto& rb : rigid_bodies) {
		if (rb.is_kinematic) {
			continue;
		}
		rb.position += time_step * rb.linear_velocity;
		rb.linear_velocity += time_step * rb.force * rb.inv_mass;
		auto ang_vel = Quat(rb.angular_vel.x, rb.angular_vel.y, rb.angular_vel.z, 0);
		rb.orientation += time_step * 0.5f * ang_vel * rb.orientation;
		rb.orientation = rb.orientation.unit();
		rb.angular_momentum += time_step * rb.torque;
		auto inv_inertia = rb.get_transformed_inertia(rb.inv_inertia_0);
		rb.angular_vel = inv_inertia * rb.angular_momentum;
	}
	// Clear forces & torques
	for (auto& rb : rigid_bodies) {
		rb.force = mouse_force + gravity;
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

void RigidBodySystemSimulator::passTimestepVariable(float& time_step)
{
	timestep = &time_step;
}

