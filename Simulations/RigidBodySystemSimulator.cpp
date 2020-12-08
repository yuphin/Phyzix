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
	switch (testCase) {
	case 0:
		break;
	case 1:
		addRigidBody({ 0, 0, 0 }, { 1, 1, 1 }, 10);
		applyForceOnBody(0, { 0.0, 0.5, 0.5 }, { 1,1,0 });
		add_torque(0, { 5000, 5000, 5000 });
		break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {}

void RigidBodySystemSimulator::handleCollisions() {
	if (rigid_bodies.size() < 2) {
		return;
	}

	for (int i = 0; i < rigid_bodies.size(); i++) {
		RigidBody& b1 = rigid_bodies[i];

		for (int j = i + 1; i < rigid_bodies.size(); j++) {
			RigidBody& b2 = rigid_bodies[j];

			auto collision_info = checkCollisionSAT(b1.get_world_matrix(), b2.get_world_matrix());

			if (collision_info.isValid) {
				auto b1_collision_pos = collision_info.collisionPointWorld - b1.position;
				auto b2_collision_pos = collision_info.collisionPointWorld - b2.position;

				auto b1_collision_vel = b1.linear_velocity + cross(b1.angular_vel, b1_collision_pos);
				auto b2_collision_vel = b2.linear_velocity + cross(b2.angular_vel, b2_collision_pos);

				auto normal_dot_relative_vel = dot(collision_info.normalWorld, b1_collision_vel - b2_collision_vel);

				if (normal_dot_relative_vel < 0) {
					auto numerator = -(1 + bounciness) * normal_dot_relative_vel;
					auto denominator = b1.inv_mass + b2.inv_mass
						+ dot(
							cross(b1.inv_inertia_0 * cross(b1_collision_pos, collision_info.normalWorld), b1_collision_pos) + 
							cross(b2.inv_inertia_0 * cross(b2_collision_pos, collision_info.normalWorld), b2_collision_pos),
							collision_info.normalWorld);

					auto impulse = numerator / denominator;

					b1.linear_velocity += impulse * collision_info.normalWorld * b1.inv_mass;
					b2.linear_velocity -= impulse * collision_info.normalWorld * b2.inv_mass;

					b1.angular_momentum += cross(b1_collision_pos, impulse * collision_info.normalWorld);
					b2.angular_momentum -= cross(b2_collision_pos, impulse * collision_info.normalWorld);
				}
			}
		}
	}
}

void RigidBodySystemSimulator::simulateTimestep(float time_step) {
	handleCollisions();

	for (auto& rb : rigid_bodies) {
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
		rb.force = 0;
		rb.torque = 0;
	}
}

void RigidBodySystemSimulator::onClick(int x, int y) {}

void RigidBodySystemSimulator::onMouse(int x, int y) {}

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

