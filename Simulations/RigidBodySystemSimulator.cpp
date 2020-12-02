#include "RigidBodySystemSimulator.h"

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {}

void RigidBodySystemSimulator::reset() {}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* context) {}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {}

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
	return rigid_bodies[i].angular_velocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	rigid_bodies.push_back({ position, size, mass });
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	rigid_bodies[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	rigid_bodies[i].linear_velocity = velocity;
}
