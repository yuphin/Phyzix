#include "SPHSimulator.h"

SPHSimulator::SPHSimulator() {
	num_particles = 10;
	gravity = Vec3(0, -9.81, 0);
	particle_radius = 0.025;
	// For kernel
	support_radius = 4 * particle_radius;

}

const char* SPHSimulator::getTestCasesStr() {
    return "Box";
}

void SPHSimulator::initUI(DrawingUtilitiesClass* DUC) {
}

void SPHSimulator::reset() {
	mouse.x = mouse.y = 0;
	track_mouse.x = track_mouse.y = 0;
	old_track_mouse.x = old_track_mouse.y = 0;
}

void SPHSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
}

void SPHSimulator::notifyCaseChanged(int testCase) {
}

void SPHSimulator::simulateTimestep(float timeStep) {
	// Implements IISPH as described in [Koschier2019] pp. 14
	compute_density();
	compute_non_pressure_forces();
	enforce_continuity();
	solve_pressure();
	integrate();
}

void SPHSimulator::compute_density() {
}

void SPHSimulator::compute_non_pressure_forces() {
}

void SPHSimulator::enforce_continuity() {
}

void SPHSimulator::solve_pressure() {
}

void SPHSimulator::integrate() {
}

void SPHSimulator::externalForcesCalculations(float timeElapsed) {
}

void SPHSimulator::onClick(int x, int y) {
}

void SPHSimulator::onMouse(int x, int y) {
}



void SPHSimulator::pass_time_step_variable(float time_step) {
	this->time_step = &time_step;
}
