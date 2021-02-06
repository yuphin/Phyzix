#include "StableFluid.h"

#define IX(i,j) ((i) + (grid_size + 2) * (j))
#define SWAP(x0,x) {auto* tmp=x0; x0=x; x=tmp;}

StableFluid::StableFluid()
{
	grid_size = 32;
	total_size = (grid_size + 2) * (grid_size + 2);

	viscosity = 0.0f;
	diffusionRate = 0.0f;
	jacobiNum = 20;

	vel0_x = new float[total_size];
	vel1_x = new float[total_size];

	vel0_y = new float[total_size];
	vel1_y = new float[total_size];
	
	dens0 = new float[total_size];
	dens1 = new float[total_size];

	for (int i = 0; i < total_size; i++) {
		vel0_x[i] = vel1_x[i] = vel0_y[i] = vel1_y[i] = dens0[i] = dens1[i] = 0.0f;
	}
}

void StableFluid::pass_time_step_variable(float& time_step)
{
	timestep = &time_step;
}

const char* StableFluid::getTestCasesStr()
{
	return "Solver";
}

void StableFluid::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void StableFluid::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	*timestep = 0.4f;
}

void StableFluid::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (int i = 0; i <= grid_size + 1; i++) {
		for (int j = 0; j <= grid_size + 1; j++) {
			DUC->setUpLighting(Vec3(dens1[IX(i,j)], dens1[IX(i, j)], dens1[IX(i, j)]), Vec3(), 2000.0, Vec3(0.1, 0.1, 0.1));
			DUC->drawSphere(Vec3(i * 0.1f, j * 0.1f, 0), { 0.1,0.1,0.1 });
		}
	}
}

void StableFluid::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	//
	//to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		*timestep = 0.1f;
		cout << "Solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void StableFluid::addSource(float* x, float* x0) {
	for (int i = 0; i < total_size; i++) {
		x[i] += (*timestep) * x0[i];
	}
}

void StableFluid::setBoundaries(int b, float* x) {
	for (int i = 1; i <= grid_size; i++) {
		if (b == 1) {
			x[IX(0, i)] = -x[IX(1, i)];
			x[IX(grid_size + 1, i)] = -x[IX(grid_size, i)];
		}
		else {
			x[IX(0, i)] = x[IX(1, i)];
			x[IX(grid_size + 1, i)] = x[IX(grid_size, i)];
		}
		if (b == 2) {
			x[IX(i, 0)] = -x[IX(i, 1)];
			x[IX(i, grid_size + 1)] = -x[IX(i, grid_size)];
		}
		else {
			x[IX(i, 0)] = x[IX(i, 1)];
			x[IX(i, grid_size + 1)] = x[IX(i, grid_size)];
		}

		x[IX(0, 0)] = 0.5f * (x[IX(1, 0)] + x[IX(0, 1)]);
		x[IX(0, grid_size + 1)] = 0.5f * (x[IX(1, grid_size + 1)] + x[IX(0, grid_size)]);
		x[IX(grid_size + 1, 0)] = 0.5f * (x[IX(grid_size, 0)] + x[IX(grid_size + 1, 1)]);
		x[IX(grid_size + 1, grid_size + 1)] = 0.5f * (x[IX(grid_size, grid_size + 1)] + x[IX(grid_size + 1, grid_size)]);
	}
}

void StableFluid::linearSolve(int b, float* x, float* x0, int a, int c) {
	for (int i = 0; i < jacobiNum; i++) {
		for (int i = 1; i <= grid_size; i++) {
			for (int j = 1; j <= grid_size; j++) {
				x[IX(i, j)] = (x0[IX(i, j)] + a * (x[IX(i - 1, j)] + x[IX(i + 1, j)] + x[IX(i, j - 1)] + x[IX(i, j + 1)])) / c;
			}
		}
		setBoundaries(b, x);
	}
}

void StableFluid::diffuse(int b, float* x, float* x0) {
	float rate = b == 0 ? diffusionRate : viscosity;

	float a = (*timestep) * rate * grid_size * grid_size;
	linearSolve(b, x, x0, a, 1 + 4 * a);
}

void StableFluid::advect(int b, float* d, float* d0, float* u, float* v) {
	float timestep0 = (*timestep) * grid_size;
	for (int i = 1; i <= grid_size; i++) {
		for (int j = 1; j <= grid_size; j++) {
			float x = i - timestep0 * u[IX(i, j)];
			float y = j - timestep0 * v[IX(i, j)];
			if (x < 0.5f) x = 0.5f;
			if (x > grid_size + 0.5) x = grid_size + 0.5f;
			int i0 = (int)x;
			int i1 = i0 + 1;
			if (y < 0.5f) y = 0.5f;
			if (y > grid_size + 0.5) y = grid_size + 0.5f;
			int j0 = (int)y;
			int j1 = j0 + 1;
			float s1 = x - i0;
			float s0 = 1 - s1;
			float t1 = y - j0;
			float t0 = 1 - t1;

			d[IX(i, j)] = s0 * (t0 * d0[IX(i0, j0)] + t1 * d0[IX(i0, j1)]) + s1 * (t0 * d0[IX(i1, j0)] + t1 * d0[IX(i1, j1)]);
		}
	}
	setBoundaries(b, d);
}

void StableFluid::project(float* u, float* v, float* u0, float* v0) {
	for (int i = 1; i <= grid_size; i++) {
		for (int j = 1; j <= grid_size; j++) {
			v0[IX(i, j)] = -0.5f * (u[IX(i + 1, j)] - u[IX(i - 1, j)] + v[IX(i, j + 1)] - v[IX(i, j - 1)]) / grid_size;
			u0[IX(i, j)] = 0;
		}
	}
	setBoundaries(0, v0);
	setBoundaries(0, u0);

	linearSolve(0, u0, v0, 1, 4);

	for (int i = 1; i <= grid_size; i++) {
		for (int j = 1; j <= grid_size; j++) {
			u[IX(i, j)] -= 0.5f * grid_size * (u0[IX(i + 1, j)] - u0[IX(i - 1, j)]);
			v[IX(i, j)] -= 0.5f * grid_size * (u0[IX(i, j + 1)] - u0[IX(i, j - 1)]);
		}
	}

	setBoundaries(1, u);
	setBoundaries(2, v);

}

void StableFluid::velocityStep() {
	addSource(vel1_x, vel0_x);
	addSource(vel1_y, vel0_y);
	SWAP(vel0_x, vel1_x);
	SWAP(vel0_y, vel1_y);
	diffuse(1, vel1_x, vel0_x);
	diffuse(2, vel1_y, vel0_y);
	project(vel1_x, vel1_y, vel0_x, vel0_y);
	SWAP(vel0_x, vel1_x);
	SWAP(vel0_y, vel1_y);
	advect(1, vel1_x, vel0_x, vel0_x, vel0_y);
	advect(2, vel1_y, vel0_y, vel0_x, vel0_y);
	project(vel1_x, vel1_y, vel0_x, vel0_y);
}

void StableFluid::densityStep() {
	addSource(dens1, dens0);
	SWAP(dens0, dens1);
	diffuse(0, dens1, dens0);
	SWAP(dens0, dens1);
	advect(0, dens1, dens0, vel1_x, vel1_y);
}

void StableFluid::simulateTimestep(float timeStep)
{
	for (int i = 0; i < total_size; i++) {
		vel0_x[i] = vel0_y[i] = dens0[i] = 0.0f;
	}

	dens0[IX(16, 2)] = 10.0f;

	auto radians = 90 * 0.0174532925f;
	auto x_velocity = 40 * cos(radians);
	auto y_velocity = 40 * sin(radians);
	
	vel0_x[IX(16, 2)] = x_velocity;
	vel0_y[IX(16, 2)] = y_velocity;

	velocityStep();
	densityStep();
}

void StableFluid::externalForcesCalculations(float timeElapsed)
{
}

void StableFluid::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void StableFluid::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}


