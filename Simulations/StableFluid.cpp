#include "StableFluid.h"

#define IX(i,j) ((i) + (grid_size + 2) * (j))
#define SWAP(x0,x) {auto* tmp=x0; x0=x; x=tmp;}

#define FIRE_TEMP 593.0

StableFluid::StableFluid()
{
	grid_size = 128;
	total_size = (grid_size + 2) * (grid_size + 2);

	jacobiNum = 20;

	vel0_x = new float[total_size];
	vel1_x = new float[total_size];

	vel0_y = new float[total_size];
	vel1_y = new float[total_size];
	
	dens0 = new float[total_size];
	dens1 = new float[total_size];

	temp0 = new float[total_size];
	temp1 = new float[total_size];

	solid = new bool[total_size];
}

void StableFluid::pass_time_step_variable(float& time_step)
{
	timestep = &time_step;
}

const char* StableFluid::getTestCasesStr()
{
	return "Flame 1, Flame 2";
}

void StableFluid::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Enable Solid", TW_TYPE_BOOLCPP, &enable_solid, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &gravity, "step=0.001 min=0 max=100 ");
	TwAddVarRW(DUC->g_pTweakBar, "Buoyancy", TW_TYPE_FLOAT, &buoyancy, "step=0.001 min=0 max=100");
	TwAddVarRW(DUC->g_pTweakBar, "Density Diffusion", TW_TYPE_FLOAT, &diffusionRate, "min=0 max=1");
	TwAddVarRW(DUC->g_pTweakBar, "Temperature Diffusion", TW_TYPE_FLOAT, &temperatureRate, "min=0 max=1");
	TwAddVarRW(DUC->g_pTweakBar, "Viscosity", TW_TYPE_FLOAT, &viscosity, "min=0 max=1");
}

void StableFluid::reset()
{
	for (int i = 0; i < total_size; i++) {
		vel0_x[i] = vel1_x[i] = vel0_y[i] = vel1_y[i] = dens0[i] = dens1[i] = temp0[i] = temp1[i] = 0.0f;
		solid[i] = false;
	}

	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void StableFluid::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (int i = 0; i <= grid_size + 1; i++) {
		for (int j = 0; j <= grid_size + 1; j++) {
			Vec3 color = {};
			float density = dens1[IX(i, j)];
			float temp = temp1[IX(i, j)];

			if (solid[IX(i, j)]) {
				color = { 0, 0, 1};
			}
			else if (temp < 323) {
				color = { density / 4, density / 4, density / 4 };
			}
			else if (temp < 503) {
				color = { (1.0 - (temp / FIRE_TEMP)) * density * 4, (0.4 - (temp / FIRE_TEMP) * 0.4) * density * 4, 0 };
			}
			else if (temp < 573) {
				color = { (temp / FIRE_TEMP) * density * 4, 0.2 * (temp / FIRE_TEMP), 0 };
			}
			else {
				color = { density * 4, 0.85 * density * 4, 0 };
			}

			if (density == 0 && !solid[IX(i, j)]) {
				continue;
			}

			//color = { 1, 0, 0 };

			DUC->setUpLighting(Vec3(), Vec3(), 2000.0, color);
			DUC->drawSphere(Vec3(i * 0.1f - grid_size * 0.05f, j * 0.1f, 0), { 0.1,0.1,0.1 });
		}
	}
}

void StableFluid::notifyCaseChanged(int testCase)
{
	reset();
	m_iTestCase = testCase;

	switch (m_iTestCase)
	{
	case 0:
		*timestep = 0.1f;
		viscosity = 0.0f;
		diffusionRate = 1e-10f;
		temperatureRate = 0.00005f;
		gravity = 0.5f;
		buoyancy = 1.5f;
		cout << "Solver!\n";
		break;
	case 1:
		*timestep = 0.1f;
		viscosity = 0.0f;
		gravity = 0;
		buoyancy = 0;
		diffusionRate = 0;
		temperatureRate = 0;
		cout << "Solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void StableFluid::addSource(float* x, float* x0) {
	#pragma omp parallel for schedule(static)
	for (int i = 0; i < total_size; i++) {
		if (solid[i]) {
			continue;
		}
		x[i] += (*timestep) * x0[i];
	}
}

void StableFluid::setBoundaries(int b, float* x) {
	#pragma omp parallel for schedule(static)
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
	}

	x[IX(0, 0)] = 0.5f * (x[IX(1, 0)] + x[IX(0, 1)]);
	x[IX(0, grid_size + 1)] = 0.5f * (x[IX(1, grid_size + 1)] + x[IX(0, grid_size)]);
	x[IX(grid_size + 1, 0)] = 0.5f * (x[IX(grid_size, 0)] + x[IX(grid_size + 1, 1)]);
	x[IX(grid_size + 1, grid_size + 1)] = 0.5f * (x[IX(grid_size, grid_size + 1)] + x[IX(grid_size + 1, grid_size)]);

	#pragma omp parallel for schedule(static)
	for (int i = 1; i <= grid_size; i++) {
		for (int j = 1; j <= grid_size; j++) {
			if (solid[IX(i - 1, j)]) {
				x[IX(i, j)] = -x[IX(i + 1, j)];
			}
			if (solid[IX(i + 1, j)]) {
				x[IX(i, j)] = -x[IX(i - 1, j)];
			}
			if (solid[IX(i, j - 1)]) {
				x[IX(i, j)] = -x[IX(i, j + 1)];
			}
			if (solid[IX(i, j + 1)]) {
				x[IX(i, j)] = -x[IX(i, j - 1)];
			}
		}
	}
}

void StableFluid::linearSolve(int b, float* x, float* x0, float a, float c) {
	for (int i = 0; i < jacobiNum; i++) {
		#pragma omp parallel for schedule(static)
		for (int i = 1; i <= grid_size; i++) {
			for (int j = 1; j <= grid_size; j++) {
				x[IX(i, j)] = (x0[IX(i, j)] + a * (x[IX(i - 1, j)] + x[IX(i + 1, j)] + x[IX(i, j - 1)] + x[IX(i, j + 1)])) / c;
			}
		}
		setBoundaries(b, x);
	}
}

void StableFluid::diffuse(int b, float* x, float* x0) {
	float rate = viscosity;

	if (b == 0) {
		rate = diffusionRate;
	}
	else if (b == 3) {
		rate = temperatureRate;
	}

	float a = (*timestep) * rate * grid_size * grid_size;
	linearSolve(b, x, x0, a, 1 + 4 * a);
}

void StableFluid::advect(int b, float* d, float* d0, float* u, float* v) {
	float timestep0 = (*timestep) * grid_size;
	#pragma omp parallel for schedule(static)
	for (int i = 1; i <= grid_size; i++) {
		for (int j = 1; j <= grid_size; j++) {
			if (solid[IX(i, j)]) {
				continue;
			}
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
	#pragma omp parallel for schedule(static)
	for (int i = 1; i <= grid_size; i++) {
		for (int j = 1; j <= grid_size; j++) {
			v0[IX(i, j)] = -0.5f * (u[IX(i + 1, j)] - u[IX(i - 1, j)] + v[IX(i, j + 1)] - v[IX(i, j - 1)]) / grid_size;
			u0[IX(i, j)] = 0;
		}
	}
	setBoundaries(0, v0);
	setBoundaries(0, u0);

	linearSolve(0, u0, v0, 1, 4);

	#pragma omp parallel for schedule(static)
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

void StableFluid::tempStep() {
	addSource(temp1, temp0);
	SWAP(temp0, temp1);
	diffuse(3, temp1, temp0);
	SWAP(temp0, temp1);
	advect(3, temp1, temp0, vel1_x, vel1_y);
}

void StableFluid::simulateTimestep(float timeStep)
{
	for (int i = 0; i < total_size; i++) {
		vel0_x[i] = vel0_y[i] = dens0[i] = temp0[i] = 0.0f;
	}
	
	for (int i = -10; i <= 10; i++) {
		for (int j = -2; j <= 2; j++) {
			dens0[IX(64 + i, 2 + j)] = 4.0f;
			temp0[IX(64 + i, 2 + j)] = FIRE_TEMP * 5;
			vel0_y[IX(64 + i, 2 + j)] += 50;
		}
	}

	for (int i = 1; i <= grid_size; i++) {
		for (int j = 1; j <= grid_size; j++) {
			vel0_y[IX(i, j)] += (dens0[IX(i, j)] * gravity + (1.0 / (temp0[IX(i, j)] + 1) ) * buoyancy);
		}
	}

	for (int i = 56; i < 72; i++) {
		solid[IX(i, 64)] = enable_solid;
	}

	velocityStep();
	densityStep();
	tempStep();
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


