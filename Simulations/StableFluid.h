#pragma once
#include "Simulator.h"
#include "util/util.h"
#include "util/vectorbase.h"
#include <omp.h>

struct Vec2
{
	float x;
	float y;
};

class StableFluid : public Simulator {
public:
	StableFluid();
	void pass_time_step_variable(float& time_step);
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC) override;
	void reset() override;
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext) override;
	void notifyCaseChanged(int testCase) override;
	void simulateTimestep(float timeStep) override;
	void externalForcesCalculations(float timeElapsed) override;
	void onClick(int x, int y) override;
	void onMouse(int x, int y) override;
	void velocityStep();
	void densityStep();
	void tempStep();
	void addSource(float* x, float* x0);
	void setBoundaries(int b, float* x);
	void linearSolve(int b, float* x, float* x0, float a, float c);
	void diffuse(int b, float* x, float* x0);
	void advect(int b, float* d, float* d0, float* u, float* v);
	void project(float* u, float* v, float* u0, float* v0);
private:
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	float* timestep;
	bool enable_solid = false;

	int grid_size;
	int total_size;
	int jacobiNum;
	float viscosity;
	float diffusionRate;
	float temperatureRate;

	float* vel0_x;
	float* vel0_y;

	float* vel1_x;
	float* vel1_y;

	float* dens0;
	float* dens1;

	float* temp0;
	float* temp1;

	bool* solid;

	float gravity = 0;
	float buoyancy = 0;
};
