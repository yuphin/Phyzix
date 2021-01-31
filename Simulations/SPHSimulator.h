#pragma once
#include "Simulator.h"
#include "vectorbase.h"

// Reference resources:
//	[Koschier2019] https://interactivecomputergraphics.github.io/SPH-Tutorial/pdf/SPH_Tutorial.pdf
//	[Akinci2012] https://cg.informatik.uni-freiburg.de/publications/2012_SIGGRAPH_rigidFluidCoupling.pdf
//	Sample implementation: https://github.com/InteractiveComputerGraphics/SPlisHSPlasH	


class SPHSimulator : public Simulator {
public:
	SPHSimulator();
	void pass_time_step_variable(float time_step);
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC) override;
	void reset() override;
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext) override;
	void notifyCaseChanged(int testCase) override;
	void simulateTimestep(float timeStep) override;
	void externalForcesCalculations(float timeElapsed) override;
	void onClick(int x, int y) override;
	void onMouse(int x, int y) override;
private:
	void compute_density();
	void compute_non_pressure_forces();
	void enforce_continuity();
	void solve_pressure();
	void integrate();

	ID3D11DeviceContext* context = nullptr;
	ID3D11Device* device = nullptr;
	Point2D mouse;
	Point2D track_mouse;
	Point2D old_track_mouse;
	int num_particles;
	float* time_step;
	Vec3 gravity;
	Real particle_radius;
	Real support_radius;


};

