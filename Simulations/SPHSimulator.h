#pragma once
#include "Simulator.h"
#include "vectorbase.h"
#include "Kernel.h"
// Reference resources:
//	[Koschier2019] https://interactivecomputergraphics.github.io/SPH-Tutorial/pdf/SPH_Tutorial.pdf
//	[Akinci2012] https://cg.informatik.uni-freiburg.de/publications/2012_SIGGRAPH_rigidFluidCoupling.pdf
//	Sample implementation: https://github.com/InteractiveComputerGraphics/SPlisHSPlasH	
//  


struct Particle {
	Particle(Real dm, Real dv, const Vec3& pos, bool boundary = false) {
		this->mass = dm;
		this->dv = dv;
		this->pos = pos;
	}
	Real mass = 0;
	Real dv = 0;
	Vec3 accel = Vec3();
	Vec3 vel = Vec3();
	Vec3 pos = Vec3();
	Vec3 force = Vec3();
	Real rho = 0;
	Real pressure = 0;
	Real old_pressure = 0;
	Real aii;
	Vec3 dv_div_rhoSqr_grad;
	Vec3 dv_divRhoSqr_p_grad;
	Real rho_star;
};

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
	void init_sim();
	void compute_boundary_volumes();
	void update_time_step(float& time_step);
	void compute_density();
	void compute_non_pressure_forces();
	void enforce_continuity(float time_step);
	void solve_pressure(float time_step);
	void integrate(float time_step);

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
	CubicSplineKernel2 kernel;
	std::vector<Particle> particles;
	std::vector<Particle> boundary_particles;
	Real rho_0;
	Real cfl_k = 1;
	bool is_2d = true;
	Real dv = 0;
	Real dm = 0;
};

