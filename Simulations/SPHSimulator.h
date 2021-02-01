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
	Particle(float mass, const Vec3& pos, bool boundary = false) {
		this->mass = mass;
		this->pos = pos;
	}
	float mass = 0;
	Vec3 accel = Vec3();
	Vec3 vel = Vec3();
	Vec3 pos = Vec3();
	Vec3 force = Vec3();
	Real density = 0;
	Real pressure = 0;
	Real old_pressure = 0;
	Real aii;
	Vec3 m_divRhoSqr_grad;
	Vec3 m_divRhoSqr_p_grad;
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
	void compute_boundary_masses();
	void compute_density();
	void compute_non_pressure_forces();
	void enforce_continuity(float time_step);
	void solve_pressure(float time_step);
	void integrate(float time_step);
	void handle_collision(Particle& particle, float dt);

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
	CubicSplineKernel kernel;
	std::vector<Particle> particles;
	std::vector<Particle> boundary_particles;
	Real rho_0;

};

