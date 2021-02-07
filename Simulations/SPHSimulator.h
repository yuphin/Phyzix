#pragma once
#include "Simulator.h"
#include "vectorbase.h"
#include "Kernel.h"
#include "Particle.h"
#include "NeighborhoodSearcher.h"
// Reference resources:
//	[Koschier2019] https://interactivecomputergraphics.github.io/SPH-Tutorial/pdf/SPH_Tutorial.pdf
//	[Akinci2012] https://cg.informatik.uni-freiburg.de/publications/2012_SIGGRAPH_rigidFluidCoupling.pdf
//	Sample implementation: https://github.com/InteractiveComputerGraphics/SPlisHSPlasH	
//  
class Sandbox;

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
	void init_sim(const Vec3& gravity, const Real rho_0, 
				  const int num_particles, const Real particle_radius);
	void compute_boundary_volumes();
	void update_time_step(float& time_step);
	void compute_density();
	void compute_non_pressure_forces();
	void enforce_continuity(float time_step);
	void solve_pressure(float time_step);
	void integrate(float time_step);
	void init_particles(const Vec3& x_offset, const Vec3& y_offset, 
						const Vec3& z_offset, const int DIM_X, 
						const int DIM_Y, const int DIM_Z);

private:
	friend Sandbox;

	ID3D11DeviceContext* context = nullptr;
	ID3D11Device* device = nullptr;
	Point2D mouse;
	Point2D track_mouse;
	Point2D old_track_mouse;
	int num_particles;
	float time_step;
	Vec3 gravity;
	Real particle_radius;
	Real boundary_radius;
	Real support_radius;
	CubicSplineKernel2 kernel;
	std::vector<Particle> particles;
	std::vector<Particle> moving_boundary_particles;
	std::vector<Particle> boundary_particles;
	Real rho_0;
	Real cfl_k = 1;
	bool is_2d = false;
	Real dv = 0;
	Real dm = 0;
	const int FLD_ID = 0;
	const int BND_ID = 1;
	const int MV_BND_ID = 2;
	Vec3 box_t = Vec3(3, 0, -1);
	Vec3 box_r = Vec3();
	Vec3 box_s = Vec3(1, 1, 1);
	std::vector<Vec3> samples;
	std::unique_ptr<NeighborhoodSearcher> neighborhood_searcher = nullptr;
};

