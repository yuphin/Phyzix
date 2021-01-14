#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"
#include "pcgsolver.h"

//Implement your own grid class for saving grid data
struct Grid {
	// Constructors
	Grid(int dim_x, bool is_3d = false);
	std::vector<int> get_internal_repr_idxs();
	std::vector<Real> values;
	std::vector<Vec3> positions;
	std::vector<Vec3> pos_internal;
	int num_points, dim_x;
private:
	bool is_3d;
};



class DiffusionSimulator:public Simulator{
public:
	// Constructors
	DiffusionSimulator(bool adaptive_step = false);

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC) override;
	void reset() override;
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext) override;
	void notifyCaseChanged(int testCase) override;
	void simulateTimestep(float timeStep) override;
	void externalForcesCalculations(float timeElapsed) override;
	void onClick(int x, int y) override;
	void onMouse(int x, int y) override;
	// Specific Functions
	void draw_objects();
	Grid* solve_explicit(float time_step);
	void solve_implicit(float time_step);
	void pass_time_step_variable(float time_step);

private:
	void init_grid();
	// Attributes
	// Assume uniform grid for now
	int dim_size = 10;
	Real grid_size = 1;
	double alpha = 1;
	Vec3  movable_obj_pos;
	Vec3  movable_obj_final_pos;
	Vec3  rotate;
	Point2D mouse;
	Point2D track_mouse;
	Point2D old_track_mouse;
	std::unique_ptr<Grid> grid = nullptr;
	std::unique_ptr<SparseMatrix<Real>> A = nullptr;
	float time_step;
	bool adaptive_step;
	bool is_3d = false;
};

#endif