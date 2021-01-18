#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"
#include "pcgsolver.h"
#include <wrl/client.h>
using namespace Microsoft::WRL;
//Implement your own grid class for saving grid data
struct Grid {
	// Constructors
	Grid(int dim_x, int dim_y, int dim_z, bool is_3d = false);
	std::vector<Real> values;
	std::vector<Vec3> positions;
	std::vector<Vec3> pos_internal;
	std::vector<int> boundary_indices;
	int num_points, dim_x, dim_y, dim_z;
	// For GPU
	std::vector<float> vals_gpu;
	std::vector<Real> create_new();
private:
	std::vector<int> get_internal_repr_idxs();
	void set_boundary_indices(int dim_sqr, int dim_z);
	bool is_3d;
};

class DiffusionSimulator;

struct DiffusionCB {
	int N;
	int align[3];
};

struct ClientData {
	DiffusionSimulator* self;
	int* dim_size;
	int* dim_x;
	int* dim_y;
	int* dim_z;
	int* cg_iters;
	int* jacobi_iters;
};

class DiffusionSimulator:public Simulator{
public:
	// Constructors
	DiffusionSimulator(bool adaptive_step = false);
	~DiffusionSimulator();

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
	void init_resources(ID3D11Device* device);
	void fill_static_resources();
	void fill_dynamic_resources(const std::vector<Real>& x);
	static void TW_CALL set_dim_size(const void* value, void* client_data);
	static void TW_CALL set_dim_x(const void* value, void* client_data);
	static void TW_CALL set_dim_y(const void* value, void* client_data);
	static void TW_CALL set_dim_z(const void* value, void* client_data);
	static void TW_CALL get_dim_size(void* value, void* client_data);
	static void TW_CALL get_dim_x(void* value, void* client_data);
	static void TW_CALL get_dim_y(void* value, void* client_data);
	static void TW_CALL get_dim_z(void* value, void* client_data);
	static void TW_CALL set_cg_iters(const void* value, void* client_data);
	static void TW_CALL get_cg_iters(void* value, void* client_data);
	static void TW_CALL set_jacobi_iters(const void* value, void* client_data);
	static void TW_CALL get_jacobi_iters(void* value, void* client_data);
private:
	ID3D11DeviceContext* context;
	ID3D11Device* device;
	void init_grid();
	void setup_for_implicit();
	void setup_for_jacobi(const FixedSparseMatrix<Real>& mat);
	void free_resources();
	// Attributes
	// Assume uniform grid for now
	int dim_size = 6;
	int dim_x = 6;
	int dim_y = 6;
	int dim_z = 6;
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
	bool use_gpu = false;
	int num_jacobi_iters = 50;
	int num_cg_iters = 20;
	std::unique_ptr<ClientData> data;
	ComPtr<ID3D11ComputeShader> compute_shader;
	// Buffers
	ComPtr<ID3D11Buffer> rowstart_buf;
	ComPtr<ID3D11Buffer> colindex_buf;
	ComPtr<ID3D11Buffer> mat_values_buf;
	ComPtr<ID3D11Buffer> rhs_buf;
	ComPtr<ID3D11Buffer> x_in_buf;
	ComPtr<ID3D11Buffer> x_out_buf;
	ComPtr<ID3D11Buffer> diffusion_cb;
	//

	// SRVs and UAVs
	ID3D11ShaderResourceView* rowstart_srv = nullptr;
	ID3D11ShaderResourceView* colindex_srv = nullptr;
	ID3D11ShaderResourceView* mat_values_srv = nullptr;
	ID3D11ShaderResourceView* rhs_srv = nullptr;
	ID3D11ShaderResourceView* x_in_srv = nullptr;
	ID3D11UnorderedAccessView* x_in_uav = nullptr;
	ID3D11ShaderResourceView* x_out_srv = nullptr;
	ID3D11UnorderedAccessView* x_out_uav = nullptr;
	//

	// For GPU
	std::unique_ptr<FixedSparseMatrix<float>> fsm = nullptr;
	std::vector<float> x_gpu;
	DiffusionCB cb;

};
#endif