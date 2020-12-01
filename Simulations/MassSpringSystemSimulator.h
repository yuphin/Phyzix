#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <wrl/client.h>

using namespace Microsoft::WRL;

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

struct MassPoint {
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	bool is_fixed;
	Vec3 uv;
	bool colliding = false;

};

struct Spring {
	int mp1, mp2;
	float initial_length;
};

struct MassPointVertex {
	DirectX::XMFLOAT3 pos;
	DirectX::XMFLOAT2 uv;
	DirectX::XMFLOAT3 vel;
	DirectX::XMFLOAT3 tmp_pos;
	DirectX::XMFLOAT3 f2;
	uint32_t is_fixed;
};

struct ClothCB {
	DirectX::XMFLOAT4X4 model;
	DirectX::XMFLOAT4X4 view;
	DirectX::XMFLOAT4X4 projection;
	DirectX::XMFLOAT4X4 mvp;
};

struct StateCB {
	DirectX::XMFLOAT3 sphere_pos;
	float delta;
	DirectX::XMUINT2 grid_dim;
	float sphere_radius;
	float mass;
	float stiffness;
	float damping;
	float initial_len;
	uint32_t num_cloths;
	DirectX::XMFLOAT3 external_force;
	float cube_radius;
	DirectX::XMFLOAT3 cube_pos;
	uint32_t integrator;
	float cross_len;
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 velocity, bool is_fixed);

	void addSpring(int masspoint1, int masspoint2, float initial_length);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	inline void applyExternalForce(Vec3 force) { external_force += force; };
	inline float clamp(float value, float low, float high) { 
		return (value < low) ? low : (high < value) ? high : value; 
	}
	inline Vec3 clamp(const Vec3& value, float low , float high) {
		auto x = clamp(value.x, low, high);
		auto y = clamp(value.y, low, high);
		auto z = clamp(value.z, low, high);
		return Vec3(x,y,z);
	}
	template <typename T> 
	int sign(T val) {
		return (T(0) < val) - (val < T(0));
	}

	void passTimestepVariable(float& time_step);

	// Do Not Change
	void setIntegrator(int integrator) {
		this->integrator = integrator;
	}

	void init_resources(ID3D11Device* device);
	void fill_resources();

	~MassSpringSystemSimulator();

private:
	// Custom functions
	void compute_elastic_force(const Spring&);
	void initScene();
	void update_vertex_data();
	void update_vertex_extended();
	void compute_collision();
	// Data Attributes
	float mass;
	float stiffness;
	float damping;
	int integrator;

	// Maybe we can change these through UI?
	int GRIDX = 40;
	int GRIDY = 40;
	const int NUM_THREADS_X = 20;
	const int NUM_THREADS_Y = 20;
	const int  NUM_CLOTHS = 3;
	// UI Attributes
	Vec3 external_force;
	Point2D mouse;
	Point2D trackmouse;
	Point2D old_trackmouse;

	Vec3 mouse_force;

	float* timestep;
	bool running;
	bool update_vertex = true;
	bool enable_collision = false;

	// Custom attributes
	std::vector<MassPoint> mass_points;
	std::vector<Spring> springs;
	std::vector<MassPointVertex> vertices;

	//Midpoint related attributes
	std::vector<Vec3> old_positions;
	std::vector<Vec3> old_velocities;
	
	uint32_t index_count;
	Vec3 sphere_pos = Vec3(0.0, 0.0, -0.25);
	Vec3 cube_pos = Vec3(-1.25, 0.0, 0.75);
	float sphere_rad = 0.35;
	float cube_rad = 0.3;

	ComPtr<ID3D11Buffer> vertex_buffer;
	ComPtr<ID3D11Buffer> index_buffer;
	ComPtr<ID3D11VertexShader> vertex_shader;
	ComPtr<ID3D11InputLayout> input_layout;
	ComPtr<ID3D11PixelShader> pixel_shader;
	ComPtr<ID3D11Buffer> cloth_buffer;
	ComPtr<ID3D11Buffer> simulation_buffer;
	ComPtr<ID3D11RasterizerState> rasterizer_old;
	ComPtr<ID3D11RasterizerState> rasterizer_grid;
	ComPtr<ID3D11ComputeShader> compute_shader;
	ComPtr<ID3D11Buffer> buffer_in;
	ComPtr<ID3D11Buffer> buffer_out;
	ID3D11ShaderResourceView* texture_view;
	ID3D11SamplerState* sampler_state;
	ID3D11ShaderResourceView* srv_in = nullptr;
	ID3D11ShaderResourceView* srv_out = nullptr;
	ID3D11UnorderedAccessView* uav_in = nullptr;
	ID3D11UnorderedAccessView* uav_out = nullptr;
	ClothCB cloth_cb;
	StateCB simulation_cb;
};
#endif