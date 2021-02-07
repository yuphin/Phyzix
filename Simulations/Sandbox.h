#pragma once
#include "Simulator.h"
#include "util/util.h"
#include "collisionDetect.h"
#include <unordered_map>
#include "SPHSimulator.h"
class Sandbox : public Simulator {
public:
	// Construtors
	Sandbox();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void add_box(Vec3 position, Vec3 size, int mass);
	void add_sphere(const Vec3& pos, float radius, int mass);
	void add_plane(float offset, const Vec3& normal);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	void handle_collisions();
	void add_torque(int i, Vec3 ang_velocity);
	void pass_time_step_variable(float& time_step);
	static void TW_CALL Sandbox::get_gravity(void* value, void* clientData);
	static void TW_CALL Sandbox::set_gravity(const void* value, void* clientData);
	static void TW_CALL Sandbox::addRandomBox(void* value);
	static void TW_CALL Sandbox::addRandomSphere(void* value);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	void resolve_positions(CollisionData& data);
	void resolve_velocities(CollisionData& data, Contact* best_col, const std::vector<RigidBody*>& pairs);
	void calc_after_col_vel(Contact* collision_info, float delta_vel, const std::vector<RigidBody*>& pairs);
	void create_rb_boundaries(bool create = false);

	Vec3 m_externalForce;
	Vec3 mouse_force;
	Vec3 gravity;
	double bounciness = 0.4;

	// UI Attributes
	Point2D mouse;
	Point2D trackmouse;
	Point2D old_trackmouse;
	float sph_timestep;
	std::vector<RigidBody> rigid_bodies;
	Plane plane;
	DrawingUtilitiesClass* DUC;
	float* timestep;
	bool render_planes;
	std::unordered_map<uint32_t, Contact* (*)(
		RigidBody*, RigidBody*, Mat4&, CollisionData&)> collision_map;
	std::unique_ptr<SPHSimulator> sph = nullptr;
};

