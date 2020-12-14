#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "RigidBody.h"
#include "util/util.h"
#include "collisionDetect.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator :public Simulator {
public:
	// Construtors
	RigidBodySystemSimulator();

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
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void add_sphere(const Vec3& pos, float radius, int mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	void handle_collisions();
	void add_torque(int i, Vec3 ang_velocity);
	void pass_time_step_variable(float& time_step);
	static void TW_CALL RigidBodySystemSimulator::getGravity(void* value, void* clientData);
	static void TW_CALL RigidBodySystemSimulator::setGravity(const void* value, void* clientData);
	static void TW_CALL RigidBodySystemSimulator::addBox(void* value);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	void resolve_positions(CollisionData& data);
	void resolve_velocities(CollisionData& data, Contact* best_col, const std::vector<RigidBody*>& pairs);
	void calc_after_col_vel(Contact* collision_info, float delta_vel, const std::vector<RigidBody*>& pairs);

	Vec3 m_externalForce;
	Vec3 mouse_force;
	Vec3 gravity;
	double bounciness = 0.6;
	
	// UI Attributes
	Point2D mouse;
	Point2D trackmouse;
	Point2D old_trackmouse;

	std::vector<RigidBody> rigid_bodies;
	Plane plane;
	DrawingUtilitiesClass* DUC;
	float* timestep;
	bool running;

};
#endif
