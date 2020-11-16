#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

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
};

struct Spring {
	int mp1, mp2;
	float initial_length;
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
	
	// Do Not Change
	void setIntegrator(int integrator) {
		this->integrator = integrator;
	}

private:
	// Custom functions
	void compute_elastic_force(const Spring&);

	// Data Attributes
	float mass;
	float stiffness;
	float damping;
	int integrator;

	// UI Attributes
	Vec3 external_force;
	Point2D mouse;
	Point2D trackmouse;
	Point2D old_trackmouse;

	// Custom attributes
	std::vector<MassPoint> mass_points;
	std::vector<Spring> springs;
};
#endif