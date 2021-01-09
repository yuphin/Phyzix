#ifndef TEMPLATESIMULATOR_h
#define TEMPLATESIMULATOR_h

#include "Simulator.h"

class TemplateSimulator:public Simulator{
public:
	// Construtors
	TemplateSimulator();

	// Functions
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
	void drawSomeRandomObjects();
	void drawMovableTeapot();
	void drawTriangle();

private:
	// Attributes
	Vec3  movable_obj_pos;
	Vec3  movable_obj_final_pos;
	Vec3  rotate;
	int   m_iNumSpheres;
	float m_fSphereSize;
	Point2D mouse;
	Point2D track_mouse;
	Point2D old_track_mouse;
};

#endif