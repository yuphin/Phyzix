#include "TemplateSimulator.h"

TemplateSimulator::TemplateSimulator()
{
	m_iTestCase = 0;
	movable_obj_pos = Vec3();
	movable_obj_final_pos = Vec3();
	rotate = Vec3();
	m_iNumSpheres    = 100;
	m_fSphereSize    = 0.05f;
}

const char * TemplateSimulator::getTestCasesStr(){
	return "Teapot,Random Objects,Triangle";
}

void TemplateSimulator::reset(){
		mouse.x = mouse.y = 0;
		track_mouse.x = track_mouse.y = 0;
		old_track_mouse.x = old_track_mouse.y = 0;
}

void TemplateSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:
		TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		break;
	case 2:break;
	default:break;
	}
}

void TemplateSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Teapot !\n";
		movable_obj_pos = Vec3(0, 0, 0);
		rotate = Vec3(0, 0, 0);
		break;
	case 1:
		cout << "Random Object!\n";
		m_iNumSpheres = 100;
		m_fSphereSize = 0.05f;
		break;
	case 2:
		cout << "Triangle !\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void TemplateSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = track_mouse.x - old_track_mouse.x;
	mouseDiff.y = track_mouse.y - old_track_mouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		movable_obj_pos = movable_obj_final_pos + inputWorld;
	}
	else {
		movable_obj_final_pos = movable_obj_pos;
	}
}

void TemplateSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0:
		// rotate the teapot
		rotate.x += timeStep;
		if (rotate.x > 2 * M_PI) rotate.x -= 2.0f * (float)M_PI;
		rotate.y += timeStep;
		if (rotate.y > 2 * M_PI) rotate.y -= 2.0f * (float)M_PI;
		rotate.z += timeStep;
		if (rotate.z > 2 * M_PI) rotate.z -= 2.0f * (float)M_PI;

		break;
	default:
		break;
	}
}

void TemplateSimulator::drawSomeRandomObjects()
{
    std::mt19937 eng;
    std::uniform_real_distribution<float> randCol( 0.0f, 1.0f);
    std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
    for (int i=0; i<m_iNumSpheres; i++)
    {
		DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,0.6*Vec3(randCol(eng),randCol(eng), randCol(eng)));
		DUC->drawSphere(Vec3(randPos(eng),randPos(eng),randPos(eng)),Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
    }
}

void TemplateSimulator::drawMovableTeapot()
{
	DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,0.6*Vec3(0.97,0.86,1));
	DUC->drawTeapot(movable_obj_pos,rotate,Vec3(0.5,0.5,0.5));
}

void TemplateSimulator::drawTriangle()
{
	DUC->DrawTriangleUsingShaders();
}

void TemplateSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch(m_iTestCase)
	{
	case 0: drawMovableTeapot();break;
	case 1: drawSomeRandomObjects();break;
	case 2: drawTriangle();break;
	}
}

void TemplateSimulator::onClick(int x, int y)
{
	track_mouse.x = x;
	track_mouse.y = y;
}

void TemplateSimulator::onMouse(int x, int y)
{
	old_track_mouse.x = x;
	old_track_mouse.y = y;
	track_mouse.x = x;
	track_mouse.y = y;
}
