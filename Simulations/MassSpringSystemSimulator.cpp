#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
    m_iTestCase = 0;
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
    return "Demo 2, Demo 3, Demo 4, Demo 5";
}

void MassSpringSystemSimulator::reset() {
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

    mass_points.clear();
    springs.clear();
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
    this->DUC = DUC;
    
    TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "step=0.01 min=0.0001");
    TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "step=0.1 min=0.0001");
    TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.01 min=0.0001");

    TwType TW_TYPE_INTEGRATORTYPE = TwDefineEnumFromString("IntegrationType", "EULER,LEAPFROG,MIDPOINT");
    TwAddVarRW(DUC->g_pTweakBar, "IntegrationType", TW_TYPE_INTEGRATORTYPE, &m_iIntegrator, "");
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    switch (m_iTestCase)
    {
    case 0:
        cout << "Demo 2 !\n";
        break;
    case 1:
        cout << "Demo 3 !\n";
        break;
    case 2:
        cout << "Demo 4 !\n";
        break;
    default:
        cout << "Demo 5 !\n";
        break;
    }
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
    Point2D mouseDiff;
    mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
    mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
    if (mouseDiff.x != 0 || mouseDiff.y != 0)
    {
        Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
        worldViewInv = worldViewInv.inverse();
        Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
        Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
        // find a proper scale!
        float inputScale = 0.001f;
        inputWorld = inputWorld * inputScale;
        
        for (auto& masspoint : mass_points) {
            masspoint.force += inputWorld;
        }
    }
}

void MassSpringSystemSimulator::onClick(int x, int y) {
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y) {
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass) {
    m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
    m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
    m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
    MassPoint mp = {};
    mp.position = position;
    mp.velocity = Velocity;
    mp.is_fixed = isFixed;
    mass_points.emplace_back(mp);
    return mass_points.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
    springs.push_back({ masspoint1, masspoint2, initialLength });
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
    return mass_points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
    return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
    return mass_points[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
    return mass_points[index].velocity;
}
