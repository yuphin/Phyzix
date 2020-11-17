#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
    m_iTestCase = 0;
    // For debugging:
    /*this->mass = 10.0f;
    this->damping = 0.0f;
    this->stiffness = 40.0f;
    this->applyExternalForce(Vec3(0, 0, 0));
    int p0 = this->addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
    int p1 = this->addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
    this->addSpring(p0, p1, 1.0);
    this->setIntegrator(EULER);*/
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
    return "Demo 2, Demo 3, Demo 4, Demo 5";
}
 
void MassSpringSystemSimulator::reset() {
    mouse.x = mouse.y = 0;
    trackmouse.x = trackmouse.y = 0;
    old_trackmouse.x = old_trackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
    this->DUC = DUC;
    
    TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &mass, "step=0.01 min=0.0001");
    TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &stiffness, "step=0.1 min=0.0001");
    TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &damping, "step=0.01 min=0.0001");

    TwType TW_TYPE_INTEGRATORTYPE = TwDefineEnumFromString("IntegrationType", "EULER,LEAPFROG,MIDPOINT");
    TwAddVarRW(DUC->g_pTweakBar, "IntegrationType", TW_TYPE_INTEGRATORTYPE, &integrator, "");
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
    mouseDiff.x = trackmouse.x - old_trackmouse.x;
    mouseDiff.y = trackmouse.y - old_trackmouse.y;
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


void MassSpringSystemSimulator::compute_elastic_force(const Spring& s) {
    auto& mp1 = mass_points[s.mp1];
    auto& mp2 = mass_points[s.mp2];

    Vec3 spring_vec = mp1.position - mp2.position;
    float x = norm(spring_vec) - s.initial_length;
    normalize(spring_vec);
    Vec3 f = -stiffness * x * spring_vec;
    mp1.force += f - damping * mp1.velocity;
    mp2.force += -f  - damping * mp2.velocity;
}

void MassSpringSystemSimulator::simulateTimestep(float time_step) {
    // Clear forces + add external forces 
    // Compute elastic forces
    // Apply integrator

    for(auto& mp : mass_points) {
        mp.force = this->external_force;
    }
    for(const auto& spring : springs) {
        compute_elastic_force(spring);
    }

    switch(integrator) {
        case EULER:
        {
            for(auto& mp : mass_points) {
                Vec3 accel = mp.force / mass;
                mp.position = mp.position + time_step * mp.velocity;
                mp.velocity = mp.velocity + time_step * accel;
            }
            break;
        }
        case LEAPFROG:
        {
            for(auto& mp : mass_points) {
                Vec3 accel = mp.force / mass;
                // Not tested
                mp.velocity = mp.velocity + time_step * accel;
                mp.position = mp.position + time_step * mp.velocity;
            }
            break;
        }
        case MIDPOINT:
        {
            old_positions.clear();
            old_velocities.clear();

            for (auto& mp : mass_points) {
                Vec3 accel = mp.force / mass;

                old_positions.push_back(mp.position);
                old_velocities.push_back(mp.velocity);

                mp.position = mp.position + time_step / 2.0f * mp.velocity;
                mp.velocity = mp.velocity + time_step / 2.0f * accel;
            }

            for (auto& mp : mass_points) {
                mp.force = this->external_force;
            }

            for (const auto& spring : springs) {
                compute_elastic_force(spring);
            }

            for (int i = 0; i < mass_points.size(); i++) {
                Vec3 accel = mass_points[i].force / mass;

                mass_points[i].position = old_positions[i] + time_step * mass_points[i].velocity;
                mass_points[i].velocity = old_velocities[i] + time_step * accel;
            }

            break;
        }
        default:
            break;
    }

}

void MassSpringSystemSimulator::onClick(int x, int y) {
    trackmouse.x = x;
    trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y) {
    old_trackmouse.x = x;
    old_trackmouse.y = y;
    trackmouse.x = x;
    trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass) {
    this->mass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
    this->stiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
    this->damping = damping;
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
