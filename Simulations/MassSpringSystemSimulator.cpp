#include "MassSpringSystemSimulator.h"

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
