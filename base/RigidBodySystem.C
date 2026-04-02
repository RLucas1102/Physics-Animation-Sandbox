#include "RigidBodySystem.h"
#include "Vector.h"
#include "Matrix.h"

using namespace pba;

RigidBodySystem::RigidBodySystem(const std::string& nam) :
    name (nam)
    {}

void RigidBodySystem::ComputeRBDData() {

    _COM = Vector(0,0,0);
    _totalMass = 0.0;

    for (size_t p = 0; p < Psize(); p++)
    {
        Vector pos = GetPos(p);
        double mass = GetMass(p);

        _totalMass += mass;
        _COM += pos*mass;
    }

    _COM /= _totalMass;

    for (size_t p = 0; p < Psize(); p++)
    {
        Vector pos = GetPos(p);
        Vector d = pos - _COM;
        SetLeverArm(p, d);
    }
    
}

void RigidBodySystem::ComputeM() {

}

void RigidBodySystem::RecomputeMOI() {

}

Vector RigidBodySystem::RBD_pos( const size_t p ) {
    Vector pos = GetLeverArm(p);
    Vector P = pos + _COM;
    return P;
}

RigidBody CreateRigidBody(const std::string& nam) {
    return std::make_shared<RigidBodySystem>(RigidBodySystem(nam));
}

