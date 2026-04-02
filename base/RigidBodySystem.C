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
    
    // Setup linear velocity
    Vector initVel = Vector(0,0,0);
    for (size_t p = 0; p < Psize(); p++)
    {
        initVel += GetVel(p);   
    }

    initVel.normalize();

    _linearVel = initVel;

    // Setup rotation matrix
    _angularRot = unitMatrix();

    // Setup angular velocity
    _angularVel = pba::Vector(drand48() * 2 - 1, drand48() * 2 - 1, drand48() * 2 - 1);

    // Setup angular momentum
    RecomputeMOI();
    _angularMom = _momentOfInertia * _angularVel;
    
}

void RigidBodySystem::ComputeM() {

}

void RigidBodySystem::RecomputeMOI() {
    Matrix delta = unitMatrix();

    for (size_t p = 0; p < Psize(); p++)
    {
        double mass = GetMass(p);
        Vector pos  = GetLeverArm(p);

        for (size_t i = 0; i < 3; i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                double value = _momentOfInertia(i,j) + (mass * (delta(i,j) * pos.magnitude() - pos[i] * pos[j]));
                _momentOfInertia.Set(i, j, value);
            }
            
        }
        
    }

    _inverseMomentOfInertia = inverse(_momentOfInertia);
    
}

Vector RigidBodySystem::RBD_pos( const size_t p ) {
    Vector pos = GetLeverArm(p);
    Vector P = _angularRot * pos + _COM;
    return P;
}

RigidBody pba::CreateRigidBody(const std::string& nam) {
    return std::make_shared<RigidBodySystem>(RigidBodySystem(nam));
}

