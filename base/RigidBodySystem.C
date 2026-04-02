#include "RigidBodySystem.h"

using namespace pba;

RigidBodySystem::RigidBodySystem(const std::string& nam) :
    name (nam)
    {}

void RigidBodySystem::ComputeRBDData() {

}

void RigidBodySystem::ComputeM() {

}

void RigidBodySystem::RecomputeMOI() {

}

RigidBody CreateRigidBody(const std::string& nam) {
    return std::make_shared<RigidBodySystem>(RigidBodySystem(nam));
}

