#include "SPHSystem.h"

using namespace pba;

SPHSystem::SPHSystem(const std::string &nam) :
    ParticleSystem (nam)
    {}

double const SPHSystem::GetRho(const size_t p) {
    return particles[p].density;
}

double const SPHSystem::GetH() {
    return h;
}

void SPHSystem::SetRho(const size_t p, double inDensity) {
    particles[p].density = inDensity;
}

void SPHSystem::SetH(const double inH) {
    h = inH;
}

SPH pba::CreateSPHSystem(const std::string &nam) {
    SPH psys = std::make_shared<SPHSystem>(SPHSystem(nam));
    std::cout << "Created SPH system " << psys->Name() << "\n";
    return psys;
}