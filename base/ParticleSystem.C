#include "ParticleSystem.h"

using namespace pba;

ParticleSystem::ParticleSystem(const std::string &nam) :
    name (nam)
    {}

void ParticleSystem::AddParticle() {
    particles.push_back(ParticleState());
}

void ParticleSystem::AddParticles(const size_t p) {
    for (size_t i = 0; i < p; i++) {
        particles.push_back(ParticleState());
    }
    
}

Vector const ParticleSystem::GetPos(const size_t p) {
    return particles[p].position;
}

Vector const ParticleSystem::GetVel(const size_t p) {
    return particles[p].velocity; 
}

Color const ParticleSystem::GetCol(const size_t p) {
    return particles[p].color;    
}

Vector const ParticleSystem::GetAcc(const size_t p) {
    return particles[p].acceleration;
}

double const ParticleSystem::GetMass(const size_t p) {
    return particles[p].mass;
}

void ParticleSystem::SetPos(const size_t p, Vector& inPos) {
    particles[p].position = inPos;
}

void ParticleSystem::SetVel(const size_t p, Vector& inVel) {
    particles[p].velocity = inVel;
}

void ParticleSystem::SetCol(const size_t p, Color& inCol) {
    particles[p].color = inCol;
}

void ParticleSystem::SetAcc(const size_t p, Vector& inAcc) {
    particles[p].acceleration = inAcc;
}

void ParticleSystem::SetMass(const size_t p, double inMass) {
    particles[p].mass = inMass;
}

size_t const ParticleSystem::Psize() {
    return particles.size();
} 

void ParticleSystem::Pclear() {
    particles.clear();
}

PSYS pba::CreateParticleSystem(const std::string& nam) {
    PSYS outSys = PSYS( new ParticleSystem(nam) );
    std::cout << "ParticleSystem Created: " << outSys->Name() << std::endl;    
    return outSys;
}
