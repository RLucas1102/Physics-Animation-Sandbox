#include "ParticleSystem.h"

ParticleSystem::ParticleSystem(const std::string &nam) :
    name (nam)
    {}

PSYS CreateParticleSystem(const std::string& nam) {
    PSYS outSys = PSYS( new ParticleSystem(nam) );
    std::cout << "ParticleSystem Created: " << outSys->Name() << std::endl;    
    return outSys;
}