#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include "Vector.h"
#include "Color.h"
#include <string>
#include <memory>
#include <iostream>

class ParticleSystem {
    public:
        ParticleSystem (const std::string& nam = "ParticleSysNoName");

        ~ParticleSystem() {};

        const std::string& Name() const {return name;}

    private:
        const std::string name;
};

typedef std::shared_ptr<ParticleSystem> PSYS;

PSYS CreateParticleSystem(const std::string& nam = "ParticleSysNoName");

#endif