#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include "Vector.h"
#include "Color.h"
#include <string>
#include <memory>
#include <iostream>
#include <vector>

namespace pba {

    ////////////////////////////////////////////////
    //
    //      PARTICLE STATE
    //
    // The state of a particle is characterized by
    // (1) particle positions
    // (2) particle velocities
    // (3) particle masses
    // (4) particle colors - useful for display
    ////////////////////////////////////////////////
    class ParticleState {
        public:
            ParticleState() :
                position(Vector(0,0,0)),
                velocity(Vector(0,0,0)),
                color(Color(1,1,1,1)),
                mass(1.0)
                {};
        
            ~ParticleState(){};	   

            Vector position;
            Vector velocity;
            Color color;
            float mass;
    };

    class ParticleSystem {
        public:
            ParticleSystem (const std::string& nam = "ParticleSysNoName");

            ~ParticleSystem() {};

            const std::string& Name() const {return name;}

            void AddParticle();

            void AddParticles(const size_t p);

            ParticleState GetParticle(const size_t p);

            size_t const Psize();

            void Pclear();

            void const ShowParticles();

        private:
            const std::string name;

            // This is all of the particles in the system
            std::vector<ParticleState> particles;
            std::vector<ParticleState>::iterator particles_it;
    };

    typedef std::shared_ptr<ParticleSystem> PSYS;

    PSYS CreateParticleSystem(const std::string& nam = "ParticleSysNoName");

}

#endif