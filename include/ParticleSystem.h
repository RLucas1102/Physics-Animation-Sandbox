//---------------------------------------------------------
//
//  Created by: Lucas Robenolt
//
//  Purpose: A class to hold particles for a certain system
//  and useful functions to interact with the system
//
//
//---------------------------------------------------------

#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include "Vector.h"
#include "Color.h"
#include <string>
#include <memory>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

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
    //
    // Adapted from Jerry Tessendorf
    ////////////////////////////////////////////////
    class ParticleState {
        public:
            ParticleState() :
                position(Vector(0,0,0)),
                velocity(Vector(0,0,0)),
                color(Color(1,1,1,1)),
                mass(1.0),
                density(1.0)
                {};
        
            ~ParticleState(){};	   

            Vector position;
            Vector velocity;
            Vector acceleration;
            Color color;
            double mass;
            double density;
    };

    class ParticleSystem {
        public:
        
            ParticleSystem (const std::string& nam = "ParticleSysNoName");

            virtual ~ParticleSystem() {};

            // Returns the name of particle system, if needed
            const std::string& Name() const {return name;}

            // Add a single particle to the system
            void AddParticle();

            // Add many particles to the system
            void AddParticles(const size_t p);

            // Getter functions to obtain specific particle attributes
            Vector const GetPos(const size_t p);

            Vector const GetVel(const size_t p);

            Color const GetCol(const size_t p);

            Vector const GetAcc(const size_t p);

            double const GetMass(const size_t p);

            // Setter functions to change specific particle attributes
            void SetPos(const size_t p, Vector& inPos);

            void SetVel(const size_t p, Vector& inVel);

            void SetCol(const size_t p, Color& inCol);

            void SetAcc(const size_t p, Vector& inAcc);

            void SetMass(const size_t p, double inMass);

            // Returns the number of particles in the system
            size_t const Psize();

            // Load particles based on a model
            void GenParticlesFromModel(const char* FilePath);

            // Clears all particles from the system
            void Pclear();

        protected:
            const std::string name;

            // This is all of the particles in the system
            std::vector<ParticleState> particles;
            std::vector<ParticleState>::iterator particles_it;

    };

    typedef std::shared_ptr<ParticleSystem> PSYS;

    // Create a shared pointer to particle system
    PSYS CreateParticleSystem(const std::string& nam = "ParticleSysNoName");

}

#endif