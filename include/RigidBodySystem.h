//---------------------------------------------------------
//
//  Created by: Lucas Robenolt
//
//  Purpose: A class to hold particles for a rigid body
//           system. There a specific functions
//           that need to be added to a normal particle
//           system. This class inherits from ParticleSystem
//
//
//---------------------------------------------------------
#ifndef RIGID_BODY_SYSTEM_H
#define RIGID_BODY_SYSTEM_H

#include "ParticleSystem.h"
#include "Matrix.h"
#include "LinearAlgebra.h"
#include <vector>
#include <memory>

namespace pba {

    class RigidBodySystem : public ParticleSystem {

        public:
            RigidBodySystem(const std::string& nam = "RigidBodyDataNoName");
            ~RigidBodySystem(){};

            void ComputeRBDData(); // Initialize COM, total mass, home state
            void ComputeM();
            void RecomputeMOI();

            const Matrix& InertiaMoment() const { return _momentOfInertia; }
            const Matrix& InverseMOI() const { return _inverseMomentOfInertia; }
            const float TotalMass() const { return _totalMass; }

            // Position of a vertex in current, rotated state
            Vector RBD_pos( const size_t p );

            Vector GetLeverArm( const size_t p) { return LeverArms[p]; }
            void SetLeverArm( const size_t p, const Vector leverArm) { LeverArms.push_back(leverArm); }

            Vector _COM;            // Center of mass position
            Matrix _angularRot;     // Rotation matrix R
            Vector _linearVel;
            Vector _angularVel;
            Vector _COMAcc;
            Vector _angularAcc;
            Vector _angularMom;

        private:
            const std::string name;

            std::vector<Vector> LeverArms;
            Matrix _momentOfInertia;
            Matrix _inverseMomentOfInertia;
            float _totalMass;

    };

    typedef std::shared_ptr<RigidBodySystem> RigidBody;

    RigidBody CreateRigidBody(const std::string& nam = "RigidBodyDataNoName");

}

#endif