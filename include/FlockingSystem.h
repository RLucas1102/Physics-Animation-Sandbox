//---------------------------------------------------------
//
//  Created by: Lucas Robenolt
//
//  Purpose: A class to hold particles for a flocking system.
//           There a specific functions like computing the
//           collision avoidance, etc.
//           that need to be added to a normal particle
//           system. This class inherits from ParticleSystem
//
//
//---------------------------------------------------------
#ifndef FLOCKING_SYSTEM_H
#define FLOCKING_SYSTEM_H

#include "ParticleSystem.h"
#include "Vector.h"
#include <vector>
#include <memory>

namespace pba {

    class FlockingSystem : public ParticleSystem {
        
        public:
            FlockingSystem(const std::string& nam = "FlockingDataNoName");
            ~FlockingSystem(){};

            // Getter and setter methods
            double GetKca() { return _Kca; }
            double GetKm() { return _Km; }
            double GetKc() { return _Kc; }
            double GetR() { return _R; }
            double GetRamp() { return _Ramp; }
            double GetTheta() { return _theta; }
            double GetThetaRamp() { return _theta_ramp; }

            void SetKca(double Kca) { _Kca = Kca; }
            void SetKm(double Km) { _Km = Km; }
            void SetKc(double Kc) { _Kc = Kc; }
            void SetR(double R) { _R = R; }
            void SetRamp(double Ramp) { _Ramp = Ramp; }
            void SetTheta(double theta) { _theta = theta; }
            void SetThetaRamp(double theta_ramp) { _theta_ramp = theta_ramp; }

            // Find distance between two boids
            Vector ComputeDistance(const Vector& a, const Vector& b);

            // Find the range limit based on the distance between two particles
            double ComputeRangeLimit(const Vector& a, const Vector& b);

            // Find the fov limit based on the distance between two particles
            double ComputeFOVLimit(const Vector& a, const Vector& aVel, const Vector& b);

            

        private:
            std::string name;

            double _Kca;       // Collision avoidance multiplier
            double _Km;        // Velocity matching multiplier
            double _Kc;        // Centering multiplier
            double _R;         // Distance from boid range lim. begins to ramp
            double _Ramp;      // Distance from boid range lim. begins to ramp to zero
            double _theta;     // Angular FOV for boid vision
            double _theta_ramp; // Angular range ramping vision to zero

            std::vector<Vector> Distances;
            std::vector<size_t> Candidate_Boids;
            std::vector<double> Range_Limiters;
            std::vector<double> FOV_Limiters;

    };
}


#endif