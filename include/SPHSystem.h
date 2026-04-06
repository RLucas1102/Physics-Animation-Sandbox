//---------------------------------------------------------
//
//  Created by: Lucas Robenolt
//
//  Purpose: A class to hold particles for a certain system
//  and useful functions to interact with the system
//
//
//---------------------------------------------------------

#ifndef SPH_SYSTEM_H
#define SPH_SYSTEM_H

#include "Vector.h"
#include "Color.h"
#include "ParticleSystem.h"
#include <string>
#include <memory>
#include <iostream>
#include <vector>

namespace pba {

    class SPHSystem : public ParticleSystem {
        public:
        
            SPHSystem (const std::string& nam = "SPHSysNoName");

            ~SPHSystem(){};

            // Getter functions to obtain specific particle attributes
            double const GetRho(const size_t p);

            double const GetH();

            // Setter functions to change specific particle attributes
            void SetRho(const size_t p, double inDensity);

            void SetH(const double inH);

        private:

            // Radius of influence
            double h;
    };

    typedef std::shared_ptr<SPHSystem> SPH;

    // Create a shared pointer to particle system
    SPH CreateSPHSystem(const std::string& nam = "SPHSysNoName");

}

#endif