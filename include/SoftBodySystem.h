//---------------------------------------------------------
//
//  Created by: Lucas Robenolt
//
//  Purpose: A class to hold particles for a soft body
//           system. There a specific functions like pairs
//           that need to be added to a normal particle
//           system. This class inherits from ParticleSystem
//
//
//---------------------------------------------------------
#ifndef SOFT_BODY_SYSTEM_H
#define SOFT_BODY_SYSTEM_H

#include "ParticleSystem.h"
#include "Edge.h"
#include <vector>
#include <memory>

namespace pba {
    
    class SoftBodySystem : public ParticleSystem {

        public:
            SoftBodySystem(const std::string& nam = "SoftBodyDataNoName");
            ~SoftBodySystem(){};

            // Get a pair of particles that are connected and return the edge data
            const SoftEdge& GetConnectedPair( size_t p ) const { return _connected_pairs[p]; }

            // Return how many pairs there are
            size_t Pairs() const { return _connected_pairs.size(); }

            // Return if pairs exist within the member vector
            bool ArePairs() const { return _connected_pairs.empty(); }

            // Remove all pairs
            void ClearPairs() { _connected_pairs.clear(); }

            // Connect two particles together by an edge and add it to the member vector
            void AddPair( size_t i, size_t j );

            // For each particle in the system, create its corresponding edges with particles
            void CreatePairs();

        private:
            const std::string name;

            std::vector<SoftEdge> _connected_pairs;

    };

    typedef std::shared_ptr<SoftBodySystem> SoftBody;

    SoftBody CreateSoftBody(const std::string& nam = "SoftBodyDataNoName");

}

#endif