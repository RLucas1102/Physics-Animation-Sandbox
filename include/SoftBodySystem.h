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

            const SoftEdge& GetConnectedPair( size_t p ) const { return _connected_pairs[p]; }

            size_t Pairs() const { return _connected_pairs.size(); }

            bool ArePairs() const { return _connected_pairs.empty(); }

            void ClearPairs() { _connected_pairs.clear(); }

            void AddPair( size_t i, size_t j );

            private:
                const std::string name;

                std::vector<SoftEdge> _connected_pairs;

    };

    typedef std::shared_ptr<SoftBodySystem> SoftBody;

    SoftBody CreateSoftBody(const std::string& nam = "SoftBodyDataNoName");

}

#endif