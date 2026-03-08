#include "SoftBodySystem.h"
#include "Edge.h"

#include <memory>

using namespace pba;

SoftBodySystem::SoftBodySystem(const std::string &nam) :
    name (nam)
    {}

void SoftBodySystem::AddPair(size_t i, size_t j) {
    SoftEdge se = CreateSoftEdge(i, j, 0.5);
    _connected_pairs.push_back(se);
}

void SoftBodySystem::CreatePairs() {
    for (size_t i = 0; i < Psize(); i++) {
        for (size_t j = i+1; j < Psize(); j++) {
            AddPair(i, j);
        }
    }
}

SoftBody pba::CreateSoftBody(const std::string &nam) {
    return std::make_shared<SoftBodySystem>(SoftBodySystem(nam));
}