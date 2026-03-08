#include "SoftBodySystem.h"
#include "Edge.h"

#include <memory>

using namespace pba;

SoftBodySystem::SoftBodySystem(const std::string &nam) :
    name (nam)
    {}

void SoftBodySystem::AddPair(size_t i, size_t j) {

}

SoftBody pba::CreateSoftBody(const std::string &nam) {
    return std::make_shared<SoftBodySystem>(SoftBodySystem(nam));
}