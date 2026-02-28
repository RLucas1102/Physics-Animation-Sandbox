#include "SoftBodyState.h"
#include "Edge.h"

#include <memory>

using namespace pba;

SoftBodyStateData::SoftBodyStateData(const std::string &nam) :
    name (nam)
    {}

void SoftBodyStateData::AddPair(size_t i, size_t j) {

}

SoftBodyState pba::CreateSoftBody(const std::string &nam) {
    return std::make_shared<SoftBodyStateData>(SoftBodyStateData(nam));
}