#include "Edge.h"

using namespace pba;

SoftEdgeData::SoftEdgeData(const size_t i, const size_t j, const double edgeLength) :
    inode (i),
    jnode (j),
    length (edgeLength)
    {}

SoftEdge pba::CreateSoftEdge(const size_t i, const size_t j, const double edgeLength) {
    return std::make_shared<SoftEdgeData>(SoftEdgeData(i, j, edgeLength));
}