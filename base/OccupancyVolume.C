#include "OccupancyVolume.h"
#include "Vector.h"
#include <vector>

using namespace pba;

OccupancyVolume::OccupancyVolume(const Vector& llc, const Vector& urc, const int R) :
    _llc (llc),
    _urc (urc),
    _R (R)
    {
        _Lxyz = _urc - _llc;
        _cellsize = 2*R;
        _nxyz = Vector(int(_Lxyz[0]/_cellsize) + 1, int(_Lxyz[1]/_cellsize) + 1, int(_Lxyz[2])/_cellsize + 1);
        contents = std::vector<std::vector<size_t>>(_nxyz[0] * _nxyz[1] * _nxyz[2]); 
    }