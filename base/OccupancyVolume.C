#include "OccupancyVolume.h"
#include "Vector.h"
#include "ParticleSystem.h"

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

void OccupancyVolume::populate(const PSYS& psys) {
    for (size_t num = 0; num < psys->Psize(); num++) {
        Vector P = psys->GetPos(num);
        Vector y = P - _llc;
        Vector w = y / _cellsize;
        int i = int(w[0]);
        int j = int(w[1]);
        int k = int(w[2]);

        int index = i + _nxyz[0] * (j + (_nxyz[1] * k));

        contents[index].push_back(num);
    }
    
}