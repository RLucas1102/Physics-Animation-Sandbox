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

void OccupancyVolume::Populate(const PSYS& psys) {
    for (size_t num = 0; num < psys->Psize(); num++) {
        Vector P = psys->GetPos(num);
        Vector y = P - _llc;
        Vector w = y / _cellsize;
        int i = int(w[0]);
        int j = int(w[1]);
        int k = int(w[2]);

        int index = Get_idx(i, j, k);
        
        contents[index].push_back(num);
    }
    
}

int OccupancyVolume::Get_idx(int i, int j, int k) {
    return i + _nxyz[0] * (j + (_nxyz[1] * k));
}

void OccupancyVolume::GetNeighbors() {
    for (size_t i = 0; i < _nxyz[0]; i++) {
        for (size_t j = 0; j < _nxyz[1]; j++) {
            for (size_t k = 0; k < _nxyz[2]; k++) {
                int saveIndex = Get_idx(i, j, k);
                neighbors[saveIndex].push_back(saveIndex);

                Vector cellPos = Vector(i, j, k);
                for (Vector offset : offsets) {
                    Vector tempCellPos = cellPos + offset;

                    int x = tempCellPos[0];
                    int y = tempCellPos[1];
                    int z = tempCellPos[2];

                    if (x > -1 && x < _nxyz[0]) {
                        if (y > -1 && y < _nxyz[1]) {
                            if (z > -1 && z < _nxyz[2]) {
                                int tempIndex = Get_idx(x, y, z);
                                neighbors[saveIndex].push_back(tempIndex);
                            }
                        }
                    }
                }
            }
        }
    }
}