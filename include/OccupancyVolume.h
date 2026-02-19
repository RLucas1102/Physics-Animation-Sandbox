#ifndef OCCUPANCY_VOLUME_H
#define OCCUPANCY_VOLUME_H

#include <vector>

#include "Vector.h"
#include "ParticleSystem.h"

namespace pba {
    
    class OccupancyVolume {
        public: 
            OccupancyVolume(const Vector& llc, const Vector& urc, const int R);
            ~OccupancyVolume(){};

            void Populate(const PSYS& psys);

            int Get_idx(int i, int j, int k);

            void GetNeighbors();

        private:
            Vector _llc;
            Vector _urc;
            Vector _Lxyz;
            Vector _nxyz;
            int _R;
            int _cellsize;

            std::vector<std::vector<size_t>> contents;
            std::vector<std::vector<size_t>> neighbors;

            std::vector<Vector> offsets = {
                Vector(-1, -1, -1), Vector(-1, -1,  0), Vector(-1, -1,  1),
                Vector(-1,  0, -1), Vector(-1,  0,  0), Vector(-1,  0,  1),
                Vector(-1,  1, -1), Vector(-1,  1,  0), Vector(-1,  1,  1),

                Vector( 0, -1, -1), Vector( 0, -1,  0), Vector( 0, -1,  1),
                Vector( 0,  0, -1), Vector( 1,  1,  1), Vector( 0,  0,  1),
                Vector( 0,  1, -1), Vector( 0,  1,  0), Vector( 0,  1,  1),

                Vector( 1, -1, -1), Vector( 1, -1,  0), Vector( 1, -1,  1),
                Vector( 1,  0, -1), Vector( 1,  0,  0), Vector( 1,  0,  1),
                Vector( 1,  1, -1), Vector( 1,  1,  0)
            };

    };

}


#endif