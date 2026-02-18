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

            void populate(const PSYS& psys);

        private:
            Vector _llc;
            Vector _urc;
            Vector _Lxyz;
            Vector _nxyz;
            int _R;
            int _cellsize;

            std::vector<std::vector<size_t>> contents;
            std::mdspan volume3D;

    };

}


#endif