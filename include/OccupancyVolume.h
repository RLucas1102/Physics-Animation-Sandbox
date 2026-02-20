//---------------------------------------------------------
//
//  Created by: Lucas Robenolt
//
//  Purpose: A class to create an occupancy volume to
//           optimize the calculation of SPH forces and
//           density. Each grid cellcontains particles 
//           and has neighbors.
//
//---------------------------------------------------------

#ifndef OCCUPANCY_VOLUME_H
#define OCCUPANCY_VOLUME_H

#include <vector>
#include <memory>

#include "Vector.h"
#include "ParticleSystem.h"

namespace pba {
    
    class OccupancyVolume {
        public: 

            OccupancyVolume(const Vector& llc, const Vector& urc, const double R);
            ~OccupancyVolume(){};

            // For each particle in the system, find its position in the volume
            // Save particle index/id to contents vector
            void Populate(const PSYS& psys);

            // Given a calculated position of a particle in the grid, find the
            // index in a 1D vector
            int Get_idx(int i, int j, int k);

            // For each grid cell, compute its neighboring cells. This is done once
            // in order to cache the neighboring cells ahead of time.
            void ComputeNeighbors();

            // Get neighbor indices of a cell
            const std::vector<size_t>& GetNeighborhood(const size_t i) const;

            // Get all particles within a cell
            const std::vector<size_t>& GetCellContents(const size_t i) const;

            // Find the position of a single particle in the grid
            int FindPosInVolume(const Vector& p);

            // Get size of grid
            size_t Gsize();

            // Clear particles from a cell
            void ClearCells();

        private:
            Vector _llc;
            Vector _urc;
            Vector _Lxyz;
            Vector _nxyz;
            double _R;
            double _cellsize;

            std::vector<std::vector<size_t>> contents;
            std::vector<std::vector<size_t>> neighbors;

            // These are used to in the ComputeNeighbors() function
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

    typedef std::shared_ptr<OccupancyVolume> OV;

    OV CreateOccupancyVolume(const Vector& LLC, const Vector URC, const double R);

}


#endif