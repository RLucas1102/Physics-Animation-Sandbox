#ifndef COLLISION_SURFACE_H
#define COLLISION_SURFACE_H

#include "CollisionTriangle.h"

#include <vector>
#include <memory>

namespace pba {
    
    class CollisionSurfaceRaw {

        public:

            CollisionSurfaceRaw();
            ~CollisionSurfaceRaw(){};

            void AddTriangle( const CollisionTriangle& t);
            void SClear();
            CollisionTriangle GetTriangle(size_t i);
            void IncreaseCoeffR();
            void DecreaseCoeffR();
            double GetCoeffR();
            void Display();
            void handle(    const Vector& XS, const Vector& VS, 
                            const double& dt, const Vector& XH, 
                            const double& dtH, Vector& XR, Vector& VR,
                            const size_t& i) const;

            bool MultiTriangleHit(  const Vector& pos, const Vector& vel, 
                                    const double& dt, Vector& XH, 
                                    double& dtH, size_t& pH);


        private:

            std::vector<CollisionTriangle> triangles;
            double coeffR;
            double coeffS;


    };
    
    typedef std::shared_ptr<CollisionSurfaceRaw> CollisionSurface;

    CollisionSurface MakeCollisionSurface();

}


#endif