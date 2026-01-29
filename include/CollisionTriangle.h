#ifndef COLLISION_TRIANGLE_H
#define COLLISION_TRIANGLE_H

#include "Vector.h"
#include "memory"

namespace pba {
    class CollisionTriangleRaw {
        public:

            CollisionTriangleRaw(const Vector& p0, const Vector& p1, const Vector& p2);
            ~CollisionTriangleRaw(){};

            bool hit( const Vector& XS, const Vector& VS, const double dt, Vector& XH, double& dtH);

            double GetP0(size_t i);
            double GetP1(size_t i);
            double GetP2(size_t i);

        private:

            Vector P0;
            Vector P1;
            Vector P2;
            Vector e1;
            Vector e2;
            Vector normal;

    };

    typedef std::shared_ptr<CollisionTriangleRaw> CollisionTriangle;

    CollisionTriangle MakeCollisionTriangle(const Vector& p0, const Vector& p1, const Vector& p2);

}


#endif