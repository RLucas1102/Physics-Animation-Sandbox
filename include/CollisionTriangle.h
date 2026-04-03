//---------------------------------------------------------
//
//  Created by: Lucas Robenolt
//
//  Purpose: A class to hold the data for one triangle that
//  will be hit by a particle
//
//---------------------------------------------------------

#ifndef COLLISION_TRIANGLE_H
#define COLLISION_TRIANGLE_H

#include "Vector.h"
#include "memory"
#include "RigidBodySystem.h"
#include "Color.h"

#include <iostream>

namespace pba {
    class CollisionTriangleRaw {
        public:

            CollisionTriangleRaw(const Vector& p0, const Vector& p1, const Vector& p2);
            ~CollisionTriangleRaw(){};

            // When a particle hits a triangle, this function is called
            // Each particle is tested to see if it collides with the triangle's infinite plane
            // Then it is tested to see if u and v are barycentric coords i.e. inside the triangle
            // Returns bool and sets the hit position and hit time
            bool hit( const Vector& XS, const Vector& VS, const double dt, Vector& XH, double& dtH);

            bool hit_RBD( RigidBodySystem& rbd, const size_t p, const double dt, Vector& XH, double& dtH);
            
            // When a particle needs to move based on a collision that happened, the handle function
            // is called. This updates the reflected position and reflected velocity based on a
            void handle(    const Vector& XS, const Vector& VS, 
                            const double& dt, const Vector& XH, 
                            const double& dtH, Vector& XR, Vector& VR,
                            const double Cr, const double Cs) const;

            // Getter functions for the vertices and color of a triangle
            double GetP0(size_t i);
            double GetP1(size_t i);
            double GetP2(size_t i);
            float const GetColor(size_t i);

            // Setter function for the color of a triangle
            void SetColor(const Color& inCol);

        private:

            Vector P0;
            Vector P1;
            Vector P2;
            Vector e1;
            Vector e2;
            Vector normal;
            Color color;

    };

    typedef std::shared_ptr<CollisionTriangleRaw> CollisionTriangle;

    // Create a shared pointer to a triangle
    CollisionTriangle MakeCollisionTriangle(const Vector& p0, const Vector& p1, const Vector& p2);

}


#endif