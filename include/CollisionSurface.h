//---------------------------------------------------------
//
//  Created by: Lucas Robenolt
//
//  Purpose: A class to hold all triangles of a surface that
//  particles will collide with
//
//---------------------------------------------------------
#ifndef COLLISION_SURFACE_H
#define COLLISION_SURFACE_H

#include "CollisionTriangle.h"
#include "RigidBodySystem.h"

#include <vector>
#include <memory>
#include <fstream>
#include <iostream>
#include <sstream>

namespace pba {
    
    class CollisionSurfaceRaw {

        public:

            CollisionSurfaceRaw();
            ~CollisionSurfaceRaw(){};

            // Add a triangle to the collision surface
            void AddTriangle( const CollisionTriangle& t);
            // Clear all triangles from surface
            void SClear();
            // Get specific triangle of the surface
            CollisionTriangle GetTriangle(size_t i);
            // Increase and decrease the coefficient of restitution
            // of the surface
            void IncreaseCoeffR();
            void DecreaseCoeffR();
            // Return the current coefficient of restitution
            double GetCoeffR();
            // Displays all triangles in surface
            void Display();
            // Calls a specific triangle's handle function
            void handle(    const Vector& XS, const Vector& VS, 
                            const double& dt, const Vector& XH, 
                            const double& dtH, Vector& XR, Vector& VR,
                            const size_t& i) const;
            // A function to check if there are multiple hits within a 
            // given timestep
            bool MultiTriangleHit(  const Vector& pos, const Vector& vel, 
                                    const double& dt, Vector& XH, 
                                    double& dtH, size_t& pH);
            
            // A function to check if there are multiple hits within a
            // given timestep for RBD system
            bool MultiTriangleHit_RBD( RigidBody& rbd, const size_t pos,
                                       const double& dt, Vector& XH, size_t aH,
                                       double& dtH, size_t& pH);

            // Easy method to create a collision box
            void MakeBox(const double& m);

            // Easy method to make collision plane
            void MakePlane(const double& m);

            // Create collision surface from model
            void MakeSurfFromModel(const char* FilePath, const double& m, Vector& t);

            // Get upper right and bottom left corners of box
            Vector const GetLLC();
            Vector const GetURC();

        private:

            std::vector<CollisionTriangle> triangles;
            double coeffR;
            double coeffS;
            
            Vector _llc, _urc;


    };
    
    typedef std::shared_ptr<CollisionSurfaceRaw> CollisionSurface;

    // Create shared pointer to a collision surface
    CollisionSurface MakeCollisionSurface();

}


#endif