#include "Vector.h"
#include "CollisionTriangle.h"

using namespace pba;

CollisionTriangleRaw::CollisionTriangleRaw(const Vector& p0, const Vector& p1, const Vector& p2) : 
    P0 (p0),
    P1 (p1),
    P2 (p2)
    {
        e1 = p1 - p0;
        e2 = p2 - p0;

        normal = e1 ^ e2;

    }

bool CollisionTriangleRaw::hit(const Vector& XS, const Vector& VS, const double dt, Vector& XH, double& dtH) {
    bool hit = false;
    
    Vector XU = XS + VS * dt;

    Vector Normal = normal / normal.magnitude();

    double fXS = normal * (XS - P0);
    double fXU = normal * (XU - P0);
    if (fXU == 0 || fXU * fXS < 0) {
        Vector XH_candidate = XS + VS * ((Normal * (P0 - XS) / (Normal * VS)));
        double dtH_candidate = (Normal * (P0 - XS) / (Normal * VS));

        double u = normal * ((XH_candidate - P0) ^ e2) / (normal * normal);
        double v = normal * ((e1 ^ (XH_candidate - P0))) / (normal * normal);
        if ( (u >= 0 && u <= 1) &&
             (v >= 0 && v <= 1) &&
             (u + v >= 0 && u + v <= 1) ) {

            XH = XH_candidate;
            dtH = dtH_candidate;
            hit = true;

        }
    }

    return hit;
}

double CollisionTriangleRaw::GetP0(size_t i) {
    return P0[i];
}

double CollisionTriangleRaw::GetP1(size_t i) {
    return P1[i];
}

double CollisionTriangleRaw::GetP2(size_t i) {
    return P2[i];
}

CollisionTriangle pba::MakeCollisionTriangle(const Vector& p0, const Vector& p1, const Vector& p2) {
    return CollisionTriangle(new CollisionTriangleRaw(p0, p1, p2));
}