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
    bool hitFound = false;
    
    Vector XU = XS + VS * dt;

    Vector Normal = normal / normal.magnitude();

    double fXS = Normal * (XS - P0);
    double fXU = Normal * (XU - P0);
    if (fXU * fXS < 0 && fXS < 0) {
        fXS = fXU;
    }

    if (fXU == 0 || fXU * fXS < 0) {
        Vector XH_candidate = XS + VS * ((Normal * (P0 - XS) / (Normal * VS)));
        double dtH_candidate = (Normal * (P0 - XS) / (Normal * VS));

        double u = normal * ((XH_candidate - P0) ^ e2) / pow(normal.magnitude(), 2);
        double v = normal * ((e1 ^ (XH_candidate - P0))) / pow(normal.magnitude(), 2);
        if ( (u >= 0 && u <= 1) &&
             (v >= 0 && v <= 1) &&
             (u + v >= 0 && u + v <= 1) ) 
             {

                XH = XH_candidate;
                dtH = dtH_candidate;
                hitFound = true;

             }
    }

    return hitFound;
}

void CollisionTriangleRaw::handle(  const Vector &XS, const Vector &VS, 
                                    const double &dt, const Vector &XH, 
                                    const double &dtH, Vector &XR, Vector &VR, 
                                    const double Cr, const double Cs) const 
                                    {
                                        Vector Normal = normal / normal.magnitude();

                                        VR = Cs * VS - (Cs + Cr) * Normal * (Normal * VS);
                                        XR = XH + VR * (dt - dtH);
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

void CollisionTriangleRaw::SetColor(const Color& inCol) {
    color = inCol;
}

float const CollisionTriangleRaw::GetColor(size_t i) {
    return color[i];
}

CollisionTriangle pba::MakeCollisionTriangle(const Vector& p0, const Vector& p1, const Vector& p2) {
    return CollisionTriangle(new CollisionTriangleRaw(p0, p1, p2));
}