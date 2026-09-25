#include "Vector.h"
#include "CollisionTriangle.h"
#include "LinearAlgebra.h"

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

    Vector Normal = normal;
    Normal.normalize();

    double fXS = Normal * (XS - P0);
    double fXU = Normal * (XU - P0);
 

    if (fXU == 0 || (fXU * fXS) < 0) {
        double sign = 1.0;
        if(fXS < 0) { sign = -1.0; }
        double epsilon = 0.0001;
        
        // Vector XH_candidate = XS + VS * ((Normal * (P0 - XS) / (Normal * VS)));
        Vector XH_candidate = XS + VS * ((Normal * (P0 - XS) / (Normal * VS))) + normal * sign * epsilon;
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

bool pba::CollisionTriangleRaw::hit_RBD(RigidBody &rbd, const size_t p, const double dt, Vector &XH, double &dtH)
{
    bool hitFound = false;

    Vector x_a = rbd->_COM + rbd->_angularRot * rbd->GetLeverArm(p);

    Vector rotor =  rbd->_angularVel * dt;
    Matrix angularRot_Temp = pba::rotation(rotor.unitvector(), -rotor.magnitude()) * rbd->_angularRot;
    Vector x_a_prime = rbd->_COM + rbd->_linearVel * dt + angularRot_Temp * rbd->GetLeverArm(p);

    Vector Normal = normal;
    Normal.normalize();

    double fx_a = Normal * (x_a - P0);
    double fx_a_prime = Normal * (x_a_prime - P0);

    if (fx_a_prime == 0 || (fx_a * fx_a_prime) < 0)
    {
        double dt_1 = 0;
        double dt_2 = dt;
        double tolerance = 0.00001;
        int Nmax = 20;
        int counter = 0;
        double dtH_candidate;
        Vector XH_candidate;

        while (counter < Nmax) {
            // Create half-way time
            double dt_mid = (dt_1 + dt_2) / 2;

            // Create half-way position
            Vector rotor_mid = rbd->_angularVel * dt_mid;
            Matrix angularRot_mid = pba::rotation(rotor_mid.unitvector(), -rotor_mid.magnitude()) * rbd->_angularRot;
            Vector x_mid = rbd->_COM + rbd->_linearVel * dt_mid + angularRot_mid * rbd->GetLeverArm(p);

            double f_mid = Normal * (x_mid - P0); // Create half-way plane

            if (fabs(f_mid) < tolerance) {
                dtH_candidate = dt_mid;
                XH_candidate = x_mid;
                counter = Nmax;
            }
            else {

                if ((fx_a * f_mid) > 0) {
                    fx_a = f_mid;
                    dt_1 = dt_mid;
                }
                else {
                    fx_a_prime = f_mid;
                    dt_2 = dt_mid;
                }

                counter++;
                if (counter >= Nmax) {
                    dtH_candidate = (dt_1 + dt_2) / 2;

                    Vector rotor =  rbd->_angularVel * dtH_candidate;
                    Matrix angularRot_final = pba::rotation(rotor.unitvector(), -rotor.magnitude()) * rbd->_angularRot;
                    XH_candidate = rbd->_COM + rbd->_linearVel * dtH_candidate + angularRot_final * rbd->GetLeverArm(p);
                }

            }

        }

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

void pba::CollisionTriangleRaw::handle_RBD(RigidBody &rbd, const size_t aH, const double dtH)
{
    Vector Normal = normal;
    Normal.normalize();
    
    Vector rotor =  rbd->_angularVel * dtH;
    rbd->_angularRot = pba::rotation(rotor.unitvector(), -rotor.magnitude()) * rbd->_angularRot;
    
    rbd->RecomputeMOI();
    
    rbd->_COM += rbd->_linearVel * dtH;

    double mass = rbd->GetMass(aH);
    double term1 = 2 * rbd->_linearVel * Normal;
    Vector term2 = (mass / rbd->_totalMass) * rbd->_angularVel;
    Vector term3 = Normal ^ (rbd->_angularRot * rbd->GetLeverArm(aH));
    double term4 = (pow(mass, 2)/rbd->_totalMass);
    
    double A = -((term1 + term2 * term3) / 1 + (term4 * term3 * rbd->InverseMOI() * term3));

    rbd->_linearVel = rbd->_linearVel + A * Normal;

    rbd->_angularVel = rbd->_angularVel + A * mass * rbd->InverseMOI() * (Normal ^ (rbd->_angularRot * rbd->GetLeverArm(aH)));

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