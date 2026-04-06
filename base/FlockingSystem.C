#include "FlockingSystem.h"

using namespace pba;

FlockingSystem::FlockingSystem(const std::string &nam) :
    name (nam)
    {}

Vector FlockingSystem::ComputeDistance(const Vector &a, const Vector &b)
{
    return b - a;
}

double FlockingSystem::ComputeRangeLimit(const Vector& a, const Vector& b)
{

    Vector distance = ComputeDistance(a,b);
    double dab = distance.magnitude();
    
    double rab;
    if (dab <= _R)
    {
        rab = 1;
    }
    else if (dab > _R && dab <= _R + _Ramp)
    {
        rab = 1 - ((dab - _R)/_Ramp);
    }
    else {
        rab = 0;
    }
    
    return rab;

}

double FlockingSystem::ComputeFOVLimit(const Vector& a, const Vector& aVel, const Vector& b)
{
    Vector distance = ComputeDistance(a,b);
    double theta_ab = (distance * aVel) / (distance.magnitude() * aVel.magnitude()); 
    double fab;

    if(theta_ab >= cos(_theta)) {
        fab = 1;
    }
    else if (theta_ab < cos(_theta) && theta_ab > cos(_theta + _theta_ramp)) {
        fab = 1 - ((cos(_theta) - theta_ab) / (cos(_theta) - cos(_theta + _theta_ramp)));
    }
    else {
        fab = 0;
    }

    return fab;

}