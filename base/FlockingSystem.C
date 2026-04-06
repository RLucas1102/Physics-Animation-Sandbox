#include "FlockingSystem.h"

using namespace pba;

FlockingSystem::FlockingSystem(const std::string &nam) :
    name (nam)
    {}

double FlockingSystem::ComputeDistance(const Vector &a, const Vector &b)
{
    Vector dist = a - b;
    return dist.magnitude();
}

double FlockingSystem::ComputeRangeLimit(const Vector& distance)
{

    double distance_temp = distance.magnitude();
    
    double rab;
    if (distance_temp <= _R)
    {
        rab = 1;
    }
    else if (distance_temp > _R && distance_temp <= _R + _Ramp)
    {
        rab = 1 - ((distance_temp - _R)/_Ramp);
    }
    else {
        rab = 0;
    }
    
    return rab;

}

double FlockingSystem::ComputeFOVLimit(const Vector& distance, const size_t p)
{
    Vector aVel = GetVel(p);
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