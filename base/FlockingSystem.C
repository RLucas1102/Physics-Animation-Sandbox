#include "FlockingSystem.h"

using namespace pba;

FlockingSystem::FlockingSystem(const std::string &nam) :
    name (nam)
    {}

Vector FlockingSystem::ComputeDistance(const size_t a, const size_t b)
{
    Vector posA = GetPos(a);
    Vector posB = GetPos(b);
    return posB - posA;
}

Vector FlockingSystem::ComputeVelDiff(const size_t a, const size_t b)
{
    Vector velA = GetVel(a);
    Vector velB = GetVel(b);
    return velB - velA;
}

double FlockingSystem::ComputeRangeLimit(const size_t a, const size_t b)
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

double FlockingSystem::ComputeFOVLimit(const size_t a, const size_t b)
{
    Vector aVel = GetVel(a);
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

Vector FlockingSystem::ComputeCAAcc()
{
    Vector CA = Vector(0,0,0);

    for (size_t p = 0; p < CBsize(); p++)
    {
        double mag = Distances[p].magnitude();
        CA += (Distances[p]/(mag * mag)) * Range_Limiters[p] * FOV_Limiters[p];
    }

    return -_Kca * CA;
}

Vector FlockingSystem::ComputeMAcc()
{
    Vector M = Vector(0,0,0);

    for (size_t p = 0; p < CBsize(); p++) 
    {
        M += DiffVelocities[p] * Range_Limiters[p] * FOV_Limiters[p];
    }

    return _Km * M;

}

Vector FlockingSystem::ComputeCAcc()
{
    Vector C = Vector(0,0,0);

    for (size_t p = 0; p < CBsize(); p++) 
    {
        C += Distances[p] * Range_Limiters[p] * FOV_Limiters[p];
    }

    return _Kc * C;
}

Flock pba::CreateFlock(const std::string &nam)
{
    return std::make_shared<FlockingSystem>(FlockingSystem(nam));
}