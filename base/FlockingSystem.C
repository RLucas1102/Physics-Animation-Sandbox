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

Vector FlockingSystem::ComputeCAAcc(const size_t a, const std::vector<size_t>& candidates)
{
    Vector CA = Vector(0,0,0);

    for (size_t candidate : candidates)
    {
        Vector dist = ComputeDistance(a, candidate);

        double mag = dist.magnitude();
        CA += (dist/(mag * mag)) * ComputeRangeLimit(a, candidate) * ComputeFOVLimit(a, candidate);
    }

    return -_Kca * CA;
}

Vector FlockingSystem::ComputeMAcc(const size_t a, const std::vector<size_t>& candidates)
{
    Vector M = Vector(0,0,0);

    for (size_t candidate : candidates) 
    {
        M += ComputeVelDiff(a, candidate) * ComputeRangeLimit(a, candidate) * ComputeFOVLimit(a, candidate);
    }

    return _Km * M;

}

Vector FlockingSystem::ComputeCAcc(const size_t a, const std::vector<size_t>& candidates)
{
    Vector C = Vector(0,0,0);

    for (size_t candidate : candidates) 
    {
        Vector dist = ComputeDistance(a, candidate);

        C += dist * ComputeRangeLimit(a, candidate) * ComputeFOVLimit(a, candidate);
    }

    return _Kc * C;
}

Flock pba::CreateFlock(const std::string &nam)
{
    return std::make_shared<FlockingSystem>(FlockingSystem(nam));
}