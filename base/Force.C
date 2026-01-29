#include "Force.h"
#include "ParticleSystem.h"
#include "Vector.h"

using namespace pba;

GravityForce::GravityForce(const Vector& g) :
    gravity (g)
    {}

void GravityForce::compute( PSYS& psys, const double dt) {

    Vector initA = Vector(0, 0, 0);
    for (size_t i = 0; i < psys->Psize(); i++) {

        psys->SetAcc(i, initA);

        Vector A = psys->GetAcc(i);

        A += gravity;

        psys->SetAcc(i, A);
    }
    
}

void GravityForce::IncreaseGravityForce() {
    gravity *= 1.1;
}

void GravityForce::DecreaseGravityForce() {
    gravity /= 1.1;
}

Force pba::CreateGravityForce(const Vector& g) {
    return Force( new GravityForce(g) );
}