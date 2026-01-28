#include "Force.h"
#include "ParticleSystem.h"
#include "Vector.h"

using namespace pba;

GravityForce::GravityForce(const Vector& g) :
    gravity (g)
    {}

void GravityForce::compute( PSYS& psys, const double dt) {
    for (size_t i = 0; i < psys->Psize(); i++) {
        Vector A = psys->GetAcc(i);

        A += gravity;

        psys->SetAcc(i, A);
    }
    
}

Force pba::CreateGravityForce(const Vector& g) {
    return Force( new GravityForce(g) );
}