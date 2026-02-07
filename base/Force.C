#include "Force.h"
#include "ParticleSystem.h"
#include "Vector.h"

using namespace pba;

void AccumulatingForce::compute( PSYS& psys, const double dt) {
    
    Vector resetForce = Vector(0,0,0);
    for (size_t i = 0; i < psys->Psize(); i++) {
        psys->SetAcc(i, resetForce);
    }

    for (size_t i = 0; i < forces.size(); i++) {
        forces[i]->compute(psys, dt);
    }
    
}

void AccumulatingForce::AddForce( Force& f) {
    forces.push_back(f);
}

Force AccumulatingForce::GetForce( const size_t i) {
    return forces[i];
}

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

void GravityForce::IncreaseGravityForce() {
    gravity *= 1.1;
}

void GravityForce::DecreaseGravityForce() {
    gravity /= 1.1;
}

double GravityForce::GetGravityMag() {
    return gravity[1];
}

Force pba::CreateGravityForce(const Vector& g) {
    return Force( new GravityForce(g) );
}

Force pba::CreateAccumulatingForce() {
    return Force( new AccumulatingForce() );
}