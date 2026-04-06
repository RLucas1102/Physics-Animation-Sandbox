#include "Torque.h"

using namespace pba;

TorqueFromForce::TorqueFromForce(Force& f) :
    force (f)
    {}

void TorqueFromForce::compute(PSYS& PQ, const double dt) {

    std::shared_ptr<RigidBodySystem> s = std::dynamic_pointer_cast<RigidBodySystem>(PQ);

    force->compute(PQ, dt);

    Vector totalForce = Vector(0,0,0);
    Vector torque = Vector(0,0,0);
    double totalMass = 0.0;
    for (size_t p = 0; p < s->Psize(); p++)
    {
        double mass = s->GetMass(p);
        totalMass += mass;

        Vector acc  = s->GetAcc(p);
        totalForce += mass * acc;
        
        Vector pos = s->GetLeverArm(p);
        torque += pos ^ (mass * acc); 
    }

    s->_COMAcc = totalForce / totalMass;

    s->_angularAcc = torque;

}

Torque pba::CreateTorqueFromForce(Force& f) {
    return std::make_shared<TorqueFromForce>(TorqueFromForce(f));
}