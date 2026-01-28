#include "GISolver.h"
#include "Vector.h"

using namespace pba;

    AdvancePositionStarter::AdvancePositionStarter(PSYS& pq) :
        PQ (pq)
        {}
    
    AdvanceVelocityStarter::AdvanceVelocityStarter(PSYS& pq) :
        PQ (pq)
        {}    

    void AdvancePositionStarter::solve(const double dt) {
        for (size_t i=0; i< PQ->Psize(); i++) {

            Vector P = PQ->GetPos(i);
            Vector V = PQ->GetVel(i);

            P += V * dt;

            PQ->SetPos(i, P);

        }
    }

    void AdvanceVelocityStarter::solve(const double dt) {
        Vector center;

        for(size_t i = 0; i < PQ->Psize(); i++) {
            center += PQ->GetPos(i);
        }

        center = center / PQ->Psize();

        for(size_t i = 0; i < PQ->Psize(); i++) {
            Vector n = PQ->GetPos(i) - center;
            n.normalize();
            double vmag = PQ->GetVel(i).magnitude();

            Vector V = PQ->GetVel(i);

            V -= n*(n*V);
            V *= vmag/V.magnitude();

            PQ->SetVel(i, V);
        }
    }

    GISolver pba::CreateAdvancePositionStarter(PSYS& pq) {
        return GISolver( new AdvancePositionStarter(pq) );
    }

    GISolver pba::CreateAdvanceVelocityStarter(PSYS& pq) {
        return GISolver( new AdvanceVelocityStarter(pq) );
    }

    GISolver pba::CreateForwardEulerSolver(GISolver& A, GISolver& B) {
        return GISolver( new ForwardEulerSolver(A,B) );
    }

    GISolver pba::CreateBackwardEulerSolver(GISolver& A, GISolver& B) {
        return GISolver( new BackwardEulerSolver(A,B) );
    }

    GISolver pba::CreateLeapFrogSolver(GISolver& A, GISolver& B) {
        return GISolver( new LeapFrogSolver(A,B) );
    }