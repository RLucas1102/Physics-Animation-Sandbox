#include "GISolver.h"
#include "Vector.h"
#include "Force.h"

using namespace pba;

    AdvancePositionStarter::AdvancePositionStarter(PSYS& pq) :
        PQ (pq)
        {}
    
    AdvanceVelocityStarter::AdvanceVelocityStarter(PSYS& pq) :
        PQ (pq)
        {}    

    AdvancePosition::AdvancePosition(PSYS& pq) :
        PQ (pq)
        {}

    AdvanceVelocity::AdvanceVelocity(PSYS& pq, Force& f) :
        PQ (pq),
        force (f)
        {}

    AdvancePositionWithCollision::AdvancePositionWithCollision(PSYS& pq, CollisionSurface& c) :
        PQ (pq),
        C (c)
        {}

    AdvancePositionWithCollisionSPH::AdvancePositionWithCollisionSPH(PSYS& pq, CollisionSurface& c) :
        PQ (pq),
        C (c)
        {}

    AdvanceVelocitySPH::AdvanceVelocitySPH(PSYS& pq, Force& f, const double userArray[6]) :
        PQ (pq),
        force (f),
        Pbar (userArray[0]), rhoBar (userArray[1]), gamma (userArray[2]),
        alpha (userArray[3]), beta (userArray[4]), eps (userArray[5])
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
    
    void AdvancePosition::solve(const double dt) {
        for (size_t i=0; i< PQ->Psize(); i++) {

            Vector P = PQ->GetPos(i);
            Vector V = PQ->GetVel(i);

            P += V * dt;

            PQ->SetPos(i, P);

        }
    }

    void AdvanceVelocity::solve(const double dt) {
        force->compute(PQ, dt);

        for (size_t i = 0; i < PQ->Psize(); i++) {
            Vector V = PQ->GetVel(i);
            Vector A = PQ->GetAcc(i);
            
            V += A * dt;

            PQ->SetVel(i, V);

        }
        
    }

    void AdvancePositionWithCollision::solve(const double dt) {
        for (size_t i = 0; i < PQ->Psize(); i++) {
            Vector X0 = PQ->GetPos(i);
            Vector V0 = PQ->GetVel(i);
            Vector XR = X0 + V0 * dt;
            Vector VR = V0;
            double running_dt = dt;
            bool moreHits = true;
            while(moreHits) {
                moreHits = false;
                Vector XH;
                double dtH = 0;
                size_t pH = -1;
                if(C->MultiTriangleHit(X0, V0, running_dt, XH, dtH, pH)) {
                    moreHits = true;
                    C->handle(X0, V0, dt, XH, dtH, XR, VR, pH);
                    X0 = XH;
                    V0 = VR;
                    running_dt = dt - dtH;
                    if (running_dt == 0.0) {
                        moreHits = false;
                    }
                }
            }
            PQ->SetPos(i, XR);
            PQ->SetVel(i, VR);
        }
        
    }

    void AdvancePositionWithCollisionSPH::solve(const double dt)   {
        
    }

    GISolver pba::CreateAdvancePositionStarter(PSYS& pq) {
        return GISolver( new AdvancePositionStarter(pq) );
    }

    GISolver pba::CreateAdvanceVelocityStarter(PSYS& pq) {
        return GISolver( new AdvanceVelocityStarter(pq) );
    }

    GISolver pba::CreateSixthOrderSolver(GISolver& s) {
        return GISolver( new SixthOrderSolver(s) );
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

    GISolver pba::CreateAdvancePosition(PSYS &pq) {
        return GISolver( new AdvancePosition(pq) );
    }

    GISolver pba::CreateAdvanceVelocity(PSYS &pq, Force& f) {
        return GISolver( new AdvanceVelocity(pq, f) );
    }

    GISolver pba::CreateAdvancePositionWithCollision(PSYS &pq, CollisionSurface& c) {
        return GISolver( new AdvancePositionWithCollision(pq, c) );
    }
    
    GISolver pba::CreateAdvancePositionWithCollisionSPH(PSYS &pq, CollisionSurface& c) {
        return GISolver( new AdvancePositionWithCollisionSPH(pq, c) );
    }

    GISolver pba::CreateAdvanceVelocitySPH(PSYS& pq, Force& f, const double userArray[6]) {
        return GISolver( new AdvanceVelocitySPH(pq, f, userArray));
    }
 
 double CalcSpeedOfSound(const double Pbar, const double rhoBar, const double gamma, const double density) {
    double term1 = Pbar / rhoBar;
    double term2 = std::pow(density / rhoBar, (gamma - 1) );
    double C = std::pow(gamma * term1 * term2, 0.5);
    
    return C;
 }