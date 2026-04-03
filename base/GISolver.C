#include "GISolver.h"
#include "Vector.h"
#include "Force.h"
#include <iostream>
#include <vector>

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

    AdvancePositionWithCollisionSPH::AdvancePositionWithCollisionSPH(PSYS& pq, CollisionSurface& c, OV& o) :
        PQ (pq),
        C (c),
        O (o)
        {}

    AdvanceVelocitySPH::AdvanceVelocitySPH(PSYS& pq, Force& f) :
        PQ (pq),
        force (f)
        {}
    
    AdvancePositionRBD::AdvancePositionRBD(PSYS& pq) :
        PQ (pq)
        {}    

    pba::AdvancePositionWithCollisionRBD::AdvancePositionWithCollisionRBD(PSYS& pq, CollisionSurface& c) : 
        PQ (pq),
        C (c)
        {}

    AdvanceVelocityRBD::AdvanceVelocityRBD(PSYS& pq, Force& f) :
        PQ (pq),
        tau (CreateTorqueFromForce(f))
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

        std::shared_ptr<SPHSystem> sph = std::dynamic_pointer_cast<SPHSystem>(PQ);

        #pragma omp parallel for
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

        O->ClearCells();
        O->Populate(PQ);

        #pragma omp parallel for
        for (size_t i = 0; i < PQ->Psize(); i++) {
            double newRho = 0;
            size_t cell = O->FindPosInVolume(PQ->GetPos(i));
            std::vector<size_t> neighborhood = O->GetNeighborhood(cell);
            
            for (size_t cell : neighborhood) {
                std::vector<size_t> cellContents = O->GetCellContents(cell);
                
                for (size_t particle : cellContents) {
                    Vector AB = PQ->GetPos(i) - PQ->GetPos(particle);
                    newRho += PQ->GetMass(particle) * CalcWeightKernel(AB, sph->GetH());
                }
            }

            sph->SetRho(i, newRho);

        }
    }

    void AdvanceVelocitySPH::solve(const double dt) {
        force->compute(PQ, dt);

        #pragma omp parallel for
        for (size_t i = 0; i < PQ->Psize(); i++) {
            Vector V = PQ->GetVel(i);
            Vector A = PQ->GetAcc(i);
           
            if(V.magnitude() > _VT) {
                V = V * (_VT/V.magnitude());
            }

            if(A.magnitude() > _AT) {
                A = A * (_AT/A.magnitude());
            }

            V += A * dt;

            PQ->SetVel(i, V);

        }

    }

    void AdvanceVelocitySPH::ChangeADampening(const double AT) {
        _AT += AT;
    }

    void AdvanceVelocitySPH::ChangeVDampening(const double VT) {
        _VT += VT;
    }

    void AdvancePositionRBD::solve(const double dt)
    {
        std::shared_ptr<RigidBodySystem> rbd = std::dynamic_pointer_cast<RigidBodySystem>(PQ);

        Vector rotor =  rbd->_angularVel * dt;
        rbd->_angularRot = pba::rotation(rotor.unitvector(), -rotor.magnitude()) * rbd->_angularRot;

        rbd->RecomputeMOI(); // Moment of inertia needs to be recomputed after rotation

        rbd->_COM += rbd->_linearVel * dt; // update COM position        
    }
    
    void AdvancePositionWithCollisionRBD::solve(const double dt)
    {
        std::shared_ptr<RigidBodySystem> rbd = std::dynamic_pointer_cast<RigidBodySystem>(PQ);   

        double running_dt = dt;
        bool moreHits = true;
        while (moreHits)
        {
            moreHits = false;
            
            // Initialize expected hit time, collision plane, and colliding particle
            Vector XH;
            double dt_EH = dt;
            size_t p_EH = -1;
            size_t a_EH = -1;

            for (size_t a = 0; a < PQ->Psize(); a++)
            {
                moreHits = C->MultiTriangleHit_RBD(rbd, a, running_dt, XH, a_EH, dt_EH, p_EH);
            }
            
            // Now we must handle collisions

            if (moreHits)
            {
                running_dt = dt - dt_EH;
                if (running_dt <= 0.0) {
                    moreHits = false;
                }
            }
        }
    }

    void pba::AdvanceVelocityRBD::solve(const double dt)
    {
        std::shared_ptr<RigidBodySystem> rbd = std::dynamic_pointer_cast<RigidBodySystem>(PQ);
        
        // Compute tau (updates _angularAcc and _COMAcc)
        tau->compute(PQ, dt);       

        rbd->_angularMom += rbd->_angularAcc * dt;
        rbd->_angularVel = rbd->InverseMOI() * rbd->_angularMom;

        rbd->_linearVel += rbd->_COMAcc * dt;
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
    
    GISolver pba::CreateAdvancePositionWithCollisionSPH(PSYS &pq, CollisionSurface& c, OV& o) {
        return GISolver( new AdvancePositionWithCollisionSPH(pq, c, o) );
    }

    GISolver pba::CreateAdvanceVelocitySPH(PSYS& pq, Force& f) {
        return GISolver( new AdvanceVelocitySPH(pq, f));
    }

    GISolver pba::CreateAdvancePositionRBD(PSYS& pq)
    {
        return GISolver( new AdvancePositionRBD(pq));
    }

    GISolver pba::CreateAdvancePositionWithCollisionRBD(PSYS& pq, CollisionSurface& c)
    {
        return GISolver( new AdvancePositionWithCollisionRBD(pq, c));
    }

    GISolver pba::CreateAdvanceVelocityRBD(PSYS& pq, Force& f)
    {
        return GISolver( new AdvanceVelocityRBD(pq, f));
    }