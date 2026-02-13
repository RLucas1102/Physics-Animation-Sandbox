//*******************************************************************
 //
 //  GISolver.h
 //
 //  Base class for Geometric Integration solvers
 //
 //  Copyright (c) 2017 Jerry Tessendorf
 //
 //  Adapted by Lucas Robenolt
 //
 //*******************************************************************
  
 #ifndef __PBA_GISOLVER_H__
 #define __PBA_GISOLVER_H__
  
 #include <cmath>
 #include <memory>

 #include "ParticleSystem.h"
 #include "Force.h"
 #include "CollisionSurface.h"
  
 namespace pba{
  
 // Base class that all solvers will inherent from
 class GISolverBase
 {
   public:
  
     GISolverBase(){}
  
     virtual void init() = 0;
     virtual void solve( const double dt ) = 0;
     virtual ~GISolverBase(){};
  
  
   protected:
  
 };
  
 typedef std::shared_ptr<GISolverBase> GISolver;

// -------------------------------------------------
// STARTER CODE PARTIAL SOLVERS
// These partial solvers were an adaptation of 
// Jerry Tessendorf's starter solvers
// These solvers move particles with a repeating 
// pattern around a circle.

 class AdvancePositionStarter : public GISolverBase
 {
    public:

      AdvancePositionStarter(PSYS& pq);
      ~AdvancePositionStarter() {};

      void init() {};
      void solve(const double dt);

    private:

      PSYS PQ;

 };

class AdvanceVelocityStarter : public GISolverBase
{
  public:
  
    AdvanceVelocityStarter(PSYS& pq);
    ~AdvanceVelocityStarter() {};

    void init() {};
    void solve(const double dt);

  private:

    PSYS PQ;

};

//---------------------------------------------------

//---------------------------------------------------
// SIMPLE POSITION PARTIAL SOLVER
// This solver simply updates position based on 
// velocity with no collisions
// Mainly used for testing the velocity partial solver
 class AdvancePosition : public GISolverBase
 {
    public:

      AdvancePosition(PSYS& pq);
      ~AdvancePosition() {};

      void init() {};
      void solve(const double dt);

    private:

      PSYS PQ;

 };

 //--------------------------------------------------

 //--------------------------------------------------
 // POSITION PARTIAL SOLVER WITH COLLISION
 // This solver updates position with collision
 // A collision surface is defined in MyThing and a
 // smart pointer is handed to this class to use for
 /// collision detection and handling
 class AdvancePositionWithCollision : public GISolverBase
 {
    public:

      AdvancePositionWithCollision(PSYS& pq, CollisionSurface& c);
      ~AdvancePositionWithCollision(){};

      void init() {};
      void solve(const double dt);

      private:

        PSYS PQ;
        CollisionSurface C;
 };

 //---------------------------------------------------


 //---------------------------------------------------
 // POSITION PARTIAL SOLVER WITH COLLISION FOR SPH
 // This solver updates the position of paticles
 // with collision based on fluid dynamics implemented 
 // with SPH
 class AdvancePositionWithCollisionSPH : public GISolverBase 
 {
    public:
      AdvancePositionWithCollisionSPH(PSYS& pq, CollisionSurface& c);
      ~AdvancePositionWithCollisionSPH(){};

      void init() {};
      void solve(const double dt);

    private:
      PSYS PQ;
      CollisionSurface C;
 };


// --------------------------------------------------

// --------------------------------------------------
// VELOCITY PARTIAL SOLVER FOR SPH
// This solver updates the velocity of particles
// based on SPH forces like viscosity and pressure.
// There are user defined values that need to be
// defined for SPH systems, thus they are stored here
 class AdvanceVelocitySPH : public GISolverBase
 {
    public:
      AdvanceVelocitySPH(PSYS& pq, Force& f, const double userArray[6]);
      ~AdvanceVelocitySPH(){};

      void init() {};
      void solve(const double dt);

    private:
      PSYS PQ;
      Force force;
      double Pbar, rhoBar, gamma, alpha, beta, eps; // These are user-defined variables

 };

 //---------------------------------------------------
 // VELOCITY PARTIAL SOLVER WITH FORCE
 // This solver updates velocity bsed on some force
 // acting on the system. Force is defined in MyThing
 // and passed in. This is used when there is only one
 // force acting on the system
 class AdvanceVelocity : public GISolverBase
{
  public:
  
    AdvanceVelocity(PSYS& pq, Force& f);
    ~AdvanceVelocity() {};

    void init() {};
    void solve(const double dt);

  private:

    PSYS PQ;
    Force force;

};

//----------------------------------------------------

//----------------------------------------------------
// COMPOSITE SOLVERS
// - LeapFrog: Solving position at dt/2, velocity at dt,
//   and position at dt/2
// - Sixth order solver: Does 5 calls to a given solver
//   at different time steps
// - ForwardEuler: Solving position then velocity
// - BackwardEuler: Solving velocity then position
 
class SixthOrderSolver : public GISolverBase
{
  public:

    SixthOrderSolver( GISolver& s) :
      solver(s)
      {
        a = 1.0/(4.0 - std::pow(4.0, 1.0/3.0));
        b = 1.0 - 4.0*a;
      }
    
    ~SixthOrderSolver(){}

    void init(){ solver->init(); }

    void solve(const double dt) {
      const double dta = a * dt;
      const double dtb = b * dt;
      solver->solve(dta);
      solver->solve(dta);
      solver->solve(dtb);
      solver->solve(dta);
      solver->solve(dta);
    }

    private:
      GISolver solver;
      double a, b;

};
class LeapFrogSolver : public GISolverBase
 {
   public:
  
     LeapFrogSolver( GISolver& A, GISolver&  B ) :
       a (A),
       b (B)
     {}
  
     ~LeapFrogSolver(){}
  
     void init(){ a->init(); b->init(); }
  
     void solve( const double dt )
     {
        const double dtd2 = 0.5*dt;
        a->solve(dtd2);
        b->solve(dt);
        a->solve(dtd2);
     }
  
   private:
  
     GISolver a;
     GISolver b;
 };
  
 class ForwardEulerSolver : public GISolverBase
 {
   public:
  
     ForwardEulerSolver( GISolver& A, GISolver& B ) :
       a (A),
       b (B)
     {}
  
     ~ForwardEulerSolver(){}
  
     void init(){ a->init(); b->init(); }
  
     void solve( const double dt )
     {
        a->solve(dt);
        b->solve(dt);
     }
  
   private:
  
     GISolver a;
     GISolver b;
 };
 
 class BackwardEulerSolver : public GISolverBase
 {
   public:
  
     BackwardEulerSolver( GISolver& A, GISolver& B ) :
       a (A),
       b (B)
     {}
  
     ~BackwardEulerSolver(){}
  
     void init(){ a->init(); b->init(); }
  
     void solve( const double dt )
     {
        b->solve(dt);
        a->solve(dt);
     }
  
   private:
  
     GISolver a;
     GISolver b;
 };

 //------------------------------------------------------
  

 // Create solver functions to create smart pointers of each solver
 GISolver CreateSixthOrderSolver(GISolver& s);
 GISolver CreateLeapFrogSolver( GISolver& A, GISolver&  B );
 GISolver CreateForwardEulerSolver( GISolver& A, GISolver& B );
 GISolver CreateBackwardEulerSolver( GISolver& A, GISolver& B);
 GISolver CreateAdvancePositionStarter(PSYS& pq);
 GISolver CreateAdvanceVelocityStarter(PSYS& pq);
 GISolver CreateAdvancePosition(PSYS& pq);
 GISolver CreateAdvanceVelocity(PSYS& pq, Force& f);
 GISolver CreateAdvancePositionWithCollision(PSYS& pq, CollisionSurface& c);
 GISolver CreateAdvancePositionWithCollisionSPH(PSYS& pq, CollisionSurface& c);
 GISolver CreateAdvanceVelocitySPH(PSYS& pq, Force& f, const double userArray[6]);
  

 // Utility Functions
 // ------------------------------------------------------
 
 // Calculates the speed of sound using user defined values
 double CalcSpeedOfSound(const double Pbar, const double rhoBar, const double gamma, const double density);
  


}
 #endif
