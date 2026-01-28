//*******************************************************************
 //
 //   GISolver.h
 //
 // Base class for Geometric Integration solvers
 //
 //  Copyright (c) 2017 Jerry Tessendorf
 //
 //
 //*******************************************************************
  
 #ifndef __PBA_GISOLVER_H__
 #define __PBA_GISOLVER_H__
  
 #include <cmath>
 #include <memory>

 #include "ParticleSystem.h"
 #include "Force.h"
  
 namespace pba{
  
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
  
  
 GISolver CreateLeapFrogSolver( GISolver& A, GISolver&  B );
 GISolver CreateForwardEulerSolver( GISolver& A, GISolver& B );
 GISolver CreateBackwardEulerSolver( GISolver& A, GISolver& B);
 GISolver CreateAdvancePositionStarter(PSYS& pq);
 GISolver CreateAdvanceVelocityStarter(PSYS& pq);
 GISolver CreateAdvancePosition(PSYS& pq);
 GISolver CreateAdvanceVelocity(PSYS& pq, Force& f);

  
 }
  
  
 #endif
