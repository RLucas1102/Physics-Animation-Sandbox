 //-------------------------------------------------------
 //
 //  Force.h
 //
 //  Base class and refcounted pointer for forces
 //
 //  Copyright (c) 2017 Jerry Tessendorf
 //
 //  Adapted by Lucas Robenolt
 //
 //--------------------------------------------------------
  
 #ifndef ____PBA_FORCE_H____
 #define ____PBA_FORCE_H____
  
 #include "ParticleSystem.h"
 #include "Vector.h"
 #include "OccupancyVolume.h"
 #include <iostream>
 #include <vector>
  
  
 namespace pba
 {
  
 // Base Force class that all Forces will inherit from
 class ForceBase
 {
   public:
  
     ForceBase(){};
  
     virtual void compute( PSYS& psys, const double dt ){std::cout << "calling ForceBase::compute(PSYS, double) base class virtual method\n"; }
     virtual ~ForceBase(){};
  
 };
  
 typedef std::shared_ptr<ForceBase> Force;

 //------------------------------------------------
 // ACCUMULATING FORCE
 // Computes all forces acting on a system by
 // tracking a vector of forces and iterating
 // through the vector to calculate the 
 // cumulative force of all forces in
class AccumulatingForce : public ForceBase {
    
    public:
        AccumulatingForce(){};
        ~AccumulatingForce(){};

        void compute( PSYS& psys, const double dt);

        // Build up the collection of forces to accumulate
        void AddForce( Force& f);

        // Get Force from accumulator
        Force GetForce(const size_t i); 

    private:
        std::vector<Force> forces;


};
//-------------------------------------------------

 //------------------------------------------------
 // GRAVITY FORCE
 // Computes a new acceleration for each a particle
 // based on the current set gravity
 class GravityForce : public ForceBase {

    public: 
        GravityForce( const Vector& g);

        ~GravityForce(){}

        void compute( PSYS& psys, const double dt);

        void IncreaseGravityForce();

        void DecreaseGravityForce();

        double GetGravityMag();

    private:
        Vector gravity;
 };

//-------------------------------------------------

//-------------------------------------------------
// VISCOSITY FORCE
// Computes a new acceleration for each particle
// based on the calculated viscosity for SPH sim
class ViscosityForce : public ForceBase {

    public:
        ViscosityForce(const double Pbar, const double rhoBar, const double gamma,
                       const double alpha, const double beta, const double eps,
                       const OV& o);
        ~ViscosityForce(){};

        void compute(PSYS& psys, const double dt);

    private:
        double _Pbar, _rhoBar, _gamma, _alpha, _beta, _eps;
        OV O;
};

//------------------------------------------------

//------------------------------------------------
// PRESSURE FORCE
// Computes a new acceleration for each particle
// based on the calculated pressure for SPH sim
class PressureForce : public ForceBase {

    public:
        PressureForce(const double Pbar, const double rhoBar, const double gamma, const OV& o);
        ~PressureForce(){};

        void compute(PSYS& psys, const double dt);

    private:
        double _Pbar, _rhoBar, _gamma;
        OV O;
};

//------------------------------------------------

 // Create a smart pointer to forces
 Force CreateGravityForce(const Vector& g);
 Force CreateAccumulatingForce();
 Force CreateViscosityForce(const double Pbar, const double rhoBar, const double gamma,
                            const double alpha, const double beta, const double eps, const OV& o);
 Force CreatePressureForce(const double Pbar, const double rhoBar, const double gamma, const OV& o);
  


 // Utility Functions
 // ------------------------------------------------------
 // Many of these functions are used to calculate specific values for pressure and viscosity in SPH forces
 double CalcSpeedOfSound(const double Pbar, const double rhoBar, const double gamma, const double density);
 double CalcMuab(const double h, Vector& P1, Vector& P2, Vector& V1, Vector& V2, const double eps); 
 double CalcPiab(const double alpha, const double beta, const double C, const double muab, const double densitya, const double densityb);
 double CalcWeightKernel(Vector& P, const double h);
 Vector CalcGradWeightKernel(Vector& P, const double h);
 double CalcTaitEquation(const double rhoBar, const double Pbar, const double gamma, const double density);

 }
 #endif