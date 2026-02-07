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
 #include <iostream>
  
  
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

 // Create a smart pointer to gravity force
 Force CreateGravityForce(const Vector& g);
  
  
 }
 #endif