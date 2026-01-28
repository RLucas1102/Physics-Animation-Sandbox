 //-------------------------------------------------------
 //
 //  Force.h
 //
 //  Base class and refcounted pointer for forces
 //
 //  Copyright (c) 2017 Jerry Tessendorf
 //
 //
 //--------------------------------------------------------
  
 #ifndef ____PBA_FORCE_H____
 #define ____PBA_FORCE_H____
  
 #include "ParticleSystem.h"
 #include "Vector.h"
 #include <iostream>
  
  
 namespace pba
 {
  
  
 class ForceBase
 {
   public:
  
     ForceBase(){};
  
     virtual void compute( PSYS& psys, const double dt ){std::cout << "calling ForceBase::compute(PSYS, double) base class virtual method\n"; }
     virtual ~ForceBase(){};
  
 };
  
  
  
 typedef std::shared_ptr<ForceBase> Force;
  
 class GravityForce : public ForceBase {

    public: 
        GravityForce( const Vector& g);

        ~GravityForce(){}

        void compute( PSYS& psys, const double dt);

    private:
        Vector gravity;
 };

 Force CreateGravityForce(const Vector& g);
  
  
 }
 #endif