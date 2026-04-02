 //-------------------------------------------------------
 //
 //  TorqueLibrary.h
 //
 //  A collection of torque models
 //
 //  Copyright (c) 2017 Jerry Tessendorf
 //
 //  Adapted by Lucas Robenolt
 // 
 //--------------------------------------------------------
  
 #ifndef ____PBA_TORQUELIBRARY_H____
 #define ____PBA_TORQUELIBRARY_H____
  
 #include "Torque.h"
 #include "RigidBodySystem.h" 
 #include "Force.h"
 #include <memory>
  
 namespace pba
 {
  
  
 class TorqueFromForce
 {
   public:
  
     TorqueFromForce( Force& f );
    ~TorqueFromForce(){}
  
    void compute( PSYS& PQ, const double dt );
  
   private:
  
     Force force;
  
 };
  
 typedef std::shared_ptr<TorqueFromForce> Torque;

 Torque CreateTorqueFromForce( Force& f );  
  
  
 }
 #endif