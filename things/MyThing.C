//-------------------------------------------------------
//
//  MyThing.C
//
//  PbaThing for a collection of particles
//  each doing a random walk.
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------

#include "MyThing.h"
#include <cstdlib>
#include <GL/gl.h>   // OpenGL itself.
#include <GL/glu.h>  // GLU support library.
#include <GL/glut.h> // GLUT support library.
#include <iostream>

#include "ParticleSystem.h"
#include "CollisionTriangle.h"
#include "GISolver.h"



using namespace std;

using namespace pba;


MyThing::MyThing(const std::string nam) :
 PbaThingyDingy (nam),
 emit       (false)
{
    std::cout << name << " constructed\n";
}

MyThing::~MyThing(){}

void MyThing::Init( const std::vector<std::string>& args ) {

    MyThing_PSYS = CreateParticleSystem("My_First_Particle_System");

    GForce = CreateGravityForce(Vector(0, -1, 0));

    Box = MakeCollisionSurface();

    Vector p0 = Vector(1.0, -1.0, 1.0);
    Vector p1 = Vector(1.0, -1.0, -1.0);
    Vector p2 = Vector(-1.0, -1.0, 1.0);

    CollisionTriangle t = MakeCollisionTriangle(p0, p1, p2);

    Box->AddTriangle(t);

    p0 = Vector(1.0, -1.0, -1.0);
    p1 = Vector(-1.0, -1.0, -1.0);     
    p2 = Vector(-1.0, -1.0, 1.0);

    t = MakeCollisionTriangle(p0, p1, p2);
    
    Box->AddTriangle(t);

    GISolver solverA = CreateAdvancePositionWithCollision(MyThing_PSYS, Box);
    GISolver solverB = CreateAdvanceVelocity(MyThing_PSYS, GForce);
    solver = CreateForwardEulerSolver(solverA, solverB);

    Reset(); 

}
    
void MyThing::Display() 
{

   Box->Display();

   glPointSize(5.0);
   glBegin(GL_POINTS);
   for( size_t i=0;i<MyThing_PSYS->Psize();i++ )
   {
      const Vector& P = MyThing_PSYS->GetPos(i);
      const Color& ci = MyThing_PSYS->GetCol(i);
      glColor3f( ci.red(), ci.green(), ci.blue() );
      glVertex3f( P.X(), P.Y(), P.Z() );
   }
   glEnd();
}

void MyThing::Keyboard( unsigned char key, int x, int y )
{
      PbaThingyDingy::Keyboard(key,x,y);
      if( key == 'e' ){ Emit(); }
      if( key == 'b' ){ 
         GISolver solverA = CreateAdvancePositionWithCollision(MyThing_PSYS, Box);
         GISolver solverB = CreateAdvanceVelocity(MyThing_PSYS, GForce);
         solver = CreateBackwardEulerSolver(solverA, solverB);
         cout << "Using Backward Euler Solver\n";
      }
      if( key == 'B' ){ 
         GISolver solverA = CreateAdvancePositionWithCollision(MyThing_PSYS, Box);
         GISolver solverB = CreateAdvanceVelocity(MyThing_PSYS, GForce);
         solver = CreateForwardEulerSolver(solverA, solverB);
         cout << "Using Forward Euler Solver\n";
      }
      if( key == 'L' ){ 
         GISolver solverA = CreateAdvancePositionWithCollision(MyThing_PSYS, Box);
         GISolver solverB = CreateAdvanceVelocity(MyThing_PSYS, GForce);
         solver = CreateLeapFrogSolver(solverA, solverB);
         cout << "Using Leap Frog Solver\n";
      }
      if( key == 'g'){
         std::shared_ptr<GravityForce> g = dynamic_pointer_cast<GravityForce>(GForce);
         g->DecreaseGravityForce();
      } 
      if( key == 'G'){
         std::shared_ptr<GravityForce> g = dynamic_pointer_cast<GravityForce>(GForce);
         g->IncreaseGravityForce();
      }

}


void MyThing::solve() { solver->solve(dt); }

void MyThing::Reset()
{
   // Distribute particles with random positions
   MyThing_PSYS->Pclear();
   MyThing_PSYS->AddParticles(20);
   for(size_t i=0;i<MyThing_PSYS->Psize();i++)
   {
      pba::Vector P(0, 0, 0);
      pba::Color inCol  = pba::Color(drand48(),drand48(),drand48(),0);
      pba::Vector inVec = pba::Vector(drand48()-0.5,drand48()-0.5,drand48()-0.5);
   
      MyThing_PSYS->SetPos(i, P);
      MyThing_PSYS->SetVel(i, inVec);
      MyThing_PSYS->SetCol(i, inCol);
   }
}

void MyThing::Usage()
{
   PbaThingyDingy::Usage();
   cout << "=== " << name << " ===\n";
   cout << "e            Create 100 new particles\n";
   cout << "b            Use Backward Euler Solver\n";
   cout << "B            Use Forward Euler Solver\n";
   cout << "L            Use Leap Frog Solver\n";
}

void MyThing::Emit() {
      
   size_t nbincrease = 100;
   MyThing_PSYS->AddParticles(nbincrease);
   Vector P, V;
   Color C;
   std::cout << "Total Points " << MyThing_PSYS->Psize() << std::endl;
   for(size_t i=MyThing_PSYS->Psize()-nbincrease;i<MyThing_PSYS->Psize();i++)
   {
      
      pba::Vector P(0, 0, 0);
      pba::Color inCol  = pba::Color(drand48(),drand48(),drand48(),0);
      pba::Vector inVec = pba::Vector(drand48()-0.5,drand48()-0.5,drand48()-0.5);
   
      MyThing_PSYS->SetPos(i, P);
      MyThing_PSYS->SetVel(i, inVec);
      MyThing_PSYS->SetCol(i, inCol);
   }
   
}


pba::PbaThing pba::CreateMyThing() { return PbaThing( new MyThing() ); }



