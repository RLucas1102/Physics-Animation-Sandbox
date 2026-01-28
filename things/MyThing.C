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

    GISolver solverA = CreateAdvancePositionStarter(MyThing_PSYS);
    GISolver solverB = CreateAdvanceVelocityStarter(MyThing_PSYS);
    FESolver = CreateForwardEulerSolver(solverA, solverB);
    BESolver = CreateBackwardEulerSolver(solverA, solverB);
    LFSolver = CreateLeapFrogSolver(solverA, solverB);

    currentSolver = FESolver;

    Reset(); 

}
    
void MyThing::Display() 
{
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
       if( key == 'b' ){ currentSolver = BESolver; }
       if( key == 'B' ){ currentSolver = FESolver; }
       if( key == 'L' ){ currentSolver = LFSolver; }
}


void MyThing::solve() { currentSolver->solve(dt); }

void MyThing::Reset()
{
   // Distribute particles with random positions
   MyThing_PSYS->Pclear();
   MyThing_PSYS->AddParticles(20);
   for(size_t i=0;i<MyThing_PSYS->Psize();i++)
   {
      double s = 2.0*drand48() - 1.0;
      double ss = std::sqrt( 1.0 - s*s );
      double theta = 2.0*3.14159265*drand48();
      pba::Vector P( ss*std::cos(theta), s, ss*std::sin(theta) );
      P *= std::pow( drand48(), 1.0/6.0 );
      
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
      double s = 2.0*drand48() - 1.0;
      double ss = std::sqrt( 1.0 - s*s );
      double theta = 2.0*3.14159265*drand48();
      pba::Vector P( ss*std::cos(theta), s, ss*std::sin(theta) );
      P *= std::pow( drand48(), 1.0/6.0 );
      
      pba::Color inCol  = pba::Color(drand48(),drand48(),drand48(),0);
      pba::Vector inVec = pba::Vector(drand48()-0.5,drand48()-0.5,drand48()-0.5);
   
      MyThing_PSYS->SetPos(i, P);
      MyThing_PSYS->SetVel(i, inVec);
      MyThing_PSYS->SetCol(i, inCol);
   }
   
}


pba::PbaThing pba::CreateMyThing() { return PbaThing( new MyThing() ); }



