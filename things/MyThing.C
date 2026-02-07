//-------------------------------------------------------
//
//  MyThing.C
//
//  PbaThing for a collection of particles to simulate
//  gravity
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
#include <time.h>

#include "ParticleSystem.h"
#include "CollisionSurface.h"
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

    // Create a particle system object to hold particles and interact with them
    MyThing_PSYS = CreateParticleSystem("My_First_Particle_System");

    // Create a Force object that is a gravity force
    Force GForce = CreateGravityForce(Vector(0, -1, 0));

    // Create a Force object that is an accumulating force
    accumulator = CreateAccumulatingForce();
    std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(accumulator);
    f->AddForce(GForce);

    // Create a CollisionSurface object to hold triangles to collide with
    Box = MakeCollisionSurface();

    // MakeBox creates a box with 12 triangles
    Box->MakeBox(3);

    // Create two partial solvers and set the initial solver to forward euler
    GISolver solverA = CreateAdvancePositionWithCollision(MyThing_PSYS, Box);
    GISolver solverB = CreateAdvanceVelocity(MyThing_PSYS, accumulator);
    GISolver LFSolver = CreateLeapFrogSolver(solverA, solverB);
    solver = CreateSixthOrderSolver(LFSolver);
    
    // Seed rand with time
    srand(time(NULL));

    // Reset the particle system and start ball bounces
    Reset(); 

}
    
void MyThing::Display() 
{

   // Cull any front faces
   glEnable(GL_CULL_FACE);
   glCullFace(GL_FRONT);

   // Displays all sides of the box with their specified color
   Box->Display();

   // Display particles
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
      // Keyboard presses specific to MyThing; self explanatory
      PbaThingyDingy::Keyboard(key,x,y);
      if( key == 'e' ){ Emit(); }
      if( key == 'g'){
         std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
         std::shared_ptr<GravityForce> g = dynamic_pointer_cast<GravityForce>(a->GetForce(0));
         g->DecreaseGravityForce();
         cout << "Current gravity magnitude: " << g->GetGravityMag() << "\n";
      } 
      if( key == 'G'){
         std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
         std::shared_ptr<GravityForce> g = dynamic_pointer_cast<GravityForce>(a->GetForce(0));
         g->IncreaseGravityForce();
         cout << "Current gravity magnitude: " << g->GetGravityMag() << "\n";
      }
      if( key == 'c'){
         Box->DecreaseCoeffR();
         cout << "Current coefficient of restitution: " << Box->GetCoeffR() << "\n";
      }
      if( key == 'C'){
         Box->IncreaseCoeffR();
         cout << "Current coefficient of restitution: " << Box->GetCoeffR() << "\n";
      }

}


void MyThing::solve() { solver->solve(dt); }

void MyThing::Reset()
{
   // Create 1 particle with a random position, velocity, and color
   InitPos = Vector(rand() % 3 - 1.5, rand() % 3 - 1.5, rand() % 3 - 1.5);
   MyThing_PSYS->Pclear();
   MyThing_PSYS->AddParticles(1);
   for(size_t i=0;i<MyThing_PSYS->Psize();i++)
   {
      pba::Color inCol  = pba::Color(drand48(),drand48(),drand48(),0);
      pba::Vector inVec = pba::Vector(drand48()-0.5,drand48()-0.5,drand48()-0.5);
   
      MyThing_PSYS->SetPos(i, InitPos);
      MyThing_PSYS->SetVel(i, inVec);
      MyThing_PSYS->SetCol(i, inCol);
   }
}

void MyThing::Usage()
{
   PbaThingyDingy::Usage();
   cout << "=== " << name << " ===\n";
   cout << "e            Create 100 new particles\n";
   cout << "g            Decrease magnitude of gravity\n";
   cout << "G            Increase magnitude of gravity\n";
   cout << "c            Decrease coefficient of restitution\n";
   cout << "C            Increase coefficient of restitution\n";
}

void MyThing::Emit() {
   
   // Emit 100 new particles at the initial position with random colors and velocities
   size_t nbincrease = 100;
   MyThing_PSYS->AddParticles(nbincrease);
   Vector V;
   Color C;
   std::cout << "Total Points " << MyThing_PSYS->Psize() << std::endl;
   for(size_t i=MyThing_PSYS->Psize()-nbincrease;i<MyThing_PSYS->Psize();i++)
   {
      
      pba::Color inCol  = pba::Color(drand48(),drand48(),drand48(),0);
      pba::Vector inVec = pba::Vector(drand48()-0.5,drand48()-0.5,drand48()-0.5);
   
      MyThing_PSYS->SetPos(i, InitPos);
      MyThing_PSYS->SetVel(i, inVec);
      MyThing_PSYS->SetCol(i, inCol);
   }
   
}


pba::PbaThing pba::CreateMyThing() { return PbaThing( new MyThing() ); }



