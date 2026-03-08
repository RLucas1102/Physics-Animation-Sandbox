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

    // Create a CollisionSurface object to hold triangles to collide with
    Box = MakeCollisionSurface();

    // MakeBox creates a box with 12 triangles
    Box->MakeBox(3);
    
    // Set soft body properties (spring and friction constant)
    double ks = 1;
    double kf = 2 * std::sqrt(ks);

    // Create a SPH system object to hold particles and interact with them
    MyThing_PSYS = CreateSoftBody("My_First_SoftBody_System");
 
    // Create a Force object that is a gravity force
    Force GForce = CreateGravityForce(Vector(0, -1, 0));
    Force SForce = CreateAccumulatingStrutForce(ks, kf);

    // Create a Force object that is an accumulating force
    accumulator = CreateAccumulatingForce();
    std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(accumulator);
    f->AddForce(GForce);
    f->AddForce(SForce);

    // Create two partial solvers and set the initial solver to the sixth order solver
    GISolver solverA = CreateAdvancePositionWithCollision(MyThing_PSYS, Box);
    solverB = CreateAdvanceVelocity(MyThing_PSYS, accumulator);
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
}


void MyThing::solve() { solver->solve(dt); }

void MyThing::Reset()
{
   // Create 1000 particles with a random position, velocity of 0, and random color
   Vector initVel = Vector(0, 0, 0);
   MyThing_PSYS->Pclear();
   MyThing_PSYS->AddParticles(50);
   

   std::shared_ptr<SoftBodySystem> s = std::dynamic_pointer_cast<SoftBodySystem>(MyThing_PSYS);
   s->ClearPairs();
   s->CreatePairs();

   for(size_t i=0;i<MyThing_PSYS->Psize();i++)
   {
      pba::Color inCol  = pba::Color(drand48(),drand48(),drand48(),0);
      pba::Vector inPos = pba::Vector(drand48(),drand48(),drand48());
      
      if (i==0) {
         initPos = inPos;
      }

      MyThing_PSYS->SetPos(i, inPos);
      MyThing_PSYS->SetVel(i, initVel);
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
}

void MyThing::Emit() {
   
   // Emit 100 new particles at the initial position with random colors and velocities
   size_t nbincrease = 100;
   MyThing_PSYS->AddParticles(nbincrease);
   std::cout << "Total Points " << MyThing_PSYS->Psize() << std::endl;
   for(size_t i=MyThing_PSYS->Psize()-nbincrease;i<MyThing_PSYS->Psize();i++)
   {
      
      pba::Color inCol  = pba::Color(drand48(),drand48(),drand48(),0);
      pba::Vector inVel = pba::Vector(drand48() * 5 - 2.5,drand48() * 5 - 2.5,drand48() * 5 - 2.5);
   
      MyThing_PSYS->SetPos(i, initPos);
      MyThing_PSYS->SetVel(i, inVel);
      MyThing_PSYS->SetCol(i, inCol);
   }
   
}


pba::PbaThing pba::CreateMyThing() { return PbaThing( new MyThing() ); }



