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

    // Load in a model and create collision surface
    //  CollisionSurf->MakeBox(3);

    // Create a SBD system object to hold particles and interact with them
    MyThing_PSYS = CreateRigidBody("My_First_RigidBody_System");
 
    // Create a Force objects
    // Force GForce = CreateGravityForce(Vector(0, -1, 0));

    // Create a Force object that is an accumulating force and add all forces
    //  accumulator = CreateAccumulatingForce();
    //  std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(accumulator);
    //  f->AddForce(GForce);

    // Create two partial solvers and set the initial solver to the sixth order solver
    GISolver solverA = CreateAdvancePositionRBD(MyThing_PSYS);
    solverB = CreateAdvanceVelocityRBD(MyThing_PSYS);
    GISolver LFSolver = CreateLeapFrogSolver(solverA, solverB);
    solver = CreateSixthOrderSolver(LFSolver);

    // Seed rand with time
    srand(time(NULL));

    // Reset the particle system and start ball bounces
    Reset(); 

}
    
void MyThing::Display() 
{

   // Need to cast PSYS to RBD
   std::shared_ptr<RigidBodySystem> rbd = std::dynamic_pointer_cast<RigidBodySystem>(MyThing_PSYS);

   // Cull any front faces
   glEnable(GL_CULL_FACE);
   // glCullFace(GL_FRONT);

   // Displays all sides of the surface with their specified color
   // CollisionSurf->Display();

   // Display particles
   glPointSize(5.0);
   glBegin(GL_POINTS);
   for( size_t i=0;i<MyThing_PSYS->Psize();i++ )
   {
      const Vector& P = rbd->RBD_pos(i);
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
   // Generate particles based on model vertices and then create pairs between each particle
   MyThing_PSYS->Pclear();
   MyThing_PSYS->GenParticlesFromModel("./misc/models/smallsphere.obj");

   // Need to cast PSYS to RBD
   std::shared_ptr<RigidBodySystem> rbd = std::dynamic_pointer_cast<RigidBodySystem>(MyThing_PSYS);
   rbd->ComputeRBDData();

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



