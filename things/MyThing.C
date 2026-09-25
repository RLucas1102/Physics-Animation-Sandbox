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
    CollisionSurf = MakeCollisionSurface();
    CollisionSurf->MakeBox(3);

    // Create a SBD system object to hold particles and interact with them
    MyThing_PSYS = CreateFlock("My_First_Flocking_System");
    Flock flock = std::dynamic_pointer_cast<FlockingSystem>(MyThing_PSYS);
    flock->SetKca(1);
    flock->SetKm(3);
    flock->SetKc(4);
    flock->SetR(1);
    flock->SetRamp(0.01);
    flock->SetTheta(1);
    flock->SetThetaRamp(0.01);

    // Create a Force objects
    Force FForce = CreateFlockingForce(50);

    // Create a Force object that is an accumulating force and add all forces
     accumulator = CreateAccumulatingForce();
     std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(accumulator);
     f->AddForce(FForce);

    // Create two partial solvers and set the initial solver to the sixth order solver
    GISolver solverA = CreateAdvancePositionWithCollision(MyThing_PSYS, CollisionSurf);
    solverB = CreateAdvanceVelocity(MyThing_PSYS, accumulator);
    solver = CreateLeapFrogSolver(solverA, solverB);
    //  GISolver LFSolver = CreateLeapFrogSolver(solverA, solverB);
    //  solver = CreateSixthOrderSolver(LFSolver);

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

   // Displays all sides of the surface with their specified color
   CollisionSurf->Display();

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
   if( key == 'c'){
      CollisionSurf->DecreaseCoeffR();
      cout << "Current coefficient of restitution: " << CollisionSurf->GetCoeffR() << "\n";
   }
   if( key == 'C'){
      CollisionSurf->IncreaseCoeffR();
      cout << "Current coefficient of restitution: " << CollisionSurf->GetCoeffR() << "\n";
   }
   if( key == 's'){
      CollisionSurf->DecreaseCoeffS();
      cout << "Current coefficient of sticky: " << CollisionSurf->GetCoeffS() << "\n";
   }
   if( key == 'S'){
      CollisionSurf->IncreaseCoeffS();
      cout << "Current coefficient of sticky: " << CollisionSurf->GetCoeffS() << "\n";
   }
   if (key == 'v') {
      Flock f = dynamic_pointer_cast<FlockingSystem>(MyThing_PSYS);
      double Kca = f->GetKca() - 0.1;
      f->SetKca(Kca);
      cout << "Current collision avoidance: " << Kca << "\n";
   }
   if (key == 'V') {
      Flock f = dynamic_pointer_cast<FlockingSystem>(MyThing_PSYS);
      double Kca = f->GetKca() + 0.1;
      f->SetKca(Kca);
      cout << "Current collision avoidance: " << Kca << "\n";
   }
   if (key == 'b') {
      Flock f = dynamic_pointer_cast<FlockingSystem>(MyThing_PSYS);
      double Km = f->GetKm() - 0.1;
      f->SetKm(Km);
      cout << "Current velocity matching: " << Km << "\n";
   }
   if (key == 'B') {
      Flock f = dynamic_pointer_cast<FlockingSystem>(MyThing_PSYS);
      double Km = f->GetKm() + 0.1;
      f->SetKm(Km);
      cout << "Current velocity matching: " << Km << "\n";
   }
   if (key == 'n') {
      Flock f = dynamic_pointer_cast<FlockingSystem>(MyThing_PSYS);
      double Kc = f->GetKc() - 0.1;
      f->SetKc(Kc);
      cout << "Current centering: " << Kc << "\n";
   }
   if (key == 'N') {
      Flock f = dynamic_pointer_cast<FlockingSystem>(MyThing_PSYS);
      double Kc = f->GetKc() + 0.1;
      f->SetKc(Kc);
      cout << "Current centering: " << Kc << "\n";
   }
   
}


void MyThing::solve() { solver->solve(dt); }

void MyThing::Reset()
{
   // Create 1000 particle with a random position, velocity, and color
   MyThing_PSYS->Pclear();
   MyThing_PSYS->AddParticles(1000);
   for(size_t i=0;i<MyThing_PSYS->Psize();i++)
   {
      pba::Color inCol  = pba::Color(drand48(),drand48(),drand48(),0);
      pba::Vector inVec = pba::Vector(drand48()-0.5,drand48()-0.5,drand48()-0.5);
      pba::Vector inPos = Vector(drand48() * 5 - 2.5,drand48() * 5 - 2.5,drand48() * 5 - 2.5);
   
      MyThing_PSYS->SetPos(i, inPos);
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
   cout << "s            Decrease coefficient of sticky\n";
   cout << "S            Increase coefficient of sticky\n";
   cout << "v            Decrease collision avoidance\n";
   cout << "V            Increase collision avoidance\n";
   cout << "b            Decrease velocity matching\n";
   cout << "B            Increase velocity matching\n";
   cout << "n            Decrease centering\n";
   cout << "N            Increase centering\n";

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



