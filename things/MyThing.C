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
#include "OccupancyVolume.h"

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
    
    // Create occupancy grid
    double h = 0.075;
    double R = h;
    Vector llc = Box->GetLLC();
    Vector urc = Box->GetURC();

    OV occVol = CreateOccupancyVolume(llc, urc, R);
    occVol->ComputeNeighbors();

    // Create a particle system object to hold particles and interact with them
    MyThing_PSYS = CreateParticleSystem("My_First_Particle_System");

    // Create parameters for SPH forces (these will vary)
    double Pbar = 2.0;
    double rhoBar = 5.5;
    double gamma = 3.5;
    double alpha = -10.0;
    double beta = 1.0;
    double eps = 1.0;
    MyThing_PSYS->SetH(h);

    // Create SPH forces
    Force VForce = CreateViscosityForce(Pbar, rhoBar, gamma, alpha, beta, eps, occVol);
    Force PForce = CreatePressureForce(Pbar, rhoBar, gamma, occVol);

    // Create a Force object that is a gravity force
    Force GForce = CreateGravityForce(Vector(0, -1, 0));

    // Create a Force object that is an accumulating force
    accumulator = CreateAccumulatingForce();
    std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(accumulator);
    f->AddForce(GForce);
    f->AddForce(PForce);
    f->AddForce(VForce);

    // Create two partial solvers and set the initial solver to forward euler
    GISolver solverA = CreateAdvancePositionWithCollisionSPH(MyThing_PSYS, Box, occVol);
    solverB = CreateAdvanceVelocitySPH(MyThing_PSYS, accumulator);
    GISolver LFSolver = CreateLeapFrogSolver(solverA, solverB);
    solver = CreateSixthOrderSolver(LFSolver);

    std::shared_ptr<AdvanceVelocitySPH> vs = dynamic_pointer_cast<AdvanceVelocitySPH>(solverB);
    vs->SetVT(5.0);
    vs->SetAT(5.0);

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
      if (key == 'v') {
         std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
         std::shared_ptr<ViscosityForce> v = dynamic_pointer_cast<ViscosityForce>(a->GetForce(2));
         v->ChangeVStrength(-0.1);
         cout << "Current viscosity strength: " << v->GetStrength() << "\n";
      }
      if (key == 'V') {
         std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
         std::shared_ptr<ViscosityForce> v = dynamic_pointer_cast<ViscosityForce>(a->GetForce(2));
         v->ChangeVStrength(0.1);
         cout << "Current viscosity strength: " << v->GetStrength() << "\n";
      }
      if (key == 'p') {
         std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
         std::shared_ptr<PressureForce> p = dynamic_pointer_cast<PressureForce>(a->GetForce(1));
         p->ChangePStrength(-0.1);
         cout << "Current pressure strength: " << p->GetPStrength() << "\n";
      }
      if (key == 'P') {
         std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
         std::shared_ptr<PressureForce> p = dynamic_pointer_cast<PressureForce>(a->GetForce(1));
         p->ChangePStrength(0.1);
         cout << "Current pressure strength: " << p->GetPStrength() << "\n";
      }
      if (key == 'd') {
         std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
         std::shared_ptr<PressureForce> p = dynamic_pointer_cast<PressureForce>(a->GetForce(1));
         p->ChangeBaseDensity(-0.1);
         cout << "Current base density: " << p->GetBaseDensity() << "\n";
      }
      if (key == 'D') {
         std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
         std::shared_ptr<PressureForce> p = dynamic_pointer_cast<PressureForce>(a->GetForce(1));
         p->ChangeBaseDensity(0.1);
         cout << "Current base density: " << p->GetBaseDensity() << "\n";
      }
      if (key == 'b') {
         std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
         std::shared_ptr<PressureForce> p = dynamic_pointer_cast<PressureForce>(a->GetForce(1));
         p->ChangePower(-0.1);
         cout << "Current power: " << p->GetPower() << "\n";
      }
      if (key == 'B') {
         std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
         std::shared_ptr<PressureForce> p = dynamic_pointer_cast<PressureForce>(a->GetForce(1));
         p->ChangePower(0.1);
         cout << "Current power: " << p->GetPower() << "\n";
      }
      if (key == 's') {
         std::shared_ptr<AdvanceVelocitySPH> vs = dynamic_pointer_cast<AdvanceVelocitySPH>(solverB);
         vs->ChangeVDampening(-0.1);
         cout << "Current velocity dampening: " << vs->GetVT() << "\n";
      }
      if (key == 'S') {
         std::shared_ptr<AdvanceVelocitySPH> vs = dynamic_pointer_cast<AdvanceVelocitySPH>(solverB);
         vs->ChangeVDampening(0.1);
         cout << "Current velocity dampening: " << vs->GetVT() << "\n";
      }
      if (key == 'a') {
         std::shared_ptr<AdvanceVelocitySPH> vs = dynamic_pointer_cast<AdvanceVelocitySPH>(solverB);
         vs->ChangeADampening(-0.1);
         cout << "Current acceleration dampening: " << vs->GetAT() << "\n";
      }
      if (key == 'A') {
         std::shared_ptr<AdvanceVelocitySPH> vs = dynamic_pointer_cast<AdvanceVelocitySPH>(solverB);
         vs->ChangeADampening(0.1);
         cout << "Current acceleration dampening: " << vs->GetAT() << "\n";
      }

}


void MyThing::solve() { solver->solve(dt); }

void MyThing::Reset()
{
   // Create 1000 particles with a random position, velocity of 0, and random color
   Vector initVel = Vector(0, 0, 0);
   MyThing_PSYS->Pclear();
   MyThing_PSYS->AddParticles(1000);
   for(size_t i=0;i<MyThing_PSYS->Psize();i++)
   {
      pba::Color inCol  = pba::Color(drand48(),drand48(),drand48(),0);
      pba::Vector inPos = pba::Vector(drand48() * 5 - 2.5,drand48() * 5 - 2.5,drand48() * 5 - 2.5);
      
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
   cout << "c            Decrease coefficient of restitution\n";
   cout << "C            Increase coefficient of restitution\n";
   cout << "v            Decrease viscosity strength\n";
   cout << "V            Increase viscosity strength\n";
   cout << "p            Decrease pressure strength\n";
   cout << "P            Increase pressure strength\n";
   cout << "d            Decrease base density\n";
   cout << "D            Increase base density\n";
   cout << "b            Decrease power\n";
   cout << "B            Increase power\n";
   cout << "s            Decrease velocity dampening\n";
   cout << "S            Increase velocity dampening\n";
   cout << "a            Decrease acceleration dampening\n";
   cout << "A            Increase acceleration dampening\n";

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



