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

   // Set soft body properties (spring and friction constant)
    double ks = 20;
    double kf = 0.1;

    // Create a SBD system object to hold particles and interact with them
    MyThing_PSYS = CreateSoftBody("My_First_SoftBody_System");
 
    // Create a Force objects
    Force GForce = CreateGravityForce(Vector(0, -1, 0));
    Force SForce = CreateAccumulatingStrutForce(ks, kf);

    // Create a Force objects
    Force FForce = CreateFlockingForce(50);
    std::shared_ptr<FlockingForce> ff = dynamic_pointer_cast<FlockingForce>(FForce);
    ff->SetKca(1);
    ff->SetKm(3);
    ff->SetKc(4);
    ff->SetR(1);
    ff->SetRamp(0.01);
    ff->SetTheta(1);
    ff->SetThetaRamp(0.01);

    // Create a Force object that is an accumulating force and add all forces
     accumulator = CreateAccumulatingForce();
     std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(accumulator);
     f->AddForce(SForce);
     f->AddForce(FForce);
     f->AddForce(GForce);


    // Create two partial solvers and set the initial solver to the sixth order solver
    GISolver solverA = CreateAdvancePositionWithCollision(MyThing_PSYS, CollisionSurf);
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
   if( key == 'g'){
      std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
      std::shared_ptr<GravityForce> g = dynamic_pointer_cast<GravityForce>(a->GetForce(2));
      g->DecreaseGravityForce();
      cout << "Current gravity magnitude: " << g->GetGravityMag() << "\n";
   } 
   if( key == 'G'){
      std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
      std::shared_ptr<GravityForce> g = dynamic_pointer_cast<GravityForce>(a->GetForce(2));
      g->IncreaseGravityForce();
      cout << "Current gravity magnitude: " << g->GetGravityMag() << "\n";
   }
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
      std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
      std::shared_ptr<FlockingForce> ff = dynamic_pointer_cast<FlockingForce>(a->GetForce(1));
      double Kca = ff->GetKca() - 0.1;
      ff->SetKca(Kca);
      cout << "Current collision avoidance: " << Kca << "\n";
   }
   if (key == 'V') {
      std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
      std::shared_ptr<FlockingForce> ff = dynamic_pointer_cast<FlockingForce>(a->GetForce(1));
      double Kca = ff->GetKca() + 0.1;
      ff->SetKca(Kca);
      cout << "Current collision avoidance: " << Kca << "\n";
   }
   if (key == 'b') {
      std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
      std::shared_ptr<FlockingForce> ff = dynamic_pointer_cast<FlockingForce>(a->GetForce(1));
      double Km = ff->GetKm() - 0.1;
      ff->SetKm(Km);
      cout << "Current velocity matching: " << Km << "\n";
   }
   if (key == 'B') {
      std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
      std::shared_ptr<FlockingForce> ff = dynamic_pointer_cast<FlockingForce>(a->GetForce(1));
      double Km = ff->GetKm() + 0.1;
      ff->SetKm(Km);
      cout << "Current velocity matching: " << Km << "\n";
   }
   if (key == 'n') {
      std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
      std::shared_ptr<FlockingForce> ff = dynamic_pointer_cast<FlockingForce>(a->GetForce(1));
      double Kc = ff->GetKc() - 0.1;
      ff->SetKc(Kc);
      cout << "Current centering: " << Kc << "\n";
   }
   if (key == 'N') {
      std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
      std::shared_ptr<FlockingForce> ff = dynamic_pointer_cast<FlockingForce>(a->GetForce(1));
      double Kc = ff->GetKc() + 0.1;
      ff->SetKc(Kc);
      cout << "Current centering: " << Kc << "\n";
   }
   if( key == 'z'){
      std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
      std::shared_ptr<AccumulatingStrutForce> s = dynamic_pointer_cast<AccumulatingStrutForce>(a->GetForce(0));
      s->SetSpring(-0.1);
      cout << "Current Spring magnitude: " << s->GetSpring() << "\n";
   } 
   if( key == 'Z'){
      std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
      std::shared_ptr<AccumulatingStrutForce> s = dynamic_pointer_cast<AccumulatingStrutForce>(a->GetForce(0));
      s->SetSpring(0.1);
      cout << "Current Spring magnitude: " << s->GetSpring() << "\n";
   } 
   if( key == 'x'){
      std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
      std::shared_ptr<AccumulatingStrutForce> s = dynamic_pointer_cast<AccumulatingStrutForce>(a->GetForce(0));
      s->SetFriction(-0.01);
      cout << "Current Friction magnitude: " << s->GetFriction() << "\n";
   } 
   if( key == 'X'){
      std::shared_ptr<AccumulatingForce> a = dynamic_pointer_cast<AccumulatingForce>(accumulator);
      std::shared_ptr<AccumulatingStrutForce> s = dynamic_pointer_cast<AccumulatingStrutForce>(a->GetForce(0));
      s->SetFriction(0.01);
      cout << "Current Friction magnitude: " << s->GetFriction() << "\n";
   } 
   
}


void MyThing::solve() { solver->solve(dt); }

void MyThing::Reset()
{
   // Generate particles based on model vertices and then create pairs between each particle
   MyThing_PSYS->Pclear();
   MyThing_PSYS->GenParticlesFromModel("./misc/models/smallsphere.obj");

   std::shared_ptr<SoftBodySystem> s = std::dynamic_pointer_cast<SoftBodySystem>(MyThing_PSYS);
   s->ClearPairs();
   s->CreatePairs();

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
   cout << "z            Decrease magnitude of spring\n";
   cout << "Z            Increase magnitude of spring\n";
   cout << "x            Decrease magnitude of friction\n";
   cout << "X            Increase magnitude of friction\n";

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



