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
       if( key == 'e' ){ emit = !emit; }
}


void MyThing::solve()
{
   // This is where the particle state - position and velocity - are updated
   // in several steps.

   // Step 1: Advance the particle positions in time
   for(size_t i=0;i<MyThing_PSYS->Psize();i++)
   {
      Vector P = MyThing_PSYS->GetPos(i);
      Vector V = MyThing_PSYS->GetVel(i);

      P += V * dt;

      MyThing_PSYS->SetPos(i, P);
   }

   //////////////////////////////////////////////////////////////////////////////////////////
   //
   //           THIS PART IS NOT A TYPICAL VELOCITY UPDATE
   //
   // Next the velocity is updated.  For most situations, this update comes
   // from forces on the particles. Here we do a simple, non-force, example.

   // Step 2.1: find the center
   Vector center;
   for(size_t i=0;i<MyThing_PSYS->Psize();i++)
   {
      center += MyThing_PSYS->GetPos(i);
   }
   center = center/MyThing_PSYS->Psize();

   // Step 2.2: update velocities to be perpendicular to the line from the particle to the center
   for(size_t i=0;i<MyThing_PSYS->Psize();i++)
   {
      Vector n = MyThing_PSYS->GetPos(i) - center;
      n.normalize();  // make it a unit vector
      double vmag = MyThing_PSYS->GetVel(i).magnitude();

      Vector V = MyThing_PSYS->GetVel(i);

      V -= n*(n*V);
      V *= vmag/V.magnitude();

      MyThing_PSYS->SetVel(i, V);

   }
   //
   //
   //////////////////////////////////////////////////////////////////////////////////////////////////

   // This concludes the solver action
   //
   //
   //
   //
   // This is where we can add more particles
   if(emit)
   {
      size_t nbincrease = 10;
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
}

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
   cout << "e            toggle particle emission on/off\n";
}


pba::PbaThing pba::CreateMyThing() { return PbaThing( new MyThing() ); }



