

#include "PbaViewer.h"
#include "MyThing.h"



int main(int argc, char** argv)
{
   // Set up command line arguments, if any
   std::vector<std::string> args;
   for(int i=0;i<argc;i++)
   {
      args.push_back( argv[i] );
   }

   // Instantiate a viewer
   pba::PbaViewer* viewer = pba::CreateViewer();
 
   // Set up a simulation thing
   pba::PbaThing mything = pba::CreateMyThing();
   viewer->AddThing(mything);

   // Initialize viewer
   viewer->Init(args);
   // Run the (GLUT) main loop
   viewer->MainLoop();

}
