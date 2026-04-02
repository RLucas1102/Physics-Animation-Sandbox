

OFILES = base/PbaViewer.o \
         base/PbaThing.o \
         base/Matrix.o \
         base/LinearAlgebra.o \
		 base/OccupancyVolume.o \
		 base/ParticleSystem.o\
		 base/SPHSystem.o\
		 base/SoftBodySystem.o\
		 base/RigidBodySystem.o\
		 base/Edge.o\
		 base/Force.o\
		 base/Torque.o\
		 base/CollisionTriangle.o\
		 base/CollisionSurface.o\
		 base/GISolver.o \
	 things/MyThing.o 





ROOTDIR = .
LIB = $(ROOTDIR)/lib/libpba.a 
GLLDFLAGS     = -lglut -lGL -lm -lGLU
CXX = g++ -Wall -g -O0 -fPIC $(DEFINES) -fopenmp -std=c++11
INCLUDES =  -I ./include/ -I /usr/local/include/ -I/usr/include/ -I ./things



.C.o: 
	$(CXX) -c $(INCLUDES) $< -o $@

base: $(OFILES)
	ar rv $(LIB) $?

clean:
	rm -rf *.o base/*.o base/*~ include/*~  things/*~ core $(LIB) *~ pbalitesim things/*.o bin/pbalitesim

sim:	$(OFILES)
	make base
	$(CXX) things/pbalitesim.C  $(INCLUDES) -ldl -L./lib -lpba $(GLLDFLAGS)  -o bin/pbalitesim


