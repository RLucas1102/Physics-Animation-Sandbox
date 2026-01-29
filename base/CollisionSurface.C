#include "CollisionSurface.h"
#include <GL/gl.h>
#include <GL/glut.h>

using namespace pba;

CollisionSurfaceRaw::CollisionSurfaceRaw() :
    coeffR (1.0),
    coeffS (0.0)
    {}

void CollisionSurfaceRaw::AddTriangle(const CollisionTriangle& t) {
    triangles.push_back(t);
}

void CollisionSurfaceRaw::SClear() {
    triangles.clear();
}

CollisionTriangle CollisionSurfaceRaw::GetTriangle(size_t i) {
    return triangles[i];
}

void CollisionSurfaceRaw::IncreaseCoeffR() {
    coeffR *= 1.1;
}

void CollisionSurfaceRaw::DecreaseCoeffR() {
    coeffR /= 1.1;
}

double CollisionSurfaceRaw::GetCoeffR() {
    return coeffR;
}

void CollisionSurfaceRaw::handle(  const Vector& XS, const Vector& VS, 
                                    const double& dt, const Vector& XH, 
                                    const double& dtH, Vector& XR, Vector& VR,
                                    const size_t& i) const 
{

    triangles[i]->handle(XS, VS, dt, XH, dtH, XR, VR, coeffR, coeffS);

}


bool CollisionSurfaceRaw::MultiTriangleHit(const Vector &pos, const Vector &vel, 
                                                const double &dt, Vector &XH, 
                                                double &dtH, size_t &pH)
{
    bool hitFound = false;
    dtH = 2.0*dt;
    for (size_t i = 0; i < triangles.size(); i++) {
        double dtH_candidate = dtH;
        Vector XH_candidate;
        if (triangles[i]->hit(pos, vel, dt, XH_candidate, dtH_candidate)) {
            hitFound = true;
            if (fabs(dtH_candidate) < fabs(dtH)) {
                dtH = dtH_candidate;
                XH = XH_candidate;
                pH = i;
            }
        }
    }

    return hitFound;
    
}

void CollisionSurfaceRaw::Display() {

    for (size_t i = 0; i < triangles.size(); i++) {
        glBegin(GL_TRIANGLES);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3f(triangles[i]->GetP0(0), triangles[i]->GetP0(1), triangles[i]->GetP0(2));
            glColor3f(0.0, 1.0, 0.0);
            glVertex3f(triangles[i]->GetP1(0), triangles[i]->GetP1(1), triangles[i]->GetP1(2));
            glColor3f(0.0, 0.0, 1.0);
            glVertex3f(triangles[i]->GetP2(0), triangles[i]->GetP2(1), triangles[i]->GetP2(2));
        glEnd();

    }
}

CollisionSurface pba::MakeCollisionSurface() {
    return CollisionSurface( new CollisionSurfaceRaw() );
}