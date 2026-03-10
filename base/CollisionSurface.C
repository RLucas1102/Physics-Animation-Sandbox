#include "CollisionSurface.h"
#include <GL/gl.h>
#include <GL/glut.h>

using namespace pba;

CollisionSurfaceRaw::CollisionSurfaceRaw() :
    coeffR (1.0),
    coeffS (1.0)
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
    if (coeffR >= 1) {
        coeffR = 1;
    }
    else {
        coeffR *= 1.1;
    }
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
void CollisionSurfaceRaw::MakePlane(const double &m) {
    Vector FrontBR = Vector( 1.0 * m, -1.0 * m, 1.0 * m);
    Vector FrontBL = Vector(-1.0 * m, -1.0 * m, 1.0 * m);
    Vector BackBR = Vector( 1.0 * m, -1.0 * m, -1.0 * m);
    Vector BackBL = Vector(-1.0 * m, -1.0 * m, -1.0 * m);

    CollisionTriangle Bot1   = MakeCollisionTriangle(FrontBR, FrontBL, BackBR);
    CollisionTriangle Bot2   = MakeCollisionTriangle(FrontBL, BackBL, BackBR);

    Bot1->SetColor(Color(  1.0, 0.5, 0.0, 1.0));
    Bot2->SetColor(Color(  0.5, 1.0, 0.5, 1.0));
    
    triangles.push_back(Bot1);
    triangles.push_back(Bot2);

}

void CollisionSurfaceRaw::MakeBox(const double& m) {

    Vector FrontBR = Vector( 1.0 * m, -1.0 * m, 1.0 * m);
    Vector FrontBL = Vector(-1.0 * m, -1.0 * m, 1.0 * m);
    Vector FrontTR = Vector( 1.0 * m,  1.0 * m, 1.0 * m);
    Vector FrontTL = Vector(-1.0 * m,  1.0 * m, 1.0 * m);

    Vector BackBR = Vector( 1.0 * m, -1.0 * m, -1.0 * m);
    Vector BackBL = Vector(-1.0 * m, -1.0 * m, -1.0 * m);
    Vector BackTR = Vector( 1.0 * m,  1.0 * m, -1.0 * m);
    Vector BackTL = Vector(-1.0 * m,  1.0 * m, -1.0 * m);

    _llc = BackBL;
    _urc = FrontTR;

    CollisionTriangle Front1 = MakeCollisionTriangle(FrontTR, FrontBL, FrontBR);
    CollisionTriangle Front2 = MakeCollisionTriangle(FrontTR, FrontTL, FrontBL);

    CollisionTriangle Back1  = MakeCollisionTriangle(BackTR, BackBR, BackBL);
    CollisionTriangle Back2  = MakeCollisionTriangle(BackTR, BackBL, BackTL);

    CollisionTriangle Left1  = MakeCollisionTriangle(FrontTL, BackBL, FrontBL);
    CollisionTriangle Left2  = MakeCollisionTriangle(FrontTL, BackTL, BackBL);

    CollisionTriangle Right1 = MakeCollisionTriangle(FrontTR, FrontBR, BackBR);
    CollisionTriangle Right2 = MakeCollisionTriangle(FrontTR, BackBR, BackTR);

    CollisionTriangle Top1   = MakeCollisionTriangle(FrontTR, BackTR, FrontTL);
    CollisionTriangle Top2   = MakeCollisionTriangle(BackTR, BackTL, FrontTL);

    CollisionTriangle Bot1   = MakeCollisionTriangle(FrontBR, FrontBL, BackBR);
    CollisionTriangle Bot2   = MakeCollisionTriangle(FrontBL, BackBL, BackBR);
   
    Front1->SetColor(Color(1.0, 0.0, 0.0, 1.0));
    Front2->SetColor(Color(0.0, 1.0, 0.0, 1.0));
    Back1->SetColor(Color( 0.0, 0.0, 1.0, 1.0));
    Back2->SetColor(Color( 1.0, 1.0, 0.0, 1.0));
    Left1->SetColor(Color( 1.0, 0.0, 1.0, 1.0));
    Left2->SetColor(Color( 0.0, 1.0, 1.0, 1.0));
    Right1->SetColor(Color(1.0, 1.0, 0.5, 1.0));
    Right2->SetColor(Color(1.0, 0.5, 1.0, 1.0));
    Top1->SetColor(Color(  0.5, 1.0, 1.0, 1.0));
    Top2->SetColor(Color(  0.0, 0.5, 1.0, 1.0));
    Bot1->SetColor(Color(  1.0, 0.5, 0.0, 1.0));
    Bot2->SetColor(Color(  0.5, 1.0, 0.5, 1.0));

    triangles.push_back(Front1);
    triangles.push_back(Front2);
    triangles.push_back(Back1);
    triangles.push_back(Back2);
    triangles.push_back(Left1);
    triangles.push_back(Left2);
    triangles.push_back(Right1);
    triangles.push_back(Right2);
    triangles.push_back(Top1);
    triangles.push_back(Top2);
    triangles.push_back(Bot1);
    triangles.push_back(Bot2);

}

Vector const CollisionSurfaceRaw::GetURC() {
    return _urc;
}

Vector const CollisionSurfaceRaw::GetLLC() {
    return _llc;
}

void CollisionSurfaceRaw::Display() {

    for (size_t i = 0; i < triangles.size(); i++) {
        glBegin(GL_TRIANGLES);
            glColor3f(triangles[i]->GetColor(0), triangles[i]->GetColor(1), triangles[i]->GetColor(2));
            glVertex3f(triangles[i]->GetP0(0), triangles[i]->GetP0(1), triangles[i]->GetP0(2));
            glColor3f(triangles[i]->GetColor(0), triangles[i]->GetColor(1), triangles[i]->GetColor(2));
            glVertex3f(triangles[i]->GetP1(0), triangles[i]->GetP1(1), triangles[i]->GetP1(2));
            glColor3f(triangles[i]->GetColor(0), triangles[i]->GetColor(1), triangles[i]->GetColor(2));
            glVertex3f(triangles[i]->GetP2(0), triangles[i]->GetP2(1), triangles[i]->GetP2(2));
        glEnd();

    }
}

CollisionSurface pba::MakeCollisionSurface() {
    return CollisionSurface( new CollisionSurfaceRaw() );
}