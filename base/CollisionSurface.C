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

void pba::CollisionSurfaceRaw::handle_RBD(RigidBody &rbd, const size_t aH, const double &dtH, const size_t &pH)
{
    triangles[pH]->handle_RBD(rbd, aH, dtH);
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

bool CollisionSurfaceRaw::MultiTriangleHit_RBD(RigidBody &rbd, const size_t pos,
                                               const double &dt, Vector &XH, size_t& aH, 
                                               double &dtH, size_t &pH)
{
    bool hitFound = false;
    dtH = 2.0*dt;
    for (size_t i = 0; i < triangles.size(); i++)
    {
        double dtH_candidate = dtH;
        Vector XH_candidate;
        if (triangles[i]->hit_RBD(rbd, pos, dt, XH_candidate, dtH_candidate))
        {
            hitFound = true;
            if(fabs(dtH_candidate) < fabs(dtH)) {
                dtH = dtH_candidate;
                XH = XH_candidate;
                aH = pos;
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

void CollisionSurfaceRaw::MakeSurfFromModel(const char *FilePath, const double& m, Vector& t) {

    std::vector<Vector> vertices;
    std::vector<Vector> faces;
    std::string line;

    std::ifstream file(FilePath);
    if (!file) {
        std::cout << "Could not open file" << std::endl;
    }

    while (std::getline(file, line)) {

        std::stringstream ss(line);
        std::string type;
        
        ss >> type;

        if(type.compare("v") == 0) {

            Vector inPos = Vector(0, 0, 0);

            ss >> inPos[0] >> inPos[1] >> inPos[2];

            vertices.push_back(inPos);
            
        }
        else if(type.compare("f") == 0) {

            Vector inFace = Vector(0, 0, 0);

            ss >> inFace[0] >> inFace[1] >> inFace[2];

            inFace -= Vector(1,1,1);

            faces.push_back(inFace);

        }

    }

    for (Vector face : faces) {

        int f0 = face[0];
        int f1 = face[1];
        int f2 = face[2];
        
        Vector f0Coord;
        Vector f1Coord;
        Vector f2Coord;
        
        f0Coord = vertices[f0]*m;
        f1Coord = vertices[f1]*m;
        f2Coord = vertices[f2]*m;

        f0Coord = vertices[f0] + t;
        f1Coord = vertices[f1] + t;
        f2Coord = vertices[f2] + t;
        

        CollisionTriangle triangle = MakeCollisionTriangle(f0Coord, f1Coord, f2Coord);

        triangle->SetColor(Color(drand48(),drand48(),drand48(),0));

        triangles.push_back(triangle);

    }

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