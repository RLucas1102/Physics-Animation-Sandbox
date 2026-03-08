#include "Force.h"

using namespace pba;

void AccumulatingForce::compute( PSYS& psys, const double dt) {
    
    Vector resetForce = Vector(0,0,0);
    for (size_t i = 0; i < psys->Psize(); i++) {
        psys->SetAcc(i, resetForce);
    }

    for (size_t i = 0; i < forces.size(); i++) {
        forces[i]->compute(psys, dt);
    }
    
}

void AccumulatingForce::AddForce( Force& f) {
    forces.push_back(f);
}

Force AccumulatingForce::GetForce( const size_t i) {
    return forces[i];
}

GravityForce::GravityForce(const Vector& g) :
    gravity (g)
    {}

void GravityForce::compute( PSYS& psys, const double dt) {

    for (size_t i = 0; i < psys->Psize(); i++) {

        Vector A = psys->GetAcc(i);

        A += gravity;

        psys->SetAcc(i, A);
    }
    
}

void GravityForce::IncreaseGravityForce() {
    gravity *= 1.1;
}

void GravityForce::DecreaseGravityForce() {
    gravity /= 1.1;
}

double GravityForce::GetGravityMag() {
    return gravity[1];
}

ViscosityForce::ViscosityForce(const double Pbar, const double rhoBar, const double gamma, 
                               const double alpha, const double beta, const double eps, const OV& o) :
                               _Pbar (Pbar), _rhoBar(rhoBar), _gamma(gamma),
                               _alpha(alpha), _beta(beta), _eps(eps), O(o)
                               {}

void ViscosityForce::compute(PSYS &psys, const double dt) {

    std::shared_ptr<SPHSystem> sph = std::dynamic_pointer_cast<SPHSystem>(psys);
    
    #pragma omp parallel for
    for(size_t i = 0; i < sph->Psize(); i++) {
        Vector A = sph->GetAcc(i);
        Vector viscosity = Vector(0,0,0);
            
        double Ca = CalcSpeedOfSound(_Pbar, _rhoBar, _gamma, sph->GetRho(i));
        
        size_t cell = O->FindPosInVolume(sph->GetPos(i));
        std::vector<size_t> neighborhood = O->GetNeighborhood(cell);
 
        for (size_t cell : neighborhood) {
            std::vector<size_t> cellContents = O->GetCellContents(cell);
            
            for (size_t particle : cellContents) {
                double Cb = CalcSpeedOfSound(_Pbar, _rhoBar, _gamma, sph->GetRho(particle));
                double Cab = Ca + Cb;

                Vector Pa = sph->GetPos(i);
                Vector Pb = sph->GetPos(particle);
                Vector Va = sph->GetVel(i);
                Vector Vb = sph->GetVel(particle);

                double muab = CalcMuab(sph->GetH(), Pa, Pb, Va, Vb, _eps);
                double Piab = CalcPiab(_alpha, _beta, Cab, muab, sph->GetRho(i), sph->GetRho(particle));

                Vector AB = sph->GetPos(i) - sph->GetPos(particle);
                viscosity += sph->GetMass(particle) * Piab * CalcGradWeightKernel(AB, sph->GetH());
                    
            }
        }

        A -= viscosity;
        sph->SetAcc(i, A);
        
    }
}

void ViscosityForce::ChangeVStrength(const double alpha) {
    _alpha += alpha;
}

PressureForce::PressureForce(const double Pbar, const double rhoBar, const double gamma, const OV& o) :
    _Pbar(Pbar), 
    _rhoBar(rhoBar), 
    _gamma(gamma),
    O (o)
    {}

void PressureForce::compute(PSYS &psys, const double dt) {

    std::shared_ptr<SPHSystem> sph = std::dynamic_pointer_cast<SPHSystem>(psys);
    
    #pragma omp parallel for
    for(size_t i = 0; i < sph->Psize(); i++) {
        Vector A = sph->GetAcc(i);
        Vector pressure = Vector(0,0,0);
        
        size_t cell = O->FindPosInVolume(sph->GetPos(i));
        std::vector<size_t> neighborhood = O->GetNeighborhood(cell);
 
        for (size_t cell : neighborhood) {
            std::vector<size_t> cellContents = O->GetCellContents(cell);
            
            for (size_t particle : cellContents) {
                double term1 = CalcTaitEquation(_rhoBar, _Pbar, _gamma, sph->GetRho(i))/std::pow(sph->GetRho(i), 2);
                double term2 = CalcTaitEquation(_rhoBar, _Pbar, _gamma, sph->GetRho(particle))/std::pow(sph->GetRho(particle), 2);

                Vector AB = sph->GetPos(i) - sph->GetPos(particle);
                pressure += sph->GetMass(particle) * (term1 + term2) * CalcGradWeightKernel(AB, sph->GetH());
            }
        }

        A -= pressure;
        sph->SetAcc(i, A);
        
    }

}

void PressureForce::ChangePStrength(const double Pbar) {
    _Pbar += Pbar;
}

void PressureForce::ChangeBaseDensity(const double rhoBar) {
    _rhoBar += rhoBar;
}

void PressureForce::ChangePower(const double gamma) {
    _gamma += gamma;
}

AccumulatingStrutForce::AccumulatingStrutForce(const double g, const double f) :
    _spring (g),
    _friction(f)
    {}

void AccumulatingStrutForce::compute(PSYS& psys, const double dt) {

    std::shared_ptr<SoftBodySystem> s = std::dynamic_pointer_cast<SoftBodySystem>(psys);

    for(size_t i = 0; i < s->Pairs(); i++) {
        const SoftEdge& se = s->GetConnectedPair(i);
        const size_t& inode = se->GetFirstNode();
        const size_t& jnode = se->GetSecondNode();

        Vector iPos = s->GetPos(inode);
        Vector jPos = s->GetPos(jnode);

        Vector iVel = s->GetVel(inode);
        Vector jVel = s->GetVel(jnode);
        
        Vector dx = iPos - jPos;
        Vector ff = iVel - jVel;

        Vector F;
        double separation = dx.magnitude() - se->GetEdgeLength();
        dx.normalize();
        F = dx * (separation * _spring);
        F += dx * (dx*ff) * _friction;
        
        Vector iAcc = s->GetAcc(inode);
        Vector jAcc = s->GetAcc(jnode);

        jAcc = jAcc + F/s->GetMass(jnode);
        iAcc = iAcc - F/s->GetMass(inode);
        s->SetAcc(jnode, jAcc);
        s->SetAcc(inode, iAcc);

    }

}

Force pba::CreateGravityForce(const Vector& g) {
    return Force( new GravityForce(g) );
}

Force pba::CreateAccumulatingForce() {
    return Force( new AccumulatingForce() );
}

Force pba::CreateViscosityForce(const double Pbar, const double rhoBar, const double gamma, const double alpha, const double beta, const double eps, const OV& o) {
    return Force( new ViscosityForce(Pbar, rhoBar, gamma, alpha, beta, eps, o) );
}

Force pba::CreatePressureForce(const double Pbar, const double rhoBar, const double gamma, const OV& o) {
    return Force( new PressureForce(Pbar, rhoBar, gamma, o) );
}

Force pba::CreateAccumulatingStrutForce(const double g, const double f) {
    return Force( new AccumulatingStrutForce(g, f) );
}

double pba::CalcSpeedOfSound(const double Pbar, const double rhoBar, const double gamma, const double density) {
    double term1 = Pbar / rhoBar;
    double term2 = std::pow(density / rhoBar, (gamma - 1) );
    double C = std::pow(gamma * term1 * term2, 0.5);
    
    return C;
 }

 double pba::CalcMuab(const double h, Vector &P1, Vector &P2, Vector& V1, Vector&V2, const double eps) {
    double term1 = h * (V1 - V2) * (P1 - P2);
    double term2 = std::pow((P1 - P2).magnitude(), 2);
    double term3 = term2 + (eps * h * h);
    double Muab  = term1 / term3;
    
    return Muab;
 }

 double pba::CalcPiab(const double alpha, const double beta, const double C, const double muab, const double densitya, const double densityb) {
    double term1 = -alpha * C * muab;
    double term2 = beta * muab * muab;
    double term3 = term1 + term2;
    double term4 = densitya + densityb;
    double term5 = term3 / term4;

    return term5;
}

double pba::CalcWeightKernel(Vector &P, const double h) {
    double q = P.magnitude() / h;
    double sigma = 1/(M_PI * (h * h * h));
    double result;
    
    if (q >= 0 && q < 1) {
        double term1 = ((3/2) * q * q);
        double term2 = 1 - (q/2);
        double term3 = 1 - (term1 * term2);
        result = sigma * term3;
    }
    else if (q > 1 && q <= 2) {
        double term1 = 2 - q;
        double term2 = sigma / 4;
        result = term2 * (term1 * term1 * term1);
    }
    else {
        result = 0;
    }
    
    return result;
}

Vector pba::CalcGradWeightKernel(Vector &P, const double h) {
    double q = P.magnitude() / h;
    double sigma = 1/(M_PI * (h * h * h));
    Vector term1 = (1/h) * (P / P.magnitude());
    double result;
    
    if (P.magnitude() == 0) {
        return Vector(0,0,0);
    }

    if (q >= 0 && q < 1) {
        double term2 = 1 - ((3/4) * q);
        result = -3 * sigma * q * term2;
    }
    else if (q > 1 && q <= 2) {
        double term2 = (2 - q) * (2 - q);
        result = -(3/4) * sigma * term2;
    }
    else {
        result = 0;
    }
    
    return term1 * result;
}

double pba::CalcTaitEquation(const double rhoBar, const double Pbar, const double gamma, const double density) {
    double term1 = density / rhoBar;
    double term2 = std::pow(term1, gamma) - 1;
    double result = Pbar * term2;

    return result;
}