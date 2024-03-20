#include "SpringSim.h"
#include <vector>
#include "Particle.h"
#include "BaseSystem.h"
#include "GlobalResourceManager.h"

SpringSim::SpringSim(const std::string& name) :
	BaseSimulator(name),
	intgrMethod(0),
	grav(0.0),
	drag(0.0),
	timeStep(0.001),
	curTime(0.0),
	gks(0.0),
	gkd(0.0),
	numSprings(0)
{

}

int SpringSim::init(double time) {
	animTcl::OutputMessage("init() called");
	return 0;
}

int SpringSim::step(double time) {

	double f[3] = {0.0, 0.0, 0.0};
	double acc[3] = { 0.0, 0.0, 0.0 };
	int p = 0;
	bool isFixed = false;
	double curP[7] = {0, 0, 0, 0, 0, 0, 0};
	while(true){
		isFixed = false;
		curP[0] = p;
		curSys->getState(curP);
		if (curP[6] == -200)break;
		double pPos[3] = {curP[0], curP[1], curP[2]};
		double pVel[3] = {0, 0, 0};
		pVel[0] = curP[3]; pVel[1] = curP[4]; pVel[2] = curP[5];
		double pMass[1] = { curP[6] };
		isFixed = false;
		

		//check if object is fixed
		for (int i = 0; i < fixed.size(); i++) {
			if (fixed[i] == p) {
				isFixed = true;
				break;
			}
		}
		if (isFixed) {
			p++;
			continue;
		}
		else {
			
			sumForces(pPos, pVel, pMass, p, f);
			setVel(pPos, pVel, timeStep, f, pMass);
			setPos(pPos, pVel, timeStep, f, pMass);
			double particleState[7] = {(double)p,  pPos[0], pPos[1], pPos[2], pVel[0], pVel[1], pVel[2] };
			curSys->setState(particleState);
		}
		p += 1;

	}
	return 0;
}


void SpringSim::sumForces(double pos[3], double vel[3], double mass[1], int p, double totalForces[3]) {
	std::vector<float> f = { 0, 0, 0 };//total forces
	double fPos[3] = { 0,0,0 };
	double tPos[3] = { 0,0,0 };
	double fVel[3] = { 0,0,0 };
	double tVel[3] = { 0,0,0 };
	double other[7] = { 0,0,0 };


	double inter[3] = { 0,0,0 };
	double interLen = 0;
	bool isUnderground = false;

	//check if the particle is underground =(
	if (pos[1] <= 0 && abs(vel[1]) > 0.01) {
		inter[0] = pos[0]; inter[1] = 0.0; inter[2] = pos[2];
		double t_val = -pos[1] / vel[1];
		for (int d = 0; d < 3; d++) {
			//inter[d] = pos[d] + t_val * vel[d];
			interLen += pow(pos[d] - inter[d], 2);
		}
		isUnderground = true;
		
		isUnderground = true;
		
	}
	interLen = sqrt(interLen);

		//check if there is a spring force	
	for (int i = 0; i < springs.size(); i++){

		float sf[3] = { 0,0,0 };
		float df[3] = { 0,0,0 };
		if (springs[i][0] == p) {
			fPos[0] = pos[0]; fPos[1] = pos[1]; fPos[2] = pos[2];
			fVel[0] = vel[0]; fVel[1] = vel[1]; fVel[2] = vel[2];
			other[0] = springs[i][1];
			curSys->getState(other);
			tPos[0] = other[0]; tPos[1] = other[1]; tPos[2] = other[2];
			tVel[0] = other[3]; tVel[1] = other[4]; tVel[2] = other[5];

		}
		else if (springs[i][1] == p) {
			tPos[0] = pos[0]; tPos[1] = pos[1]; tPos[2] = pos[2];
			tVel[0] = vel[0]; tVel[1] = vel[1]; tVel[2] = vel[2];
			other[0] = springs[i][0];
			curSys->getState(other);
			fPos[0] = other[0]; fPos[1] = other[1]; fPos[2] = other[2];
			fVel[0] = other[3]; fVel[1] = other[4]; fVel[2] = other[5];

		}

		float PDiff[3] = { 0,0,0 };
		float VDiff[3] = { 0,0,0 };
		for (int d = 0; d < 3; d++) {
			PDiff[d] = abs(fPos[d] - tPos[d]);
			VDiff[d] = abs(fVel[d] - tVel[d]);

		}
		float PDiffLen = sqrt(pow(PDiff[0], 2) + pow(PDiff[1], 2) + pow(PDiff[2], 2));
		if (!(PDiffLen <= 0.00001)) {
			for (int d = 0; d < 3; d++) {
						PDiff[d] /= PDiffLen;
			}
		}
		
		for (int d = 0; d < 3; d++) {
			sf[d] = springs[i][2] * (springs[i][4] - PDiffLen) * (PDiff[d]);
			df[d] = -springs[i][3] * (VDiff[d] * (PDiff[d])) * (PDiff[d]);
		}
		if (springs[i][1] == p) {
			for (int d = 0; d < 3; d++) {
				sf[d] = -sf[d];
				df[d] = -df[d];
			}
		}
		
		//apply spring force
		for (int d = 0; d < 3; d++) {
			totalForces[d] += sf[d] + df[d];
		}

	}

	if (isUnderground) {
		for (int i = 0; i < 3; i++) {
			totalForces[i] += -gks * (pos[i] - inter[i]) - gkd * (vel[i] * (pos[i] - inter[i]))*((pos[i] - inter[i]) / pow(interLen, 2));
		}
	}

	//apply gravity force
	totalForces[1] += grav * mass[0];

	//apply drag force
	for (int d = 0; d < 3; d++) {
		totalForces[d] += -drag * vel[d];
	}



}
void SpringSim::display(GLenum mode) {
	//set up GL
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glEnable(GL_COLOR_MATERIAL);
	
	for (int i = 0; i < numSprings; i++) {
		glBegin(GL_LINES);
		double from[7] = { 0, 0, 0, 0, 0, 0, 0 }; from[0] = springs[i][0];
		double to[7] = { 0, 0, 0, 0, 0, 0, 0 }; to[0] = springs[i][1];
		curSys->getState(from);
		curSys->getState(to);
		glVertex3f(from[0], from[1], from[2]);
		glVertex3f(to[0], to[1], to[2]);
		glEnd();
	}
	glPopAttrib();

}


void SpringSim::link(std::string sName, int snum) {

	BaseSystem* newSys = GlobalResourceManager::use()->getSystem(sName);
	curSys = newSys;
}

void SpringSim::addSpring(int i1, int i2, double ks, double kd, double restLength) {
	std::vector<double> spring = { (double)i1, (double)i2, ks, kd, restLength };
	springs.push_back(spring);
	numSprings++;
}

void SpringSim::fixParticle(int index) {
	fixed.push_back(index);
}

void SpringSim::setIntgrMethod(std::string type, double ts) {
	if (strcmp(type.c_str(), "euler") == 0) {
		intgrMethod = 0;
	}
	else if (strcmp(type.c_str(), "symplectic") == 0) {
		intgrMethod = 1;
	}
	else if (strcmp(type.c_str(), "verlet") == 0) {
		intgrMethod = 2;
	}
	timeStep = ts;
}

void SpringSim::setPos(double pos[3], double vel[3], double tStep, double forces[3], double mass[1]) {
	for (int x = 0; x < 3; x++) {
		float nVel = 0.0;
		float nPos = 0.0;
		//forward euler
		if (intgrMethod == 0) {
			nPos = pos[x] + tStep * vel[x];
			pos[x] = nPos;

		}

		//symplectic euler
		else if (intgrMethod == 1) {
			nPos = pos[x] + tStep * vel[x];
			pos[x] = nPos;
		}
		else if (intgrMethod == 2) {
			nPos = pos[x] + tStep * vel[x];
			pos[x] = nPos;
			prevPosition[x] = nPos;
			intgrMethod++;
		}

		//verlet 
		else {
			double accel = (forces[x] / mass[0]);
			double pPos = pos[x] - vel[x] * tStep + .5 * accel * pow(tStep, 2);
			nPos = pos[x] + vel[x] * tStep + .5 * accel * pow(tStep, 2);
			pos[x] = 2 * pos[x] - pPos + accel * pow(tStep, 2);
			vel[x] = (nPos - pPos) / (2 * tStep);
		}
	}



}

void SpringSim::setVel(double pos[3], double vel[3], double tStep, double forces[3], double mass[1]) {
	for (int x = 0; x < 3; x++) {
		float nVel = 0.0;
		float nPos = 0.0;
		if (mass[0] < 0.001) {
			continue;
		}
		//forward euler
		if (intgrMethod == 0) {
			nVel = vel[x] + tStep * (forces[x] / mass[0]);
			vel[x] = nVel;
			
		}

		//symplectic euler
		else if (intgrMethod == 1) {
			nVel = vel[x] + tStep * (forces[x] / mass[0]);
			vel[x] = nVel;
		}

		else if (intgrMethod == 2) {
			nVel = vel[x] + tStep * (forces[x] / mass[0]);
			vel[x] = nVel;
		}

		
		else {
		}
	}
}


void SpringSim::setGround(double ks, double kd) {
	gks = ks;
	gkd = kd;
}

void SpringSim::setGravity(double g) {
	grav = g;
}

void SpringSim::setDrag(double k) {
	drag = k;
}

int SpringSim::command(int argc, myCONST_SPEC char** argv) {
	if (strcmp(argv[0], "link") == 0) {
		link(argv[1], std::stoi(argv[2]));
		return TCL_OK;
	}
	else if (strcmp(argv[0], "spring") == 0) {
		int f = std::stoi(argv[1]);
		int t = std::stoi(argv[2]);
		double ks = std::stod(argv[3]);
		double kd = std::stod(argv[4]);
		double rl = std::stod(argv[5]);
		addSpring(f, t, ks, kd, rl);
	}
	else if (strcmp(argv[0], "fix") == 0) {
		int i = std::stoi(argv[1]);
		fixParticle(i);
	}
	else if (strcmp(argv[0], "integration") == 0) {
		setIntgrMethod(argv[1], std::stod(argv[2]));
	}
	else if (strcmp(argv[0], "ground") == 0) {
		setGround(std::stod(argv[1]), std::stod(argv[2]));
	}
	else if (strcmp(argv[0], "gravity") == 0) {
		setGravity(std::stod(argv[1]));
	}
	else if (strcmp(argv[0], "drag") == 0) {
		setDrag(std::stod(argv[1]));
	}
	else if (strcmp(argv[0], "step") == 0) {
		step(100);
	}
	return TCL_RETURN;
}

