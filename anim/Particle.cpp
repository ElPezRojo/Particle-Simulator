#include "Particle.h"

Particle::Particle(const std::string& name) :
	BaseSystem(name),
	n(name)
{

}

void Particle::setState(double* p) {
	pInfo[(int)p[0]][1] = p[1];
	pInfo[(int)p[0]][2] = p[2];
	pInfo[(int)p[0]][3] = p[3];
	pInfo[(int)p[0]][4] = p[4];
	pInfo[(int)p[0]][5] = p[5];
	pInfo[(int)p[0]][6] = p[6];

}

void Particle::getState(double* p) {
	int i = (int)p[0];
	if (i >= numParticles) {
		p[6] = -200;
		return;
	}
	p[6] = pInfo[i][0];
	p[0] = pInfo[i][1];
	p[1] = pInfo[i][2];
	p[2] = pInfo[i][3];
	p[3] = pInfo[i][4];
	p[4] = pInfo[i][5];
	p[5] = pInfo[i][6];

}

std::string Particle::getName() {
	return n;
}

void Particle::reset(double t) {

}

float Particle::getMass(int i) {
	return pInfo[i][0];
}

std::vector<float> Particle::getPos(int i) {
	std::vector<float> retVal = { pInfo[i][1], pInfo[i][2], pInfo[i][3]};
	return retVal;
}

std::vector<float> Particle::getVel(int i) {
	std::vector<float> retVal = { pInfo[i][4], pInfo[i][5], pInfo[i][6] };
	return retVal;
}

int Particle::getNP() {
	return numParticles;
}

void Particle::display(GLenum mode) {
	//set up GL
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glEnable(GL_COLOR_MATERIAL);
	
	

	//draw the ground
	glColor3f(0.5, 0, 0);
	glBegin(GL_QUADS);
	glVertex3f(100, 0, 100);
	glVertex3f(-100, 0, 100);
	glVertex3f(-100, 0, -100);
	glVertex3f(100, 0, -100);
	glEnd();
	
	//draw points
	glPointSize(15);
	glColor3f(0.5, 0, 0.5);
	glBegin(GL_POINTS);

	for (int i = 0; i < numParticles; i++) {
		glVertex3f(pInfo[i][1], pInfo[i][2], pInfo[i][3]);
	}
	glEnd();
	glPopAttrib();

}

void Particle::initVectors(int np) {
	numParticles = np;
	std::vector<std::vector<float>> vec(np, std::vector<float>(7));
	pInfo = vec;
}

void Particle::setAllVelocities(float nx, float ny, float nz) {

}

void Particle::editParticle(int i, float m, float px, float py, float pz, float pvx, float pvy, float pvz) {
	pInfo[i][0] = m;
	pInfo[i][1] = px;
	pInfo[i][2] = py;
	pInfo[i][3] = pz;
	pInfo[i][4] = pvx;
	pInfo[i][5] = pvy;
	pInfo[i][6] = pvz;
}

int Particle::command(int argc, myCONST_SPEC char** argv) {


	if (argc == 0) {
		return TCL_OK;
	}
	if (strcmp(argv[0], "dim") == 0) {
		if (argc >= 1) {
			initVectors(std::stoi(argv[1]));
			return TCL_OK;

		}
	}
	else if (strcmp(argv[0], "particle") == 0) {
		int index = std::stoi(argv[1]);
		float mass = std::stof(argv[2]);
		float px = std::stof(argv[3]);
		float py = std::stof(argv[4]);
		float pz = std::stof(argv[5]);
		float vx = std::stof(argv[6]);
		float vy = std::stof(argv[7]);
		float vz = std::stof(argv[8]);
		editParticle(index, mass, px, py, pz, vx, vy, vz);
		return TCL_OK;
	}
	else if (strcmp(argv[0], "all_velocities") == 0) {
		if (argc >= 2) {
			float newX = std::stof(argv[1]);
			float newY = std::stof(argv[2]);
			float newZ = std::stof(argv[3]);
			setAllVelocities(newX, newY, newZ);
			return TCL_OK;
		}
	}
	



}


