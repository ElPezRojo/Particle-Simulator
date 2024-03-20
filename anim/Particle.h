#ifndef PARTICLE_H
#define PARTICLE_H
#include <vector>
#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>
#include "shared/opengl.h"

class Particle : public BaseSystem
{

	public:
		int numParticles;
		std::string n;
		std::vector<std::vector<float>> pInfo;
		//Inherited methods
		Particle(const std::string& name);
		int getNP();
		void getState(double* p);
		void setState(double* p);
		void reset(double time);
		void display(GLenum mode = GL_RENDER);
		int command(int argc, myCONST_SPEC char** argv);

		//custom methods
		std::vector<float> getPos(int i);
		void setPos(int i, std::vector<float> npos);
		std::vector<float> getVel(int i);
		void setVel(int i, std::vector<float> nvel);
		float getMass(int i);
		void initVectors(int np);
		std::string getName();
		void setAllVelocities(float nx, float ny, float nz);
		void editParticle(int i, float m, float px, float py, float pz, float pvx, float pvy, float pvz);
	protected:

		//position of the particles
		std::vector<float> x;
		std::vector<float> y;
		std::vector<float> z;

		//velocity of the particles
		std::vector<float> vx;
		std::vector<float> vy;
		std::vector<float> vz;

		//mass of the particles
		std::vector<float> mass;



};
#endif
