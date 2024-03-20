#ifndef SPRINGSIM_H
#define SPRINGSIM_H
#include "BaseSimulator.h"
#include "Particle.h";
#include <vector>
class SpringSim : public BaseSimulator
{
public:

	double grav;
	double drag;
	double gks, gkd;
	BaseSystem* curSys;
	
	//Inherited methods
	SpringSim(const std::string& name);
	int step(double time);
	int init(double time);
	int command(int argc, myCONST_SPEC char** argv);
	void display(GLenum mode = GL_RENDER);

	//custom methods

	void link(std::string sName, int snum);
	void addSpring(int i1, int i2, double ks, double kd, double restLength);
	void fixParticle(int index);
	void setIntgrMethod(std::string type, double ts);
	void setGround(double ks, double kd);
	void setGravity(double g);
	void setDrag(double k);
	void setPos(double pos[3], double vel[3], double tStep, double forces[3], double mass[1]);
	void setVel(double pos[3], double vel[3], double tStep, double forces[3], double mass[1]);
	void sumForces(double pos[3], double vel[3], double mass[1], int p, double totalForces[3]);
	void pushFromGround();
protected:
	int numSprings;
	std::vector<std::vector<double>> springs;
	std::vector<int> fixed;

	int intgrMethod;
	double timeStep;
	double curTime;
	double prevPosition[3];




};
#endif
