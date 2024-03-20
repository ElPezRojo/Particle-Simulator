#include "HSpline.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
#include <sstream>
#include <cmath>

	
HSpline::HSpline(const std::string& name) :
	BaseSystem(name),
	len(0),
	isInitialized(false),
	p1(false),
	p2(false),
	offset(0.0)
	{
		eval_at_value(0.0L, coords);
	}

	void HSpline::catmullRom() {
		for (int d = 0; d < 3; d++) {
			CP[0][d + 3] = 2 * (CP[1][d] - CP[0][d]) - (CP[2][d] - CP[0][d]) / 2;
			CP[len - 1][d + 3] = 2 * (CP[len - 2][d] - CP[len - 1][d]) - (CP[len - 3][d] - CP[len - 2][d]) / 2;
		}
		for (int i = 1; i < len - 1; i++) {
			for (int d = 0; d < 3; d++) {
				CP[i][d + 3] = (CP[i + 1][d] - CP[i - 1][d]) / 2;
			}
		}

		animTcl::OutputMessage("done");




	}
	void HSpline::part1() {
		p1 = true;
	}

	void HSpline::part2() {
		p2 = true;


	}
	void HSpline::display(GLenum mode) {
		if (!isInitialized) {
			return;
		}
		glEnable(GL_LIGHTING);
		glMatrixMode(GL_MODELVIEW);
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glEnable(GL_COLOR_MATERIAL);

		//draw control points
		glPointSize(5);
		glBegin(GL_POINTS);
		glColor3f(1.0, 0.0, 0.0);
		for (int i = 0; i < len; i++) {
			glVertex3f(CP[i][0], CP[i][1], CP[i][2]);
		}
		glEnd();

		// draw interpolated line
		glLineWidth(5);
		glBegin(GL_LINE_STRIP);
		glColor3f(0.0, 0.0, 0.0);
		float cur_point[3];
		for (int i = 0; i < len - 1; i++) {
			for (int k = 0; k < 100; k++) {
				double t = (double)i + ((double)k / 100.0);
				eval_at_value(t, cur_point);
				glVertex3f(cur_point[0], cur_point[1], cur_point[2]);
			}
		}

		glPopAttrib();
		glEnd();

		if (!p2) {
			return;
		}
			// draw porsche
			m_model.ReadOBJ("data/porsche.obj");
			glmFacetNormals(&m_model);
			glmVertexNormals(&m_model, 90);

			glEnable(GL_LIGHTING);
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glTranslated(coords[0], coords[1], coords[2]);
			float rot = prevRoty + roty;
			glRotatef(rot, 0, 1, 0);
			animTcl::OutputMessage("rot: %04f", rot);

			glScaled(0.01, 0.01, 0.01);

			if (m_model.numvertices > 0)
				glmDraw(&m_model, GLM_SMOOTH | GLM_MATERIAL);
			else
				glutSolidSphere(1.0, 20, 20);
			glPopAttrib();
			glPopMatrix();

	





		return;
	}
	void HSpline::getName(double* p) {

	}


	void HSpline::getState(double* p) {

	}

	void HSpline::setState(double* p) {
		double tVal = *p;
		animTcl::OutputMessage("%02f", tVal);
		if (tVal > len-1) {
			return;
		}
		lastT = tVal;
		lastCoords[0] = coords[0];
		lastCoords[1] = coords[1];
		lastCoords[2] = coords[2];
		eval_at_value(tVal - offset, coords);
		float curTangent[3] = { 0.0, 0.0, 0.0 };
		getTangent(tVal, curTangent);
		prevRoty += roty;
		roty = calculate_rotation(lastDir, curTangent); 

	}

	void HSpline::reset(double time) {
		coords[0] = CP[0][0];
		coords[1] = CP[0][1];
		coords[2] = CP[0][2];
		float curDir[3] = { 1.0, 1.0, 1.0 };
		float firstTangent[3] = { 0.0, 0.0, 0.0 };
		getTangent(0, firstTangent);
		prevRoty = calculate_rotation(curDir, firstTangent);
		roty = calculate_rotation(curDir, firstTangent);
		offset = lastT;
		lastDir[0] = 0.0;
		lastDir[1] = 0.0;
		lastDir[2] = 0.0;

	}

	int HSpline::command(int argc, myCONST_SPEC char** argv) {
		if (strcmp(argv[0], "cr") == 0) {
			catmullRom();
			return TCL_OK;
		}
		else if (strcmp(argv[0], "set") == 0) {
			if (strcmp(argv[1], "point") == 0) {

				setPoint(std::stof(argv[2]), std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5]));
				return TCL_OK;
			}else if (strcmp(argv[1], "tangent") == 0) {

				setTangent(std::stof(argv[2]), std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5]));
				return TCL_OK;
			}
			animTcl::OutputMessage("parameters missing / inaccurate");
			return TCL_OK;

		}
		else if (strcmp(argv[0], "add") == 0 && strcmp(argv[1], "point") == 0) {
			setCP(std::stof(argv[2]), std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5]), std::stof(argv[6]), std::stof(argv[7]), std::stof(argv[8]));
			return TCL_OK;
		}
		else if (strcmp(argv[0], "getArcLength") == 0) {
			double t = std::stod(argv[1]);
			double arcLen = calcArcLen(t);
			animTcl::OutputMessage("arcLen: %l", arcLen);
			return TCL_OK;

		}
		else if (strcmp(argv[0], "load") == 0) {
			readFromFile(argv[1]);
			return TCL_OK;
		}else if (strcmp(argv[0], "export") == 0) {
			writeToFile(argv[1]);
			return TCL_OK;
		}
		else {
			animTcl::OutputMessage("could not recognize command");
			return TCL_OK;
		}
		


	}
	void HSpline::setPoint(float x, float y, float z, int index){

		CP[index][0] = x;
		CP[index][1] = y;
		CP[index][2] = z;
	}

	void HSpline::setTangent(float x, float y, float z, int index){
		CP[index][3] = x;
		CP[index][4] = y;
		CP[index][5] = z;


	}

	void HSpline::setCP(float px, float py, float pz, float tx, float ty, float tz, int index) {
		CP[index][0] = px;
		CP[index][1] = py;
		CP[index][2] = pz;
		CP[index][3] = tx;
		CP[index][4] = ty;
		CP[index][5] = tz;
	}

	void HSpline::eval_at_value(double t, float location[3]) {
		int ti = floor(t);
	
		if (t > len - 1) {
			location[0] = CP[len - 1][0];
			location[1] = CP[len - 1][1];
			location[2] = CP[len - 1][2];
			return;
		}else if(t == 0) {
			location[0] = CP[0][0];
			location[1] = CP[0][1];
			location[2] = CP[0][2];
			return;
		}
		/*
		for(int i = 0; i > len - 1; i++) {
			if (t <= arcTable[i][1]) {
				ti = i;
				break;
			}
		}
		t -= arcTable[ti - 1][1];
		t /= arcTable[ti][0];
		*/
		float noPow = t - ti;
		float pow2 = pow(noPow, 2);
		float pow3 = pow(noPow, 3);
		for (int i = 0; i < 3; i++) {
			float y_i = CP[ti][i] * (2.0f * pow3 - 3.0f * pow2 + 1.0f);
			float y_next = CP[ti + 1][i] * (-2 * pow3 + 3 * pow2);
			float s_i = CP[ti][i + 3] * (pow3 - 2 * pow2 + noPow);
			float s_next = CP[ti + 1][i + 3] * (pow3 - pow2);
			location[i] = y_i + y_next + s_i + s_next;
			//if (t == 0.0) {
				//animTcl::OutputMessage("i = %i, result = %02f", i, location[i]);
			//}
		}
		return;
	}

	double HSpline::calcArcLen(double t) {
		double cur_t = 0.0;
		t *= 10.0L;
		double totalDist = 0.0;
		float last[3] = { 0.0, 0.0, 0.0 };
		float cur[3] = { 0.0, 0.0, 0.0 };
		eval_at_value(0.0, last);
		while (cur_t < t) {
			cur_t += 0.01;
			eval_at_value(cur_t, cur);
			float a = pow((last[0] - cur[0]), 2);
			float b = pow((last[1] - cur[1]), 2);
			float c = pow((last[2] - cur[2]), 2);
			totalDist += sqrt(a+b+c);
			last[0] = cur[0];
			last[1] = cur[1];
			last[2] = cur[2];

		}
		return totalDist;

	}

	void HSpline::writeToFile(std::string fn) {
		if (!isInitialized) {
			return;
		}
		std::ofstream file(fn);
		std::string in = name + std::string(" ") + std::to_string(len);
		file << in;
		file << "\n";
		for (int i = 0; i < len; i++) {
			file << (std::to_string(CP[i][0]) + std::to_string(CP[i][1]) + std::to_string(CP[i][2]) + std::to_string(CP[i][3]) + std::to_string(CP[i][4]) + std::to_string(CP[i][5]));
			file << "\n";
		}
		return;
	}


	//calculates angles between 2 vectors
	float HSpline::calculate_rotation(float cur[3], float target[3]) {
		float angle = 0.0;
		float a = cur[0] * target[1] - target[0] * cur[1];
		float b = cur[0] * cur[1] + target[0] * target[1];
		angle = atan2f(a, b);
		/*
		float dotProduct = cur[0] * target[0] + cur[1] * target[1];
		float magCur = (pow(cur[0], 2) + pow(cur[1], 2));
		float magTarg = (pow(target[0], 2) + pow(target[1], 2));
		angle = acosf(dotProduct / (magCur * magTarg));
		*/
		return angle;


	}
	
	void HSpline::getTangent(double t, float tangent[3]) {
		float cur[3] = { 0.0, 0.0, 0.0 };
		float next[3] = { 0.0, 0.0, 0.0 };

		eval_at_value(t, cur);
		eval_at_value(t + 0.1, cur);

		tangent[0] = next[0] - cur[0];
		tangent[1] = next[1] - cur[1];
		tangent[2] = next[2] - cur[2];


	}

	void HSpline::readFromFile(std::string fn) {
		std::ifstream file(fn);
		std::string curLine;

		if (!file) {
			printf("rip");
			animTcl::OutputMessage("file could not be found");
			return;
		}
		getline(file, curLine);
		std::stringstream stream(curLine);
		stream >> name;
		std::string nString;
		stream >> nString;
		len = std::stoi(nString);
		for (int i = 0; i < len; i++) {
			getline(file, curLine);
			std::stringstream stream(curLine);
			std::string temp;
			for (int k = 0; k < 6; k++) {
				stream >> temp;
				CP[i][k] = std::stod(temp);
			}
		}
		coords[0] = CP[0][0];
		coords[1] = CP[0][1];
		coords[2] = CP[0][2];
		initArcTable();
		float curDir[3] = { 0.0, 1.0, 0.0 };
		getTangent(0, lastDir);
		prevRoty = 0.0;
		roty = calculate_rotation(curDir, lastDir);
		isInitialized = true;
		return;
	}

	void HSpline::initArcTable() {
		double prevVal = 0.0;
		double totalVal = 0.0;
		for (int i = 1; i < len - 1; i++) {
			double curVal = calcArcLen(i / 10.0);
			totalVal += curVal;
			arcTable[i][0] = curVal - prevVal;
			arcTable[i][1] = totalVal;
			prevVal = curVal;
			totalArcLen += curVal;
		}
	}




