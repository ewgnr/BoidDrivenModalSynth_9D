#pragma once

#include "ofMain.h"
#include "ofxOsc.h"

#define DIMS_PER_BOID 9

struct Boid
{
	int index = 0;
	float position[DIMS_PER_BOID] = { 0 };
	float velocity[DIMS_PER_BOID] = { 0 };
};

class SwarmOSCReceiver
{
public:
	SwarmOSCReceiver() = default;

	void setup(int port)
	{
		receiver.setup(port);
		ofLogNotice() << "SwarmOSCReceiver listening on port " << port;
	}

	void update()
	{
		while (receiver.hasWaitingMessages())
		{
			ofxOscMessage msg;
			receiver.getNextMessage(msg);
			handleMessage(msg);
		}
	}

	const std::vector<Boid> & getBoids() const { return boids; }
	int getSwarmIndex() const { return swarmIndex; }

	void drawDebug(float startX = 20.0f, float startY = 20.0f)
	{
		ofBackground(0);

		int y = startY;
		ofDrawBitmapStringHighlight(
			"Swarm " + ofToString(swarmIndex) + " | Boids: " + ofToString(boids.size()),
			startX, y);
		y += 25;

		for (const auto & b : boids)
		{
			ofSetColor(255);

			std::string pos = "Boid " + ofToString(b.index) + "  POS  "
															  "D1("
				+ ofToString(b.position[0], 2) + ","
				+ ofToString(b.position[1], 2) + ","
				+ ofToString(b.position[2], 2) + ") "
												 "D2("
				+ ofToString(b.position[3], 2) + ","
				+ ofToString(b.position[4], 2) + ","
				+ ofToString(b.position[5], 2) + ") "
												 "D3("
				+ ofToString(b.position[6], 2) + ","
				+ ofToString(b.position[7], 2) + ","
				+ ofToString(b.position[8], 2) + ")";

			ofDrawBitmapString(pos, startX, y);
			y += 15;

			ofSetColor(180, 180, 255);

			std::string vel = "        VEL  "
							  "D1("
				+ ofToString(b.velocity[0], 2) + ","
				+ ofToString(b.velocity[1], 2) + ","
				+ ofToString(b.velocity[2], 2) + ") "
												 "D2("
				+ ofToString(b.velocity[3], 2) + ","
				+ ofToString(b.velocity[4], 2) + ","
				+ ofToString(b.velocity[5], 2) + ") "
												 "D3("
				+ ofToString(b.velocity[6], 2) + ","
				+ ofToString(b.velocity[7], 2) + ","
				+ ofToString(b.velocity[8], 2) + ")";

			ofDrawBitmapString(vel, startX, y);
			y += 22;
		}

		ofSetColor(255);
	}

	void printMinMax()
	{
		if (boids.empty()) return;

		float posMin[3][3], posMax[3][3];
		float velMin[3][3], velMax[3][3];

		for (int d = 0; d < 3; d++)
		{ 
			for (int axis = 0; axis < 3; axis++)
			{
				posMin[d][axis] = posMax[d][axis] = boids[0].position[d * 3 + axis];
				velMin[d][axis] = velMax[d][axis] = boids[0].velocity[d * 3 + axis];
			}
		}

		for (const auto &b : boids)
		{
			for (int d = 0; d < 3; d++) {
				for (int axis = 0; axis < 3; axis++) {
					float p = b.position[d * 3 + axis];
					if (p < posMin[d][axis]) posMin[d][axis] = p;
					if (p > posMax[d][axis]) posMax[d][axis] = p;

					float v = b.velocity[d * 3 + axis];
					if (v < velMin[d][axis]) velMin[d][axis] = v;
					if (v > velMax[d][axis]) velMax[d][axis] = v;
				}
			}
		}

		std::cout << "Position Min/Max per Axis:\n";
		for (int d = 0; d < 3; d++) {
			std::cout << "D" << (d+1) 
					  << " x:[" << posMin[d][0] << "," << posMax[d][0] << "] "
					  << "y:[" << posMin[d][1] << "," << posMax[d][1] << "] "
					  << "z:[" << posMin[d][2] << "," << posMax[d][2] << "]\n";
		}

		std::cout << "Velocity Min/Max per Axis:\n";
		for (int d = 0; d < 3; d++) {
			std::cout << "D" << (d+1) 
					  << " x:[" << velMin[d][0] << "," << velMax[d][0] << "] "
					  << "y:[" << velMin[d][1] << "," << velMax[d][1] << "] "
					  << "z:[" << velMin[d][2] << "," << velMax[d][2] << "]\n";
		}
	}

private:
	ofxOscReceiver receiver;
	int swarmIndex = -1;
	std::vector<Boid> boids;

	void handleMessage(const ofxOscMessage & msg)
	{
		auto addr = ofSplitString(msg.getAddress(), "/");
		if (addr.size() != 5 || addr[1] != "swarm") return;

		swarmIndex = ofToInt(addr[2]);
		std::string type = addr[4]; // "position" or "velocity"

		int totalFloats = msg.getNumArgs();
		if (totalFloats % DIMS_PER_BOID != 0)
		{
			ofLogError() << "Float count not divisible by "
						 << DIMS_PER_BOID << ": " << totalFloats;
			return;
		}

		int numBoids = totalFloats / DIMS_PER_BOID;
		if (boids.size() != numBoids)
		{
			boids.clear();
			boids.resize(numBoids);
			for (int i = 0; i < numBoids; i++)
				boids[i].index = i;
		}

		int argIndex = 0;
		for (int i = 0; i < numBoids; i++)
		{
			for (int d = 0; d < DIMS_PER_BOID; d++)
			{
				float v = msg.getArgAsFloat(argIndex++);
				if (type == "position")
				{
					boids[i].position[d] = v;
				}
				else if (type == "velocity")
				{
					boids[i].velocity[d] = v;
				}
			}
		}
	}
};
