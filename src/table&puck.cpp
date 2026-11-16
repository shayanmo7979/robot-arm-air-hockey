#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <chrono>

#include "shared_functions.cpp"
#include "fssimplewindow.h"
#include "ysglfontdata.h"
#include "yssimplesound.h"

struct PuckState{
	Vec2 pos;
	Vec2 vel;
	double rad;
};

class Puck {
public:	

	PuckState s;

	void Reset(Vec2 startPos) {
		// Reset (Set position to table center)
	}

	void Update(double dt) {
		// 1. Friction slows the puck
		// s.vel = s.vel * max(0, 1 - cfg.friction * dt)

		// 2. Cap the speed
		// IF Length(s.vel) > cfg.maxPuckSpeed  --> s.vel = Normalize(s.vel) * cfg.maxPuckSpeed

		// 3. Move
		// s.pos = s.pos + s.vel * dt

		// 4. Check walls and goals return what happened
		// RETURN HandleWalls(cfg)
	}
	void Draw() const {
		// Draw the puck as a circle at its current position
	}
};

