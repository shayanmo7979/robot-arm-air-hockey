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

	void Reset(Vec2 startPos);
	void Update(double dt);
	void Draw(); const
};

