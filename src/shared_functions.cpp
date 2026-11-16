#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "fssimplewindow.h"
#include "ysglfontdata.h"

const double PI = 3.1415926;	// value of pi

// Draw a filled circle at (cx, cy) of radius rad
void DrawCircle(double cx, double cy, double rad) {
	glBegin(GL_TRIANGLE_FAN);

	for (int i = 0; i < 64; i++)
	{
		double angle = (double)i * PI / 32.0;
		double x = cx + cos(angle) * rad;
		double y = cy + sin(angle) * rad;
		glVertex2d(x, y);
	}
	glEnd();
}

// Draw a rectangle from (x1, y1) to (x2, y2), filled based on bool fill
void DrawRect(double x1, double y1, double x2, double y2, bool fill) {
	if (fill) glBegin(GL_QUADS);

	else glBegin(GL_LINE_LOOP);

	glVertex2d(x1, y1);
	glVertex2d(x2, y1);
	glVertex2d(x2, y2);
	glVertex2d(x1, y2);

	glEnd();
}

// Draw a rectangle with rounded corners from (x1, y1) to (x2, y2) with corner radius r
void DrawRoundedRect(double x1, double y1, double x2, double y2, bool fill, double r) {
	DrawRect(x1, y1 + r, x2, y2 - r, fill);
	DrawRect(x1 + r, y1, x2 - r, y2, fill);
	DrawCircle(x1 + r, y1 + r, r);
	DrawCircle(x1 + r, y2 - r, r);
	DrawCircle(x2 - r, y1 + r, r);
	DrawCircle(x2 - r, y2 - r, r);
}

// Draw a rectangle with a color gradient from top to bottom with center (x,y), width w, height h
// Use gl blend to make gradient from topCol at top to botCol at bottom
void DrawGradientRect(double x, double y, double w, double h,
	const int topCol[3], const int botCol[3])
{
	glShadeModel(GL_SMOOTH);
	glBegin(GL_QUADS);
	glColor3ub(topCol[0], topCol[1], topCol[2]);
	glVertex2i(x, y);
	glVertex2i(x + w, y);
	glColor3ub(botCol[0], botCol[1], botCol[2]);
	glVertex2i(x + w, y + h);
	glVertex2i(x, y + h);
	glEnd();
	glShadeModel(GL_FLAT);
}

// Draw a generic quadrilateral shape (used for trapezoid perspective road in front view)
void DrawQuad(double x1, double y1, double x2, double y2,
	double x3, double y3, double x4, double y4) {
	glBegin(GL_QUADS);
	glVertex2d(x1, y1);
	glVertex2d(x2, y2);
	glVertex2d(x3, y3);
	glVertex2d(x4, y4);
	glEnd();
}

// Return a random number in a range between min and max
double RandRange(double min, double max) {
	return min + double(rand()) * (max - min) / RAND_MAX;
}

class Vec2 {
public:
	double x, y;
};

Vec2 operator+(Vec2 a, Vec2 b) {
	return {a.x + b.x, a.y + b.y};
}

Vec2 operator-(Vec2 a, Vec2 b) {
	return { a.x - b.x, a.y - b.y };
}

Vec2 operator*(Vec2 a, Vec2 b) {
	return { a.x * b.x, a.y * b.y };
}

double Dot(Vec2 a, Vec2 b) {
	return a.x * b.x + a.y * b.y;
}

double Length(Vec2 a) {
	return sqrt(a.x * a.x + a.y * a.y);
}

Vec2 Normalize(Vec2 a) {
	double L = Length(a);
	if (L == 0) {
		return { 0, 0 };
	}
	return { a.x / L, a.y / L };
}

enum PhysicsEvent {EV_NON, EV_WALL_HIT, EV_MALLET_HIT, EV_GOAL_HUMAN, EV_GOAL_ROBOT};
enum WallId {WALL_LEF, WALL_RIGHT, WALL_TOP, WALL_BOTTOM};

struct TableConfig {
	double width, height, goalWidth;
	double wallRestitution, friction, maxPuckSpeed;
	double puckRadius, malletRadius, malletRestitution, maxHumanSpeed;
};

struct PuckState { Vec2 pos, vel; double radius; };
struct MalletState { Vec2 pos, vel; double radius; };

struct PredictResult { bool found; Vec2 point; double time; };

struct GameState {
	PuckState puck;
	MalletState human, robot;
	int humanScore, robotScore;
};