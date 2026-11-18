#pragma once

const double PI = 3.14159265358979;	// value of pi

class Vec2 {
public:
	double x, y;
};

// Inputs to game flow to tell it what to do or what sounds to play
enum PhysicsEvent { EV_NONE, EV_WALL_HIT, EV_MALLET_HIT, EV_GOAL_HUMAN, EV_GOAL_ROBOT };
enum Difficulty { EASY, MEDIUM, HARD };

// Table parameters
// Restitution is how much momentum is maintained following the collision (not perfectly elastic)

const double TABLE_OX = 50;
const double TABLE_OY = 50;

struct TableConfig {
	double width = 400, height = 800, goalWidth = 150;
	double wallRestitution = 0.9, friction = 0.15, maxPuckSpeed = 2200;
	double puckRadius = 18, malletRadius = 30, malletRestitution = 0.9, maxHumanSpeed = 2700;
	double restThreshold = 5;
};

struct PuckState { Vec2 pos, vel; double radius; };
struct MalletState { Vec2 pos, vel; double radius; };
struct PredictResult { bool found; Vec2 point; double time; };

struct GameState {
	PuckState puck;
	MalletState human, robot;
	int humanScore, robotScore;
};

// declarations
Vec2 operator+(Vec2 a, Vec2 b);
Vec2 operator-(Vec2 a, Vec2 b);
Vec2 operator*(Vec2 a, Vec2 b);
Vec2 operator*(Vec2 a, double s);
Vec2 operator/(Vec2 a, double s);
double Dot(Vec2 a, Vec2 b);
double Length(Vec2 a);
Vec2 Normalize(Vec2 a);
double Clamp(double v, double lo, double hi);

void DrawCircle(double cx, double cy, double rad);
void DrawRect(double x1, double y1, double x2, double y2, bool fill);
void DrawRoundedRect(double x1, double y1, double x2, double y2, bool fill, double r);
void DrawGradientRect(double x, double y, double w, double h, const int topCol[3], const int botCol[3]);
void DrawQuad(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);
double RandRange(double min, double max);

