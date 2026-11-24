#pragma once

#include "shared.h"          // Vec2, PuckState, TableConfig, PhysicsEvent

class Puck {
public:
	PuckState s;             // position, velocity, radius

	void Reset(Vec2 startPos, const TableConfig& cfg);
	PhysicsEvent Update(double dt, const TableConfig& cfg, bool goalsOpen = true);
	bool CollideWithMallet(const MalletState& m, const TableConfig& cfg);
	Vec2 PredictPosition(double t, const TableConfig& cfg) const;
	PredictResult PredictLineCrossing(double lineY, double maxTime, const TableConfig& cfg) const;
	void Draw() const;

private:
	bool EdgeBounce(Vec2 edge, const TableConfig& cfg);
	PhysicsEvent WallBounce(const TableConfig& cfg, bool goalsOpen = true);
};

class Table {
private:
	TableConfig cfg;       // width, height, goalWidth, friction, restitution, radii...
	double ox, oy;         // where the table's top-left corner sits on screen (margin)

public:
	Table();
	const TableConfig& GetConfig() const;
	double GoalLeft() const;     // optional helpers
	double GoalRight() const;
	void Draw() const;
};

class HumanMallet {
public:
	MalletState s;           // position, velocity, radius

	void Reset(const TableConfig& cfg);
	void Update(double dt, int mouseX, int mouseY, const TableConfig& cfg);
	MalletState GetState() const { return s; }
	void Draw() const;
};

class Physics {
private:
	Table table;
	Puck puck;
	HumanMallet human;

public:
	Physics();                                   // builds everything and resets
	void ResetPuck(PhysicsEvent lastGoal);       // serve toward whoever was just scored on
	PhysicsEvent Step(double dt, int mouseX, int mouseY, const MalletState& robotMallet);

	// Read-only access for Game Flow and the Opponent
	const Puck& GetPuck() const { return puck; }
	MalletState GetHumanState() const { return human.GetState(); }
	const TableConfig& GetConfig() const { return table.GetConfig(); }

	void Draw() const;                           // table, puck, human mallet, in that order
};