#pragma once

#include "shared.h"          // Vec2, PuckState, TableConfig, PhysicsEvent

class Puck {
public:
	PuckState s;             // position, velocity, radius

	void Reset(Vec2 startPos, const TableConfig& cfg);
	PhysicsEvent Update(double dt, const TableConfig& cfg);
	bool CollideWithMallet(const MalletState& m, const TableConfig& cfg);
	void Draw() const;

private:
	bool EdgeBounce(Vec2 edge, const TableConfig& cfg);
	PhysicsEvent WallBounce(const TableConfig& cfg);
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
	void Draw() const;
};