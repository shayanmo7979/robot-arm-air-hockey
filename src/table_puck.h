#pragma once

#include "shared.h"          // Vec2, PuckState, TableConfig, PhysicsEvent

class Puck {
public:
	PuckState s;             // position, velocity, radius

	void Reset(Vec2 startPos, const TableConfig& cfg);
	PhysicsEvent Update(double dt, const TableConfig& cfg);
	void Draw() const;

private:
	bool EdgeBounce(Vec2 edge, const TableConfig& cfg);
	PhysicsEvent WallBounce(const TableConfig& cfg);
	bool CollideWithMallet(const MalletState& m, const TableConfig& cfg);
};