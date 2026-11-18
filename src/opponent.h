#ifndef OPPONENT_H_IS_INCLUDED
#define OPPONENT_H_IS_INCLUDED

// Opponent Logic (Anthony Lin)
//
// Decision making for the robot side.  Works with either robot: it only uses the Robot interface
// (TimeToReach, CommandMoveTo, CommandStrike, IsCommandDone).
//
//   GUARD    default: stay on the defense line in front of the goal, shifted toward the puck
//   DEFEND   puck is fast and coming in: intercept it where it crosses the defense line
//   ATTACK   puck is slow: pick a hit point on its path and strike along the best of 25 shots
//   RECOVER  just struck: wait while the robot returns home

#include <vector>

#include "shared.h"
#include "robot.h"

// A predicted puck state, t seconds from now.
struct PuckSample
{
	double t;
	Vec2 pos, vel;
};

// The puck's predicted path, with no mallets on the table, earliest sample first.
struct PuckPath
{
	std::vector<PuckSample> samples;
};

// Stand-in for the physics component's puck prediction, until physics provides one.
// Advances a puck by dt with friction, the speed cap, and wall/goal-post bounces; no mallets.
void StepFreePuck(PuckState &puck, const TableConfig &cfg, double dt);

// Samples the puck's path every dt seconds for horizon seconds, stopping if it enters a goal.
PuckPath PredictPuckPath(PuckState puck, const TableConfig &cfg, double horizon, double dt);

// One candidate shot from a hit point to the human goal.
struct Shot
{
	Vec2 goalPoint;            // aim point on the human goal line
	int nBounce = 0;           // 0, 1 or 2 side-wall bounces
	Vec2 dir;                  // puck direction right after the hit
	std::vector<Vec2> path;    // hit point -> wall bounces -> goal line
	bool open = false;         // path clears the human mallet by a safety margin
	bool reachable = false;    // robot can get to the wind-up point in time
	double cost = 0.0;         // robot's time to reach the wind-up point
};

class OpponentLogic
{
public:
	enum Mode
	{
		GUARD,
		DEFEND,
		ATTACK,
		RECOVER
	};

	OpponentLogic(const TableConfig &cfg);

	// Sets the DEFEND/ATTACK speed threshold
	void SetDifficulty(Difficulty level);

	// Clears the plan, mode = GUARD.  Call together with the puck and robot resets after a goal.
	void Reset(void);

	// Tell the opponent what the physics step reported.  A mallet hit cancels a planned attack.
	void OnPhysicsEvent(PhysicsEvent ev);

	// Call once per frame (not while the puck is frozen after a goal).
	// Picks the mode and sends this frame's command to the robot.
	void Update(Robot &robot, PuckState puck, MalletState human, const PuckPath &path, double dt);

	Mode GetMode(void) const;

	// Mode name for the HUD
	const char *GetModeName(void) const;
	
	// Defense line, candidate shots and the planned hit, for testing
	void DrawDebug(void) const;

	// Up to 25 possible shots from hitPoint into the human goal (5 aim points x 5 bounce routes).
	// Sets dir, path and open (not blocked by the human mallet); PlanAttack sets reachable and cost.
	std::vector<Shot> GenerateShots(Vec2 hitPoint, MalletState human) const;

	// Debug accessors for the test program
	double GetAttackSpeed(void) const;
	double GetDefenseY(void) const;
	Vec2 GetLastTarget(void) const;

private:
	TableConfig cfg;
	Mode mode = GUARD;
	double clock = 0.0;            // seconds since Reset, so plans can store absolute hit times
	double attackSpeed = 450.0;    // pucks slower than this get attacked
	bool malletHit = false;        // a mallet touched the puck since the last Update

	// Planned attack
	Vec2 hitPoint;                 // predicted puck center at the hit
	double hitTime = 0.0;          // when the puck will be there (on clock)
	Shot plannedShot;
	std::vector<Shot> candidates;  // last evaluated shot set, for DrawDebug

	Vec2 lastTarget;               // last point sent to the robot, for DrawDebug

	// Where to send the robot in each mode
	Vec2 GuardTarget(PuckState puck) const;
	Vec2 DefendTarget(const Robot &robot, PuckState puck, const PuckPath &path) const;
	// Finds a hit point and shot; true if an attack is planned
	bool PlanAttack(const Robot &robot, MalletState human, const PuckPath &path);
	// Moves to the wind-up point, strikes on time, or gives up the plan
	void ContinueAttack(Robot &robot);
	void MoveTo(Robot &robot, Vec2 target);

	// Shot geometry helpers for GenerateShots
	bool TraceShot(Vec2 from, Vec2 dir, int maxBounce, std::vector<Vec2> &path, std::vector<int> &walls) const;
	bool EntersGoalCleanly(const std::vector<Vec2> &path) const;
	bool InStrikeZone(Vec2 p) const;
	double ContactDistance(const Robot &robot) const;
};

#endif
