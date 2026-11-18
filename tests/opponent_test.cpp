// Test program for the Opponent Logic component (Anthony Lin)
//
// Console checks: test OpponentLogic on its own, with stand-ins for the other components:
//   FakeRobot  - a point mallet with a speed limit that implements the shared Robot interface
//   SimStep    - a small puck simulation (StepFreePuck + mallet contact) for running rallies
//
// Scenario window: runs fixed test situations so the opponent's decisions can be watched,
// with either the FakeRobot or the team's real MobileRobot (same Robot interface).
// The puck still uses SimStep until the Table & Puck code compiles.
//
// opponent_test         runs the console checks, then opens the scenario window
// opponent_test check   runs the console checks only
//
// Window controls: 1-4 = scenario, SPACE = replay, T = switch robot, E/M/H = difficulty, ESC = quit.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <chrono>

#include "fssimplewindow.h"
#include "ysglfontdata.h"
#include "shared.h"
#include "robot.h"
#include "mobile_robot.h"
#include "opponent.h"

const double FRAME_TIME = 0.016;
const int NUM_SUBSTEP = 4;
const double PREDICT_HORIZON = 2.0;
const double PREDICT_DT = 1.0 / 120.0;
const double VIEW_MARGIN = 40.0;
const double VIEW_SCALE = 0.75;
const double FOLLOW_THROUGH = 40.0;

int nPass = 0;
int nFail = 0;



////////////////////////////////////////////////////////////
// FakeRobot: a mallet that moves in a straight line at a capped speed on the robot's half

class FakeRobot : public Robot
{
private:
	enum State
	{
		HOLD,
		MOVING,
		STRIKE,
		RECOVER
	};

	TableConfig cfg;
	State state = HOLD;
	MalletState paddle;
	Vec2 home, target, strikeEnd;
	double maxSpeed = 600.0;
	double strikeSpeed = 0.0;

	Vec2 ClampToHalf(Vec2 p) const;
	bool MoveToward(Vec2 goal, double speed, double dt);

public:
	FakeRobot(const TableConfig &cfg);
	void SetDifficulty(Difficulty level) override;
	void Reset(void) override;
	void CommandMoveTo(Vec2 target) override;
	void CommandStrike(Vec2 hitPoint, Vec2 direction, double speed) override;
	double TimeToReach(Vec2 point) const override;
	bool IsCommandDone(void) const override;
	void Update(double dt) override;
	MalletState GetPaddleState(void) const override;
	void Draw(void) const override;

	int nStrike = 0;
};

FakeRobot::FakeRobot(const TableConfig &cfg) : cfg(cfg)
{
	home = {cfg.width / 2.0, 80.0};
	paddle.radius = cfg.malletRadius;
	Reset();
}

void FakeRobot::SetDifficulty(Difficulty level)
{
	const double speed[3] = {400.0, 600.0, 900.0};
	maxSpeed = speed[level];
}

void FakeRobot::Reset(void)
{
	state = HOLD;
	paddle.pos = home;
	paddle.vel = {0.0, 0.0};
	target = home;
}

Vec2 FakeRobot::ClampToHalf(Vec2 p) const
{
	double r = paddle.radius;
	return {Clamp(p.x, r, cfg.width - r), Clamp(p.y, r, cfg.height / 2.0 - r)};
}

void FakeRobot::CommandMoveTo(Vec2 t)
{
	if(state == STRIKE)
	{
		return;   // a strike can't be interrupted
	}
	target = ClampToHalf(t);
	state = MOVING;
}

void FakeRobot::CommandStrike(Vec2 hitPoint, Vec2 direction, double speed)
{
	strikeEnd = ClampToHalf(hitPoint + direction * FOLLOW_THROUGH);
	strikeSpeed = speed;
	state = STRIKE;
	nStrike++;
}

double FakeRobot::TimeToReach(Vec2 point) const
{
	double r = paddle.radius;
	if(point.x < r || point.x > cfg.width - r || point.y < r || point.y > cfg.height / 2.0 - r)
	{
		return 1e9;
	}
	return Length(point - paddle.pos) / maxSpeed;
}

bool FakeRobot::IsCommandDone(void) const
{
	return (state == HOLD);
}

// Returns true when the goal is reached.
bool FakeRobot::MoveToward(Vec2 goal, double speed, double dt)
{
	Vec2 d = goal - paddle.pos;
	double dist = Length(d);
	double step = speed * dt;
	if(dist <= step)
	{
		paddle.pos = goal;
		return true;
	}
	paddle.pos = paddle.pos + d * (step / dist);
	return false;
}

void FakeRobot::Update(double dt)
{
	Vec2 prev = paddle.pos;
	if(state == MOVING && MoveToward(target, maxSpeed, dt))
	{
		state = HOLD;
	}
	else if(state == STRIKE && MoveToward(strikeEnd, strikeSpeed, dt))
	{
		state = RECOVER;
	}
	else if(state == RECOVER && MoveToward(home, maxSpeed, dt))
	{
		state = HOLD;
	}
	paddle.vel = (paddle.pos - prev) / dt;
}

MalletState FakeRobot::GetPaddleState(void) const
{
	return paddle;
}

void FakeRobot::Draw(void) const
{
	glColor3ub(40, 90, 200);
	DrawCircle(paddle.pos.x, paddle.pos.y, paddle.radius);
}



////////////////////////////////////////////////////////////
// Small simulation for rallies: free puck motion, then contact with each mallet

bool CollideMallet(PuckState &puck, MalletState m, const TableConfig &cfg)
{
	Vec2 d = puck.pos - m.pos;
	double dist = Length(d);
	double minDist = puck.radius + m.radius;
	if(dist >= minDist || dist == 0.0)
	{
		return false;
	}
	Vec2 n = d / dist;
	puck.pos = m.pos + n * minDist;
	double vn = Dot(puck.vel - m.vel, n);
	if(vn < 0.0)
	{
		puck.vel = puck.vel - n * ((1.0 + cfg.malletRestitution) * vn);
	}
	return true;
}

// Returns the most important event: a goal, else a mallet hit, else EV_NONE.
// robotHit is set when the robot's mallet touched the puck.
PhysicsEvent SimStep(PuckState &puck, MalletState human, MalletState robot, const TableConfig &cfg,
                     double dt, bool &robotHit)
{
	StepFreePuck(puck, cfg, dt);
	if(puck.pos.y < -puck.radius)
	{
		return EV_GOAL_HUMAN;   // into the robot's goal (top)
	}
	if(puck.pos.y > cfg.height + puck.radius)
	{
		return EV_GOAL_ROBOT;   // into the human's goal (bottom)
	}
	bool humanHit = CollideMallet(puck, human, cfg);
	robotHit = CollideMallet(puck, robot, cfg);
	if(humanHit || robotHit)
	{
		return EV_MALLET_HIT;
	}
	return EV_NONE;
}



////////////////////////////////////////////////////////////
// Console checks

void Check(bool ok, const char label[])
{
	if(ok)
	{
		nPass++;
	}
	else
	{
		nFail++;
	}
	printf("[%s] %s\n", ok ? "PASS" : "FAIL", label);
}

MalletState MakeMallet(double x, double y, const TableConfig &cfg)
{
	MalletState m;
	m.pos = {x, y};
	m.vel = {0.0, 0.0};
	m.radius = cfg.malletRadius;
	return m;
}

PuckState MakePuck(double x, double y, double vx, double vy, const TableConfig &cfg)
{
	PuckState p;
	p.pos = {x, y};
	p.vel = {vx, vy};
	p.radius = cfg.puckRadius;
	return p;
}

// Every generated shot must start at the hit point, end inside the human goal mouth, bounce
// off the side walls only, and have one path point per bounce.
void CheckShotGeometry(const TableConfig &cfg)
{
	OpponentLogic opp(cfg);
	MalletState farHuman = MakeMallet(-1000.0, -1000.0, cfg);   // off the table: blocks nothing
	Vec2 from = {200.0, 300.0};
	std::vector<Shot> shots = opp.GenerateShots(from, farHuman);

	int count[3] = {0, 0, 0};
	bool geometryOk = true;
	bool allOpen = true;
	double half = cfg.goalWidth / 2.0 - cfg.puckRadius;
	for(const Shot &s : shots)
	{
		count[s.nBounce]++;
		Vec2 end = s.path.back();
		if((int)s.path.size() != s.nBounce + 2 || fabs(end.y - cfg.height) > 1e-6 ||
		   fabs(end.x - cfg.width / 2.0) > half || Length(s.path[0] - from) > 1e-6)
		{
			geometryOk = false;
		}
		for(int i = 1; i + 1 < (int)s.path.size(); i++)
		{
			double x = s.path[i].x;
			if(fabs(x - cfg.puckRadius) > 1e-6 && fabs(x - (cfg.width - cfg.puckRadius)) > 1e-6)
			{
				geometryOk = false;
			}
		}
		if(!s.open)
		{
			allOpen = false;
		}
	}
	printf("       shots from (200,300): %d total, %d direct, %d one-bank, %d two-bank\n",
	       (int)shots.size(), count[0], count[1], count[2]);
	Check(count[0] == 5 && count[1] >= 5 && count[2] >= 5, "shot generation: direct, 1-bank and 2-bank shots exist");
	Check(geometryOk, "shot generation: paths start at the hit point, bounce on side walls, end in the goal mouth");
	Check(allOpen, "shot generation: all shots open when the human mallet is out of the way");
}

// A human mallet on the straight line to the goal blocks the direct shots, not the bank shots.
void CheckBlockedShots(const TableConfig &cfg)
{
	OpponentLogic opp(cfg);
	MalletState blocker = MakeMallet(200.0, 600.0, cfg);
	std::vector<Shot> shots = opp.GenerateShots({200.0, 300.0}, blocker);

	bool directBlocked = true;
	bool someBankOpen = false;
	for(const Shot &s : shots)
	{
		if(s.nBounce == 0 && s.open)
		{
			directBlocked = false;
		}
		if(s.nBounce > 0 && s.open)
		{
			someBankOpen = true;
		}
	}
	Check(directBlocked, "blocked shots: human mallet in front of the goal blocks every direct shot");
	Check(someBankOpen, "blocked shots: some bank shot still gets around the mallet");
}

void CheckModes(const TableConfig &cfg)
{
	MalletState human = MakeMallet(200.0, 700.0, cfg);

	// Fast puck coming at the robot's goal -> DEFEND, target on or behind the defense line,
	// near where the puck crosses it.
	{
		FakeRobot robot(cfg);
		OpponentLogic opp(cfg);
		opp.SetDifficulty(MEDIUM);
		PuckState puck = MakePuck(260.0, 500.0, -60.0, -900.0, cfg);
		PuckPath path = PredictPuckPath(puck, cfg, PREDICT_HORIZON, PREDICT_DT);
		opp.Update(robot, puck, human, path, FRAME_TIME);

		double crossX = 0.0;
		for(const PuckSample &s : path.samples)
		{
			if(s.pos.y <= opp.GetDefenseY())
			{
				crossX = s.pos.x;
				break;
			}
		}
		Vec2 t = opp.GetLastTarget();
		printf("       DEFEND target (%.1f, %.1f), puck crosses the line at x=%.1f\n", t.x, t.y, crossX);
		Check(opp.GetMode() == OpponentLogic::DEFEND, "modes: fast incoming puck -> DEFEND");
		Check(t.y <= opp.GetDefenseY() + 1e-6 && fabs(t.x - crossX) < 25.0, "modes: DEFEND target is at the predicted crossing");
	}

	// Puck on the human's half moving away -> GUARD, shifted toward the puck.
	{
		FakeRobot robot(cfg);
		OpponentLogic opp(cfg);
		PuckState puck = MakePuck(300.0, 600.0, 0.0, 300.0, cfg);
		PuckPath path = PredictPuckPath(puck, cfg, PREDICT_HORIZON, PREDICT_DT);
		opp.Update(robot, puck, human, path, FRAME_TIME);
		Vec2 t = opp.GetLastTarget();
		Check(opp.GetMode() == OpponentLogic::GUARD, "modes: puck leaving on the human's half -> GUARD");
		Check(t.x > cfg.width / 2.0 && fabs(t.y - opp.GetDefenseY()) < 1e-6, "modes: GUARD shifts toward the puck on the defense line");
	}

	// Puck resting on the robot's half -> ATTACK.
	{
		FakeRobot robot(cfg);
		OpponentLogic opp(cfg);
		PuckState puck = MakePuck(260.0, 250.0, 0.0, 0.0, cfg);
		PuckPath path = PredictPuckPath(puck, cfg, PREDICT_HORIZON, PREDICT_DT);
		opp.Update(robot, puck, human, path, FRAME_TIME);
		Check(opp.GetMode() == OpponentLogic::ATTACK, "modes: slow puck on the robot's half -> ATTACK");
	}

	// The same 300 px/s incoming puck is defended on Easy and attacked on Hard.
	{
		Difficulty level[2] = {EASY, HARD};
		OpponentLogic::Mode expect[2] = {OpponentLogic::DEFEND, OpponentLogic::ATTACK};
		for(int i = 0; i < 2; i++)
		{
			FakeRobot robot(cfg);
			OpponentLogic opp(cfg);
			robot.SetDifficulty(level[i]);
			opp.SetDifficulty(level[i]);
			PuckState puck = MakePuck(220.0, 330.0, 0.0, -300.0, cfg);
			PuckPath path = PredictPuckPath(puck, cfg, PREDICT_HORIZON, PREDICT_DT);
			opp.Update(robot, puck, human, path, FRAME_TIME);
			printf("       difficulty %d: attack speed %.0f -> %s\n", (int)level[i], opp.GetAttackSpeed(), opp.GetModeName());
			Check(opp.GetMode() == expect[i], i == 0 ? "difficulty: Easy defends a 300 px/s puck" : "difficulty: Hard attacks a 300 px/s puck");
		}
	}

	// A robot whose TimeToReach is not implemented yet (returns -1, like the RobotArm stub)
	// must not attack, but still gets move commands.
	{
		class StubRobot : public FakeRobot
		{
		public:
			StubRobot(const TableConfig &cfg) : FakeRobot(cfg) {}
			double TimeToReach(Vec2) const override
			{
				return -1.0;
			}
		};
		StubRobot robot(cfg);
		OpponentLogic opp(cfg);
		PuckState puck = MakePuck(260.0, 250.0, 0.0, 0.0, cfg);
		PuckPath path = PredictPuckPath(puck, cfg, PREDICT_HORIZON, PREDICT_DT);
		opp.Update(robot, puck, human, path, FRAME_TIME);
		Check(opp.GetMode() == OpponentLogic::GUARD, "stub robot: TimeToReach() < 0 is treated as unreachable (no attack)");
	}
}

// Runs a whole rally headless: the opponent should strike a resting puck into the human goal.
void CheckRally(const TableConfig &cfg, double startX, double startY, double humanX)
{
	FakeRobot robot(cfg);
	OpponentLogic opp(cfg);
	robot.SetDifficulty(MEDIUM);
	opp.SetDifficulty(MEDIUM);
	PuckState puck = MakePuck(startX, startY, 0.0, 0.0, cfg);
	MalletState human = MakeMallet(humanX, 720.0, cfg);

	bool sawAttack = false;
	bool sawRecover = false;
	bool robotTouched = false;
	PhysicsEvent result = EV_NONE;
	double t = 0.0;
	while(t < 5.0 && result != EV_GOAL_HUMAN && result != EV_GOAL_ROBOT)
	{
		PuckPath path = PredictPuckPath(puck, cfg, PREDICT_HORIZON, PREDICT_DT);
		opp.Update(robot, puck, human, path, FRAME_TIME);
		sawAttack = sawAttack || (opp.GetMode() == OpponentLogic::ATTACK);
		sawRecover = sawRecover || (opp.GetMode() == OpponentLogic::RECOVER);
		for(int i = 0; i < NUM_SUBSTEP; i++)
		{
			double h = FRAME_TIME / NUM_SUBSTEP;
			robot.Update(h);
			bool robotHit = false;
			PhysicsEvent ev = SimStep(puck, human, robot.GetPaddleState(), cfg, h, robotHit);
			robotTouched = robotTouched || robotHit;
			opp.OnPhysicsEvent(ev);
			if(ev == EV_GOAL_HUMAN || ev == EV_GOAL_ROBOT)
			{
				result = ev;
				break;
			}
		}
		t = t + FRAME_TIME;
	}

	char label[256];
	sprintf(label, "rally: puck at (%.0f,%.0f), human at x=%.0f -> ATTACK, strike, RECOVER, robot scores", startX, startY, humanX);
	printf("       %s after %.2f s, strikes=%d\n", result == EV_GOAL_ROBOT ? "robot scored" : "no robot goal", t, robot.nStrike);
	Check(sawAttack && sawRecover && robotTouched && result == EV_GOAL_ROBOT, label);
}

// 25 shots generated and scored (with TimeToReach) must take under 1 ms.
void CheckTiming(const TableConfig &cfg)
{
	const int nRepeat = 2000;
	FakeRobot robot(cfg);
	OpponentLogic opp(cfg);
	MalletState human = MakeMallet(150.0, 650.0, cfg);

	auto t0 = std::chrono::high_resolution_clock::now();
	double sink = 0.0;
	for(int i = 0; i < nRepeat; i++)
	{
		std::vector<Shot> shots = opp.GenerateShots({120.0 + (double)(i % 160), 260.0}, human);
		for(const Shot &s : shots)
		{
			sink = sink + robot.TimeToReach(s.path[0] - s.dir * 100.0);
		}
	}
	auto t1 = std::chrono::high_resolution_clock::now();
	double usec = std::chrono::duration<double, std::micro>(t1 - t0).count() / (double)nRepeat;
	printf("       one hit point (25 candidates): %.1f us (sink %.0f)\n", usec, sink);
	Check(usec < 1000.0, "timing: one set of candidate shots is generated and scored in under 1 ms");
}

void RunChecks(const TableConfig &cfg)
{
	printf("=== Opponent Logic checks ===\n");
	CheckShotGeometry(cfg);
	CheckBlockedShots(cfg);
	CheckModes(cfg);
	CheckRally(cfg, 260.0, 250.0, 60.0);
	CheckRally(cfg, 120.0, 300.0, 200.0);
	CheckTiming(cfg);
	printf("=== %d passed, %d failed ===\n", nPass, nFail);
}



////////////////////////////////////////////////////////////
// Scenario window

void SetTableProjection(const TableConfig &cfg)
{
	int wid;
	int hei;
	FsGetWindowSize(wid, hei);
	glViewport(0, 0, wid, hei);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-VIEW_MARGIN, cfg.width + VIEW_MARGIN, cfg.height + VIEW_MARGIN, -VIEW_MARGIN, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void DrawTable(const TableConfig &cfg)
{
	double goalLeft = cfg.width / 2.0 - cfg.goalWidth / 2.0;
	double goalRight = cfg.width / 2.0 + cfg.goalWidth / 2.0;

	glColor3ub(235, 240, 245);
	DrawRect(0.0, 0.0, cfg.width, cfg.height, true);
	glColor3ub(80, 80, 80);
	DrawRect(0.0, 0.0, cfg.width, cfg.height, false);
	glBegin(GL_LINES);
	glVertex2d(0.0, cfg.height / 2.0);
	glVertex2d(cfg.width, cfg.height / 2.0);
	glEnd();

	glColor3ub(40, 90, 200);
	DrawRect(goalLeft, -8.0, goalRight, 0.0, true);
	glColor3ub(200, 60, 60);
	DrawRect(goalLeft, cfg.height, goalRight, cfg.height + 8.0, true);
}

void DrawText(double x, double y, const char str[])
{
	glRasterPos2d(x, y);
	YsGlDrawFontBitmap8x12(str);
}

// A fixed test situation: puck start, human mallet spot (it stays still), and what should happen.
struct Scenario
{
	const char *name;
	double puckX, puckY, puckVx, puckVy;
	double humanX, humanY;
	const char *expect;
};

const int NUM_SCENARIO = 4;
const Scenario SCENARIOS[NUM_SCENARIO] = {
	{"1: fast shot at the robot's goal", 260.0, 520.0, -60.0, -850.0, 120.0, 700.0, "DEFEND, block at the line"},
	{"2: slow puck on robot's half", 260.0, 250.0, 0.0, 0.0, 60.0, 720.0, "ATTACK, strike, RECOVER"},
	{"3: human blocks the middle", 200.0, 300.0, 0.0, 0.0, 200.0, 600.0, "ATTACK with a bank shot"},
	{"4: puck stays on human's half", 300.0, 500.0, 0.0, 100.0, 100.0, 720.0, "GUARD, shift toward puck"},
};
const double SCENARIO_TIME = 6.0;   // stop a scenario after this long

const int NUM_ROBOT = 2;
const char *const ROBOT_NAME[NUM_ROBOT] = {"FakeRobot", "MobileRobot"};
const char *const LEVEL_NAME[3] = {"EASY", "MEDIUM", "HARD"};

class ScenarioViewer
{
private:
	TableConfig cfg;
	FakeRobot fake;
	MobileRobot mobile;
	Robot *robots[NUM_ROBOT];
	int robotIdx = 0;
	OpponentLogic opp;
	Difficulty level = MEDIUM;

	int scenarioIdx = 0;
	PuckState puck;
	MalletState human;
	PuckPath path;
	double clock = 0.0;
	bool finished = false;
	char result[128];

	void StartScenario(int idx);
	void SetLevel(Difficulty newLevel);
	void Step(void);

public:
	ScenarioViewer(const TableConfig &cfg);
	bool RunOneStep(void);   // false when ESC is pressed
	void Render(void) const;
};

ScenarioViewer::ScenarioViewer(const TableConfig &cfg) : cfg(cfg), fake(cfg), mobile(cfg), opp(cfg)
{
	robots[0] = &fake;
	robots[1] = &mobile;
	SetLevel(MEDIUM);
	StartScenario(0);
}

void ScenarioViewer::SetLevel(Difficulty newLevel)
{
	level = newLevel;
	for(int i = 0; i < NUM_ROBOT; i++)
	{
		robots[i]->SetDifficulty(level);
	}
	opp.SetDifficulty(level);
}

void ScenarioViewer::StartScenario(int idx)
{
	const Scenario &s = SCENARIOS[idx];
	scenarioIdx = idx;
	puck = MakePuck(s.puckX, s.puckY, s.puckVx, s.puckVy, cfg);
	human = MakeMallet(s.humanX, s.humanY, cfg);
	for(int i = 0; i < NUM_ROBOT; i++)
	{
		robots[i]->Reset();
	}
	opp.Reset();
	path = PredictPuckPath(puck, cfg, PREDICT_HORIZON, PREDICT_DT);
	clock = 0.0;
	finished = false;
	result[0] = 0;
}

// One frame, in the order the real game will use: predict, opponent, then robot + physics sub-steps.
void ScenarioViewer::Step(void)
{
	Robot &robot = *robots[robotIdx];
	path = PredictPuckPath(puck, cfg, PREDICT_HORIZON, PREDICT_DT);
	opp.Update(robot, puck, human, path, FRAME_TIME);

	for(int i = 0; i < NUM_SUBSTEP; i++)
	{
		double h = FRAME_TIME / NUM_SUBSTEP;
		robot.Update(h);
		bool robotHit = false;
		PhysicsEvent ev = SimStep(puck, human, robot.GetPaddleState(), cfg, h, robotHit);
		opp.OnPhysicsEvent(ev);
		if(ev == EV_GOAL_ROBOT || ev == EV_GOAL_HUMAN)
		{
			sprintf(result, "Result: %s at %.2f s", ev == EV_GOAL_ROBOT ? "robot scored" : "robot conceded", clock);
			finished = true;
			return;
		}
	}

	clock = clock + FRAME_TIME;
	if(clock >= SCENARIO_TIME)
	{
		sprintf(result, "Result: no goal in %.0f s", SCENARIO_TIME);
		finished = true;
	}
}

bool ScenarioViewer::RunOneStep(void)
{
	FsPollDevice();
	int key = FsInkey();
	if(key == FSKEY_ESC)
	{
		return false;
	}

	const int scenarioKey[NUM_SCENARIO] = {FSKEY_1, FSKEY_2, FSKEY_3, FSKEY_4};
	for(int i = 0; i < NUM_SCENARIO; i++)
	{
		if(key == scenarioKey[i])
		{
			StartScenario(i);
		}
	}
	if(key == FSKEY_SPACE)
	{
		StartScenario(scenarioIdx);
	}
	if(key == FSKEY_T)
	{
		robotIdx = (robotIdx + 1) % NUM_ROBOT;
		StartScenario(scenarioIdx);
	}
	if(key == FSKEY_E || key == FSKEY_M || key == FSKEY_H)
	{
		SetLevel(key == FSKEY_E ? EASY : (key == FSKEY_M ? MEDIUM : HARD));
		StartScenario(scenarioIdx);
	}

	if(!finished)
	{
		Step();
	}
	return true;
}

void ScenarioViewer::Render(void) const
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	SetTableProjection(cfg);
	DrawTable(cfg);

	// Predicted puck path, then the opponent's own debug drawing
	glColor3ub(160, 160, 160);
	glBegin(GL_LINE_STRIP);
	for(const PuckSample &s : path.samples)
	{
		glVertex2d(s.pos.x, s.pos.y);
	}
	glEnd();
	opp.DrawDebug();

	robots[robotIdx]->Draw();
	glColor3ub(20, 20, 20);
	DrawCircle(puck.pos.x, puck.pos.y, puck.radius);
	glColor3ub(200, 60, 60);
	DrawCircle(human.pos.x, human.pos.y, human.radius);

	char str[256];
	glColor3ub(0, 0, 0);
	sprintf(str, "%s  %s  %s", opp.GetModeName(), ROBOT_NAME[robotIdx], LEVEL_NAME[level]);
	DrawText(0.0, -20.0, str);

	DrawText(8.0, cfg.height / 2.0 + 24.0, SCENARIOS[scenarioIdx].name);
	sprintf(str, "Expect: %s", SCENARIOS[scenarioIdx].expect);
	DrawText(8.0, cfg.height / 2.0 + 44.0, str);
	if(finished)
	{
		DrawText(8.0, cfg.height / 2.0 + 64.0, result);
	}

	DrawText(0.0, cfg.height + 22.0, "1-4 scenario  SPACE replay");
	DrawText(0.0, cfg.height + 38.0, "T robot  E/M/H level  ESC quit");

	FsSwapBuffers();
}

void RunWindow(const TableConfig &cfg)
{
	int winWid = (int)((cfg.width + 2.0 * VIEW_MARGIN) * VIEW_SCALE);
	int winHei = (int)((cfg.height + 2.0 * VIEW_MARGIN) * VIEW_SCALE);
	FsOpenWindow(0, 0, winWid, winHei, 1, "Opponent Logic Test");

	ScenarioViewer viewer(cfg);
	while(viewer.RunOneStep())
	{
		viewer.Render();
		FsSleep((int)(FRAME_TIME * 1000.0));
	}
}

int main(int argc, char *argv[])
{
	TableConfig cfg;
	RunChecks(cfg);
	if(argc >= 2 && strcmp(argv[1], "check") == 0)
	{
		return (nFail == 0 ? 0 : 1);
	}
	RunWindow(cfg);
	return 0;
}
