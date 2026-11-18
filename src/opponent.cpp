// Opponent Logic implementation.  See opponent.h for the overview of the four modes.
// Coordinates (from shared.h): pixels, origin top-left, y down.  Robot goal at the top (y = 0),
// human goal at the bottom (y = cfg.height).  So a puck coming at the robot has vel.y < 0.

#include <math.h>

#include "fssimplewindow.h"
#include "opponent.h"

// Tuning constants, in pixels and seconds
const double DEFENSE_LINE_Y = 80.0;      // defense line, measured from the robot's end wall
const double GUARD_SHIFT = 0.4;          // how far GUARD follows the puck sideways (0 = stay centered)
const double WINDUP = 50.0;              // gap between the wind-up point and the contact point
const double STRIKE_SPEED = 800.0;       // paddle speed requested for a strike (px/s)
const double WINDUP_TOLERANCE = 12.0;    // how close to the wind-up point a strike may start
const double MIN_LEAD = 0.08;            // never plan a hit sooner than this (s)
const double HUMAN_MARGIN = 20.0;        // shot paths must clear the human mallet by this much
const double POST_MARGIN = 2.0;          // shots must clear the goal posts by this much
const double MAX_RECOVER_TIME = 1.5;     // stop waiting for the robot after this long (s)
const double UNREACHABLE = 1e9;          // reach time for points the robot can't get to
const int PLAN_STRIDE = 3;               // check every 3rd predicted sample as a hit point
const int NUM_GOAL_POINT = 5;            // aim points across the human goal

// Pucks slower than this get attacked, per difficulty (px/s)
const double ATTACK_SPEED[3] = {200.0, 350.0, 550.0};

// Side walls, for bank shots
const int WALL_LEFT = -1;
const int WALL_RIGHT = 1;



////////////////////////////////////////////////////////////
// Helpers

// Shortest distance from point p to the line segment a-b.
static double DistancePointSegment(Vec2 p, Vec2 a, Vec2 b)
{
	Vec2 ab = b - a;
	double len2 = Dot(ab, ab);
	if(len2 == 0.0)
	{
		return Length(p - a);
	}
	double t = Clamp(Dot(p - a, ab) / len2, 0.0, 1.0);   // closest point is a + ab * t
	return Length(p - (a + ab * t));
}

static void DrawCircleOutline(Vec2 c, double r)
{
	const int nDiv = 32;
	glBegin(GL_LINE_LOOP);
	for(int i = 0; i < nDiv; i++)
	{
		double a = 2.0 * PI * (double)i / (double)nDiv;
		glVertex2d(c.x + r * cos(a), c.y + r * sin(a));
	}
	glEnd();
}

// Draws connected line segments through the points.
static void DrawPath(const std::vector<Vec2> &path)
{
	glBegin(GL_LINE_STRIP);
	for(const Vec2 &p : path)
	{
		glVertex2d(p.x, p.y);
	}
	glEnd();
}

// Robot::TimeToReach with "unreachable" mapped to one large number.  A robot that has not
// implemented TimeToReach yet (returns a negative value) is treated as unable to reach anything.
static double ReachTime(const Robot &robot, Vec2 p)
{
	double t = robot.TimeToReach(p);
	if(t < 0.0)
	{
		return UNREACHABLE;
	}
	return t;
}



////////////////////////////////////////////////////////////
// Puck prediction (stand-in for the physics component)

// Pushes the puck out of a goal post and reflects it, like Puck::EdgeBounce in table_puck.cpp.
static void BouncePost(PuckState &puck, Vec2 post, const TableConfig &cfg)
{
	Vec2 d = puck.pos - post;
	double dist = Length(d);
	if(dist > puck.radius || dist == 0.0)
	{
		return;
	}
	Vec2 n = d / dist;                  // unit vector from the post to the puck
	puck.pos = post + n * puck.radius;
	double vn = Dot(puck.vel, n);
	if(vn < 0.0)                        // moving into the post
	{
		puck.vel = puck.vel - n * ((1.0 + cfg.wallRestitution) * vn);
	}
}

// Same rules as Puck::Update in table_puck.cpp, so the prediction matches the real puck.
void StepFreePuck(PuckState &puck, const TableConfig &cfg, double dt)
{
	// Friction, speed cap, stop when very slow, then move
	puck.vel = puck.vel * fmax(0.0, 1.0 - cfg.friction * dt);
	if(Length(puck.vel) > cfg.maxPuckSpeed)
	{
		puck.vel = Normalize(puck.vel) * cfg.maxPuckSpeed;
	}
	if(Length(puck.vel) < cfg.restThreshold)
	{
		puck.vel = {0.0, 0.0};
	}
	puck.pos = puck.pos + puck.vel * dt;

	double r = puck.radius;
	double goalLeft = cfg.width / 2.0 - cfg.goalWidth / 2.0;
	double goalRight = cfg.width / 2.0 + cfg.goalWidth / 2.0;
	bool inMouth = (goalLeft < puck.pos.x && puck.pos.x < goalRight);   // lined up with a goal opening

	BouncePost(puck, {goalLeft, 0.0}, cfg);
	BouncePost(puck, {goalRight, 0.0}, cfg);
	BouncePost(puck, {goalLeft, cfg.height}, cfg);
	BouncePost(puck, {goalRight, cfg.height}, cfg);

	// Side walls
	if(puck.pos.x <= r && puck.vel.x < 0.0)
	{
		puck.pos.x = r;
		puck.vel.x = -puck.vel.x * cfg.wallRestitution;
	}
	if(puck.pos.x >= cfg.width - r && puck.vel.x > 0.0)
	{
		puck.pos.x = cfg.width - r;
		puck.vel.x = -puck.vel.x * cfg.wallRestitution;
	}

	// End walls, except where the goal opening is
	if(!inMouth && puck.pos.y <= r && puck.vel.y < 0.0)
	{
		puck.pos.y = r;
		puck.vel.y = -puck.vel.y * cfg.wallRestitution;
	}
	if(!inMouth && puck.pos.y >= cfg.height - r && puck.vel.y > 0.0)
	{
		puck.pos.y = cfg.height - r;
		puck.vel.y = -puck.vel.y * cfg.wallRestitution;
	}
}

PuckPath PredictPuckPath(PuckState puck, const TableConfig &cfg, double horizon, double dt)
{
	PuckPath path;
	double t = 0.0;
	while(true)
	{
		path.samples.push_back({t, puck.pos, puck.vel});   // sample 0 is the puck right now
		bool inGoal = (puck.pos.y < -puck.radius || puck.pos.y > cfg.height + puck.radius);
		if(t >= horizon || inGoal)
		{
			break;
		}
		StepFreePuck(puck, cfg, dt);
		t = t + dt;
	}
	return path;
}



////////////////////////////////////////////////////////////
// OpponentLogic

OpponentLogic::OpponentLogic(const TableConfig &cfg) : cfg(cfg)
{
	Reset();
}

void OpponentLogic::SetDifficulty(Difficulty level)
{
	// Harder robots treat faster pucks as attackable, so they attack more often.
	attackSpeed = ATTACK_SPEED[level];
}

void OpponentLogic::Reset(void)
{
	mode = GUARD;
	clock = 0.0;
	malletHit = false;
	hitTime = 0.0;
	candidates.clear();
	lastTarget = {cfg.width / 2.0, DEFENSE_LINE_Y};
}

void OpponentLogic::OnPhysicsEvent(PhysicsEvent ev)
{
	if(ev == EV_MALLET_HIT)
	{
		malletHit = true;   // used (and cleared) by the next Update
	}
}

void OpponentLogic::Update(Robot &robot, PuckState puck, MalletState human, const PuckPath &path, double dt)
{
	clock = clock + dt;

	// 1. An attack in progress keeps going until it strikes or is given up.
	if(mode == ATTACK)
	{
		ContinueAttack(robot);
		malletHit = false;
		if(mode == ATTACK || mode == RECOVER)
		{
			return;
		}
	}
	malletHit = false;

	// 2. Otherwise pick the mode from the puck's speed and direction.
	double speed = Length(puck.vel);
	bool incoming = (puck.vel.y < 0.0);

	if(incoming && speed >= attackSpeed)
	{
		// A fast shot also interrupts RECOVER: the robot has to block it instead of going home.
		mode = DEFEND;
		MoveTo(robot, DefendTarget(robot, puck, path));
	}
	else if(mode == RECOVER && !robot.IsCommandDone() && clock < hitTime + MAX_RECOVER_TIME)
	{
		// The robot is returning home by itself; send nothing.
	}
	else if(speed < attackSpeed && PlanAttack(robot, human, path))
	{
		mode = ATTACK;
		ContinueAttack(robot);
	}
	else
	{
		mode = GUARD;
		MoveTo(robot, GuardTarget(puck));
	}
}

OpponentLogic::Mode OpponentLogic::GetMode(void) const
{
	return mode;
}

const char *OpponentLogic::GetModeName(void) const
{
	switch(mode)
	{
	case GUARD:
		return "GUARD";
	case DEFEND:
		return "DEFEND";
	case ATTACK:
		return "ATTACK";
	case RECOVER:
		return "RECOVER";
	}
	return "?";
}

double OpponentLogic::GetAttackSpeed(void) const
{
	return attackSpeed;
}

double OpponentLogic::GetDefenseY(void) const
{
	return DEFENSE_LINE_Y;
}

Vec2 OpponentLogic::GetLastTarget(void) const
{
	return lastTarget;
}

// Every move command goes through here so DrawDebug can show the current target.
void OpponentLogic::MoveTo(Robot &robot, Vec2 target)
{
	lastTarget = target;
	robot.CommandMoveTo(target);
}

// Distance between the paddle center and the puck center when they touch.
double OpponentLogic::ContactDistance(const Robot &robot) const
{
	return robot.GetPaddleState().radius + cfg.puckRadius;
}

// Stay on the defense line, shifted toward the puck but never past the goal posts.
Vec2 OpponentLogic::GuardTarget(PuckState puck) const
{
	double center = cfg.width / 2.0;
	double half = cfg.goalWidth / 2.0;
	double x = center + Clamp((puck.pos.x - center) * GUARD_SHIFT, -half, half);
	return {x, DEFENSE_LINE_Y};
}

// Where the predicted path first crosses the defense line, if the robot gets there in time.
// Otherwise the earliest later point (between the line and the goal) it can still reach first.
Vec2 OpponentLogic::DefendTarget(const Robot &robot, PuckState puck, const PuckPath &path) const
{
	// Keep the paddle fully on the table
	const double minY = robot.GetPaddleState().radius;
	const double minX = robot.GetPaddleState().radius;
	const double maxX = cfg.width - robot.GetPaddleState().radius;

	// First predicted sample past the defense line
	int first = -1;
	for(int i = 0; i < (int)path.samples.size(); i++)
	{
		if(path.samples[i].pos.y <= DEFENSE_LINE_Y)
		{
			first = i;
			break;
		}
	}
	if(first < 0)
	{
		return GuardTarget(puck);   // puck stops (or turns) before the line
	}

	// From there on, the first point the robot reaches before the puck does
	for(int i = first; i < (int)path.samples.size(); i++)
	{
		const PuckSample &s = path.samples[i];
		if(s.pos.y < minY)
		{
			break;   // the puck is in the goal mouth now; too late
		}
		Vec2 p = {Clamp(s.pos.x, minX, maxX), s.pos.y};
		if(ReachTime(robot, p) <= s.t)
		{
			return p;
		}
	}

	// Can't make it anywhere: do the best we can at the first crossing.
	const PuckSample &s = path.samples[first];
	return {Clamp(s.pos.x, minX, maxX), DEFENSE_LINE_Y};
}

// Puck positions the robot may plan to hit: on its own half, off the side walls and its end wall.
bool OpponentLogic::InStrikeZone(Vec2 p) const
{
	double r = cfg.puckRadius;
	return (r + 2.0 < p.x && p.x < cfg.width - r - 2.0 &&
	        r + 30.0 < p.y && p.y < cfg.height / 2.0 - 10.0);
}

// The earliest predicted puck position where some open shot is reachable in time wins;
// among its shots, the one needing the least robot movement.
bool OpponentLogic::PlanAttack(const Robot &robot, MalletState human, const PuckPath &path)
{
	const double contact = ContactDistance(robot);
	const double strikeTime = WINDUP / STRIKE_SPEED;   // wind-up point -> contact during the strike

	for(int i = 0; i < (int)path.samples.size(); i = i + PLAN_STRIDE)
	{
		const PuckSample &s = path.samples[i];
		if(s.t < MIN_LEAD || !InStrikeZone(s.pos))
		{
			continue;
		}

		std::vector<Shot> shots = GenerateShots(s.pos, human);
		int best = -1;
		for(int k = 0; k < (int)shots.size(); k++)
		{
			// The wind-up point is behind the puck on the shot line, so the strike pushes the
			// puck along shot.dir.
			Shot &shot = shots[k];
			Vec2 windup = s.pos - shot.dir * (contact + WINDUP);
			shot.cost = ReachTime(robot, windup);
			shot.reachable = (shot.cost + strikeTime <= s.t);
			if(shot.open && shot.reachable && (best < 0 || shot.cost < shots[best].cost))
			{
				best = k;
			}
		}

		if(best >= 0)
		{
			hitPoint = s.pos;
			hitTime = clock + s.t;
			plannedShot = shots[best];
			candidates = shots;
			return true;
		}
	}
	return false;
}

// Moves to the wind-up point, then sends the strike just in time to meet the puck.
void OpponentLogic::ContinueAttack(Robot &robot)
{
	if(malletHit)
	{
		mode = GUARD;   // the puck was hit before our strike: the plan is stale
		return;
	}

	const double contact = ContactDistance(robot);
	const double strikeTime = WINDUP / STRIKE_SPEED;
	const Vec2 dir = plannedShot.dir;
	Vec2 windup = hitPoint - dir * (contact + WINDUP);
	double timeLeft = hitTime - clock;   // until the puck reaches hitPoint
	bool inPosition = (Length(robot.GetPaddleState().pos - windup) <= WINDUP_TOLERANCE);

	if(inPosition && timeLeft <= strikeTime)
	{
		robot.CommandStrike(hitPoint, dir, STRIKE_SPEED);
		lastTarget = hitPoint;
		mode = RECOVER;
	}
	else if(!inPosition && timeLeft < strikeTime)
	{
		mode = GUARD;   // too late; a strike from here would hit at the wrong angle, so replan
	}
	else
	{
		MoveTo(robot, windup);
	}
}



////////////////////////////////////////////////////////////
// Shot generation

// Follows a straight puck path, bouncing off the side walls, until it reaches the human goal line.
// Returns false if it would need more than maxBounce bounces.  walls lists the walls it hit, in order.
bool OpponentLogic::TraceShot(Vec2 from, Vec2 dir, int maxBounce, std::vector<Vec2> &path, std::vector<int> &walls) const
{
	const double wallL = cfg.puckRadius;               // side walls as seen by the puck center
	const double wallR = cfg.width - cfg.puckRadius;

	path.clear();
	walls.clear();
	if(dir.y <= 0.0)
	{
		return false;   // not heading toward the human goal
	}

	Vec2 pos = from;
	path.push_back(pos);
	while(true)
	{
		// Time (in units of dir) until the goal line and until the next side wall
		double tGoal = (cfg.height - pos.y) / dir.y;
		double tWall = UNREACHABLE;
		int wall = 0;
		if(dir.x < 0.0)
		{
			tWall = (wallL - pos.x) / dir.x;
			wall = WALL_LEFT;
		}
		else if(dir.x > 0.0)
		{
			tWall = (wallR - pos.x) / dir.x;
			wall = WALL_RIGHT;
		}

		if(tGoal <= tWall)
		{
			path.push_back(pos + dir * tGoal);
			return true;
		}
		if((int)walls.size() == maxBounce)
		{
			return false;
		}
		pos = pos + dir * tWall;
		path.push_back(pos);
		walls.push_back(wall);
		dir.x = -dir.x * cfg.wallRestitution;   // the wall takes some of the sideways speed
	}
}

// The traced path ends on the goal line; the puck still has to fit through the mouth at that
// angle without catching a post or the end wall beside it.
bool OpponentLogic::EntersGoalCleanly(const std::vector<Vec2> &path) const
{
	const double r = cfg.puckRadius;
	const double center = cfg.width / 2.0;
	const double half = cfg.goalWidth / 2.0;

	Vec2 a = path[path.size() - 2];              // last segment of the shot, a -> b
	Vec2 b = path.back();
	Vec2 d = Normalize(b - a);
	Vec2 end = b + d * (2.0 * r / fabs(d.y));    // continue until the puck is fully in the goal
	Vec2 atWall = b - d * (r / fabs(d.y));       // where the puck's edge reaches the end wall

	Vec2 postL = {center - half, cfg.height};
	Vec2 postR = {center + half, cfg.height};
	return (fabs(atWall.x - center) < half - POST_MARGIN &&
	        DistancePointSegment(postL, a, end) > r + POST_MARGIN &&
	        DistancePointSegment(postR, a, end) > r + POST_MARGIN);
}

std::vector<Shot> OpponentLogic::GenerateShots(Vec2 hitPoint, MalletState human) const
{
	// The 5 bounce routes, walls listed in the order the puck hits them
	const std::vector<std::vector<int>> sequences = {
		{},                        // straight
		{WALL_LEFT},               // off the left wall
		{WALL_RIGHT},              // off the right wall
		{WALL_LEFT, WALL_RIGHT},   // left, then right
		{WALL_RIGHT, WALL_LEFT},   // right, then left
	};
	const double wallL = cfg.puckRadius;
	const double wallR = cfg.width - cfg.puckRadius;
	const double spread = cfg.goalWidth / 2.0 - cfg.puckRadius - 6.0;           // aim points stay inside the posts
	const double clearance = human.radius + cfg.puckRadius + HUMAN_MARGIN;      // puck-to-mallet distance for "open"

	std::vector<Shot> shots;
	for(const std::vector<int> &seq : sequences)
	{
		for(int k = 0; k < NUM_GOAL_POINT; k++)
		{
			// Aim points evenly spaced from -spread to +spread around the goal center
			Shot shot;
			double offset = spread * (double)(k - NUM_GOAL_POINT / 2) / (double)(NUM_GOAL_POINT / 2);
			shot.goalPoint = {cfg.width / 2.0 + offset, cfg.height};
			shot.nBounce = (int)seq.size();

			// Aim at the goal point's mirror image: reflect it across the walls, last wall first.
			// A bounce keeps only wallRestitution of the sideways speed, so after the bounce the
			// puck covers less sideways distance; stretching the image by 1/restitution makes up
			// for it.
			Vec2 image = shot.goalPoint;
			for(int i = (int)seq.size() - 1; i >= 0; i--)
			{
				double wallX = (seq[i] == WALL_LEFT ? wallL : wallR);
				image.x = wallX + (wallX - image.x) / cfg.wallRestitution;
			}
			shot.dir = Normalize(image - hitPoint);

			// Trace it to check the puck really takes this route into the goal
			std::vector<int> walls;
			if(!TraceShot(hitPoint, shot.dir, shot.nBounce, shot.path, walls) || walls != seq ||
			   !EntersGoalCleanly(shot.path))
			{
				continue;   // impossible from here (hits the other wall first, or clips a post)
			}

			// Open if every segment of the path stays clear of the human mallet
			shot.open = true;
			for(int i = 0; i + 1 < (int)shot.path.size(); i++)
			{
				if(DistancePointSegment(human.pos, shot.path[i], shot.path[i + 1]) < clearance)
				{
					shot.open = false;
					break;
				}
			}
			shots.push_back(shot);
		}
	}
	return shots;
}



////////////////////////////////////////////////////////////
// Debug drawing

void OpponentLogic::DrawDebug(void) const
{
	// Defense line
	glColor3ub(200, 60, 60);
	glBegin(GL_LINES);
	glVertex2d(0.0, DEFENSE_LINE_Y);
	glVertex2d(cfg.width, DEFENSE_LINE_Y);
	glEnd();

	// Current target
	glColor3ub(230, 160, 0);
	DrawCircleOutline(lastTarget, 8.0);

	if(mode != ATTACK)
	{
		return;
	}

	// Candidate shots: green = open and reachable, pink = not usable
	for(const Shot &s : candidates)
	{
		if(s.open && s.reachable)
		{
			glColor3ub(140, 210, 140);
		}
		else
		{
			glColor3ub(230, 170, 170);
		}
		DrawPath(s.path);
	}

	// Chosen shot (thick) and where the puck will be hit
	glColor3ub(0, 150, 0);
	glLineWidth(3);
	DrawPath(plannedShot.path);
	glLineWidth(1);
	DrawCircleOutline(hitPoint, cfg.puckRadius);
}
