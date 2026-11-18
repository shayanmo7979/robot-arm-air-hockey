#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <chrono>

#include "table_puck.h"
#include "fssimplewindow.h"
#include "ysglfontdata.h"
#include "yssimplesound.h"


void Puck::Reset(Vec2 startPos, const TableConfig& cfg) {
	// Reset (Set position to table center)
	s.pos = startPos;
	s.vel = { 0, 0 };
	s.radius = cfg.puckRadius;
}

bool Puck::EdgeBounce(Vec2 edge, const TableConfig& cfg) {
	Vec2 d = s.pos - edge;
	double dist = Length(d);

	if (dist > s.radius || dist == 0) return false;

	else {
		Vec2 n = d / dist;
		s.pos = edge + n * s.radius;
		double vn = Dot(s.vel, n);

		if (vn < 0) {
			s.vel = s.vel - n * ((1 + cfg.wallRestitution) * vn);
			return true;
		}
	}

	return false;
}

PhysicsEvent Puck::WallBounce(const TableConfig& cfg) {

	double cent = cfg.width / 2;
	double goalLeft = cent - cfg.goalWidth / 2;
	double goalRight = cent + cfg.goalWidth / 2;
	double puckLeft = s.pos.x - s.radius;
	double puckRight = s.pos.x + s.radius;
	double puckTop = s.pos.y - s.radius;
	double puckBot = s.pos.y + s.radius;

	PhysicsEvent event = EV_NONE;

	// Goal check
	if (puckBot <= 0) return EV_GOAL_HUMAN;
	if (puckTop >= cfg.height) return EV_GOAL_ROBOT;

	// Goalpost collisions
	if (EdgeBounce({ goalLeft, 0 }, cfg) || EdgeBounce({ goalRight, 0 }, cfg) ||
		EdgeBounce({ goalLeft, cfg.height }, cfg) || EdgeBounce({ goalRight, cfg.height }, cfg))
	{
		event = EV_WALL_HIT;
	}

	// Left check
	double leftDist = s.pos.x;
	bool leftApproach = s.vel.x < 0;
	if (leftDist <= s.radius && leftApproach) {
		s.pos.x = s.radius;
		s.vel.x = -s.vel.x * cfg.wallRestitution;
		event = EV_WALL_HIT;
	}

	// Right check
	double rightDist = cfg.width - s.pos.x;
	bool rightApproach = s.vel.x > 0;
	if (rightDist <= s.radius && rightApproach) {
		s.pos.x = cfg.width - s.radius;
		s.vel.x = -s.vel.x * cfg.wallRestitution;
		event = EV_WALL_HIT;
	}

	// Top check
	double topDist = s.pos.y;
	bool topApproach = s.vel.y < 0;
	if (topDist <= s.radius && topApproach) {

		if (s.pos.x <= goalLeft || s.pos.x >= goalRight) {
			s.pos.y = s.radius;
			s.vel.y = -s.vel.y * cfg.wallRestitution;
			event = EV_WALL_HIT;
		}
	}

	// Bottom check
	double botDist = cfg.height - s.pos.y;
	bool botApproach = s.vel.y > 0;
	if (botDist <= s.radius && botApproach) {

		if (s.pos.x <= goalLeft || s.pos.x >= goalRight) {
			s.pos.y = cfg.height - s.radius;
			s.vel.y = -s.vel.y * cfg.wallRestitution;
			event = EV_WALL_HIT;
		}
	}

	return event;
}

PhysicsEvent Puck::Update(double dt, const TableConfig& cfg) {
	// 1. Friction slows the puck
	s.vel = s.vel * exp(-cfg.friction * dt);

	// 2. Cap the speed, stop at threshold
	if (Length(s.vel) > cfg.maxPuckSpeed) {
		s.vel = Normalize(s.vel) * cfg.maxPuckSpeed;
	}

	if (Length(s.vel) < cfg.restThreshold) s.vel = { 0, 0 };

	// 3. Move
	s.pos = s.pos + s.vel * dt;

	// 4. Check walls and goals return what happened
	return WallBounce(cfg);
}

bool Puck::CollideWithMallet(const MalletState& m, const TableConfig& cfg) {
	Vec2 d = s.pos - m.pos;
	double dist = Length(d);
	double minDist = s.radius + m.radius;

	// No collision
	if (dist >= minDist) return false;

	Vec2 n = (dist == 0) ? Vec2{ 0, 1 } : d * (1.0 / dist);   // from mallet to puck
	s.pos = m.pos + n * minDist;                             // push out of overlap
	double vn = Dot(s.vel - m.vel, n);                       // closing speed along the normal
	if (vn < 0) s.vel = s.vel - n * ((1 + cfg.malletRestitution) * vn);
	return true;
}

Vec2 Puck::PredictPosition(double t, const TableConfig& cfg) const {
	Puck sim = *this;
	double substep = 1.0 / 200;
	for (double el = 0.0; el < t; el += substep) {
		sim.Update(substep, cfg, false);
	}

	return sim.s.pos;
}

PredictResult Puck::PredictLineCrossing(double lineY, double maxTime, const TableConfig& cfg) const {
	Puck sim = *this;
	double substep = 1.0 / 200;
	double time = 0.0;

	PredictResult p;
	p.found = false;
	p.point = { 0.0, 0.0 };
	p.time = 0.0;

	double prevY = 0.0;
	double curY = 0.0;
	Vec2 prev = { 0.0, 0.0 };

	for (double el = 0.0; el < maxTime; el += substep) {

		prevY = sim.s.pos.y;
		prev = sim.s.pos;

		sim.Update(substep, cfg, false);
		curY = sim.s.pos.y;
		
		// lineY was crossed in either direction by the puck
		if ((prevY - lineY) * (curY - lineY) <= 0 && prevY != curY) {
			p.found = true;
			double f = (lineY - prevY) / (curY - prevY);
			p.point = prev + (sim.s.pos - prev) * f;
			p.time = el + substep * f;
			break;
		}
	}

	return p;
}

void Puck::Draw() const {
	glColor3ub(0, 0, 0);
	DrawCircle(TABLE_OX + s.pos.x, TABLE_OY + s.pos.y, s.radius);
}

Table::Table() {
	ox = TABLE_OX;
	oy = TABLE_OY;
}

const TableConfig& Table::GetConfig() const { return cfg; }

double Table::GoalLeft() const { return cfg.width / 2 - cfg.goalWidth / 2 + ox; }

double Table::GoalRight() const { return cfg.width / 2 + cfg.goalWidth / 2 + oy; }

void Table::Draw() const {
	double ox = TABLE_OX;
	double oy = TABLE_OY;

	glColor3ub(0, 0, 0);
	glLineWidth(8.0f);
	glBegin(GL_LINES);

	double w = cfg.width;
	double h = cfg.height;
	double gl = GoalLeft() + ox;
	double gr = GoalRight() + ox;

	// Top
	glVertex2d(ox, oy);
	glVertex2d(gl, oy);
	glVertex2d(gr, oy);
	glVertex2d(ox + w, oy);

	// Bottom
	glVertex2d(ox, oy + h);
	glVertex2d(gl, oy + h);
	glVertex2d(gr, oy + h);
	glVertex2d(ox + w, oy + h);

	// Right
	glVertex2d(ox + w, oy);
	glVertex2d(ox + w, oy + h);

	// Left
	glVertex2d(ox, oy);
	glVertex2d(ox, oy + h);

	glEnd();

	// Middle Line
	glColor3ub(150, 150, 150);
	glLineWidth(4.0f);
	glBegin(GL_LINES);
	glVertex2d(ox, oy + h / 2);
	glVertex2d(ox + w, oy + h / 2);
	glEnd();
}


// Human Mallet Functions 
void HumanMallet::Reset(const TableConfig& cfg) {
	s.radius = cfg.malletRadius;
	s.pos = { cfg.width / 2, cfg.height * 0.85 };
	s.vel = { 0,0 };
}

void HumanMallet::Update(double dt, int mouseX, int mouseY, const TableConfig& cfg) {

	// Convert mouse to table coordinates
	double mx = mouseX - TABLE_OX;
	double my = mouseY - TABLE_OY;

	// Clamp to human side
	if (mx < s.radius) mx = s.radius;
	if (mx > cfg.width - s.radius) mx = cfg.width - s.radius;
	if (my < cfg.height / 2 + s.radius) my = cfg.height / 2 + s.radius;
	if (my > cfg.height - s.radius) my = cfg.height - s.radius;

	// Movement step
	Vec2 target = { mx, my };
	Vec2 prev = s.pos;
	Vec2 step = target - prev;

	// Clamp movement speed
	double maxStep = cfg.maxHumanSpeed * dt;
	if (Length(step) > maxStep) {
		step = Normalize(step) * maxStep;
	}

	s.pos = s.pos + step;
	if (dt > 0) s.vel = step / dt;


}

void HumanMallet::Draw() const {
	double cx = s.pos.x + TABLE_OX;
	double cy = s.pos.y + TABLE_OY;

	glColor3ub(0, 0, 255);
	DrawCircle(cx, cy, s.radius);

	glColor3ub(0, 0, 125);
	DrawCircle(cx, cy, s.radius * 0.85);

	glColor3ub(0, 0, 255);
	DrawCircle(cx, cy, s.radius * 0.35);
}