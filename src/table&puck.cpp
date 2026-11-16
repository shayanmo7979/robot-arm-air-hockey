#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <chrono>

#include "shared_types.h"
#include "fssimplewindow.h"
#include "ysglfontdata.h"
#include "yssimplesound.h"


class Puck {
public:	

	PuckState s;

	void Reset(Vec2 startPos) {
		// Reset (Set position to table center)
		s.pos = startPos;
		s.vel = { 0, 0 };
	}

	bool EdgeBounce(Vec2 edge, const TableConfig& cfg) {
		Vec2 d = s.pos - edge;
		double dist = Length(d);

		if (dist > s.radius || dist == 0) { return false; }

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

	

	PhysicsEvent WallBounce(const TableConfig& cfg) {

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
			return EV_WALL_HIT
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
	
	PhysicsEvent Update(double dt, const TableConfig &cfg) {
		// 1. Friction slows the puck
		s.vel = s.vel * fmax(0.0, 1 - cfg.friction * dt);

		// 2. Cap the speed
		if (Length(s.vel) > cfg.maxPuckSpeed) {
			s.vel = Normalize(s.vel) * cfg.maxPuckSpeed;
		}

		// 3. Move
		s.pos = s.pos + s.vel * dt;

		// 4. Check walls and goals return what happened
		
		// RETURN HandleWalls(cfg)

		return EV_NONE;
	}

	void Draw() const {
		// Draw the puck as a circle at its current position
		glColor3ub(0, 0, 0);
		DrawCircle(s.pos.x, s.pos.y, s.radius);
	}
};

