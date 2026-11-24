#include <stdio.h>
#include <string.h>
#include "fssimplewindow.h"
#include "ysglfontdata.h"
#include "table_puck.h"

// White box with centered text in the middle of the window
void DrawMessage(const char* msg, int winW, int winH) {
	int cx = winW / 2, cy = winH / 2;
	int textW = (int)strlen(msg) * 20;          // 20 px per character for the 20x32 font

	glColor3ub(255, 255, 255);
	DrawRect(cx - 160, cy - 50, cx + 160, cy + 50, true);

	glColor3ub(0, 0, 0);
	DrawRect(cx - 160, cy - 50, cx + 160, cy + 50, false);   // thin border

	glRasterPos2i(cx - textW / 2, cy + 16);     // y is the text baseline
	YsGlDrawFontBitmap20x32(msg);
}

void DrawScore(int humanScore, int robotScore, int winW, int winH) {
	char buf[64];
	sprintf(buf, "YOU %d  -  %d ENEMY", humanScore, robotScore);

	int textW = (int)strlen(buf) * 20;
	glColor3ub(0, 0, 0);
	glRasterPos2i((winW - textW) / 2, winH - 25);   // baseline near the bottom edge
	YsGlDrawFontBitmap20x32(buf);
}

int main(void) {
	const int winW = 500, winH = 950;
	int humanScore = 0, robotScore = 0;   // dummy scores
	FsOpenWindow(0, 0, winW, winH, 1);

	Table table;
	const TableConfig& cfg = table.GetConfig();

	Puck puck;
	HumanMallet human;
	puck.Reset({ cfg.width / 2, cfg.height / 2 }, cfg);
	human.Reset(cfg);

	// Fake robot mallet
	MalletState robot;
	robot.pos = { cfg.width / 2, cfg.height * 0.15 };
	robot.vel = { 0, 0 };
	robot.radius = cfg.malletRadius;

	const char* message = "";       // text to show during a goal pause
	double messageTimer = 0;        // seconds left in the goal pause

	FsPassedTime();                 // start the clock

	for (;;) {
		FsPollDevice();
		double dt = FsPassedTime() / 2000.0;
		if (dt > 0.05) dt = 0.05;

		int key = FsInkey();
		if (key == FSKEY_ESC) break;
		if (key == FSKEY_SPACE) puck.Reset({ cfg.width / 2, cfg.height / 2 }, cfg);

		int lb, mb, rb, mx, my;
		FsGetMouseState(lb, mb, rb, mx, my);

		if (messageTimer > 0) {
			// Goal pause: physics frozen, count down, then reset the puck
			messageTimer -= dt;
			if (messageTimer <= 0) {
				puck.Reset({ cfg.width / 2, cfg.height / 2 }, cfg);
			}
		}
		else {
			human.Update(dt, mx, my, cfg);
			PhysicsEvent e = puck.Update(dt, cfg);
			if (puck.CollideWithMallet(human.s, cfg)) e = EV_MALLET_HIT;
			if (puck.CollideWithMallet(robot, cfg))            e = EV_MALLET_HIT;

			if (e != EV_NONE) {
				const char* names[] = { "NONE", "WALL_HIT", "MALLET_HIT", "GOAL_HUMAN", "GOAL_ROBOT" };
				printf("event: %s\n", names[e]);
			}
			if (e == EV_GOAL_HUMAN) { humanScore++; message = "GOAL!";       messageTimer = 1.5; }
			if (e == EV_GOAL_ROBOT) { robotScore++; message = "ENEMY GOAL!"; messageTimer = 1.5; }
		}

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		table.Draw();
		puck.Draw();
		human.Draw();
		glColor3ub(255, 0, 0);
		DrawCircle(TABLE_OX + robot.pos.x, TABLE_OY + robot.pos.y, robot.radius);
		glColor3ub(125, 0, 0);
		DrawCircle(TABLE_OX + robot.pos.x, TABLE_OY + robot.pos.y, robot.radius * 0.85);
		glColor3ub(255, 0, 0);
		DrawCircle(TABLE_OX + robot.pos.x, TABLE_OY + robot.pos.y, robot.radius * 0.35);

		DrawScore(humanScore, robotScore, winW, winH);

		if (messageTimer > 0) DrawMessage(message, winW, winH);

		FsSwapBuffers();
		FsSleep(5);
	}
	return 0;
}