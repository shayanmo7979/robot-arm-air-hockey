#include <stdio.h>
#include <string.h>

#include "fssimplewindow.h"
#include "ysglfontdata.h"
#include "title.h"

const int ITEM_WIDTH = 360;
const int ITEM_HEIGHT = 56;
const int ITEM_GAP = 18;
const int MENU_TOP = 300;

static void DrawCenteredText(const char *str, int cx, int baseY, int charWid, void (*drawFunc)(const char[]))
{
	int textWid = (int)strlen(str) * charWid;
	glRasterPos2i(cx - textWid / 2, baseY);
	drawFunc(str);
}

void TitleScreen::Initialize(void)
{
	endState = ENDSTATE_NONE;
	cursor = ITEM_START;
}

const char *TitleScreen::RobotName(RobotChoice choice)
{
	switch (choice)
	{
	case ROBOT_ARM:
		return "Robot Arm";
	case ROBOT_MOBILE:
		return "Mobile Robot";
	}
	return "?";
}

const char *TitleScreen::DifficultyName(Difficulty level)
{
	switch (level)
	{
	case EASY:
		return "Easy";
	case MEDIUM:
		return "Medium";
	case HARD:
		return "Hard";
	}
	return "?";
}

void TitleScreen::Activate(int item, int dir)
{
	switch (item)
	{
	case ITEM_ROBOT:
		robotChoice = (ROBOT_ARM == robotChoice ? ROBOT_MOBILE : ROBOT_ARM);
		break;
	case ITEM_DIFFICULTY:
	{
		int level = ((int)difficulty + dir + 3) % 3;
		difficulty = (Difficulty)level;
		break;
	}
	case ITEM_START:
		endState = ENDSTATE_START_GAME;
		break;
	case ITEM_QUIT:
		endState = ENDSTATE_QUIT;
		break;
	}
}

void TitleScreen::GetItemBox(int item, int &x1, int &y1, int &x2, int &y2) const
{
	int wid, hei;
	FsGetWindowSize(wid, hei);
	x1 = (wid - ITEM_WIDTH) / 2;
	x2 = x1 + ITEM_WIDTH;
	y1 = MENU_TOP + item * (ITEM_HEIGHT + ITEM_GAP);
	y2 = y1 + ITEM_HEIGHT;
}

int TitleScreen::ItemAt(int mx, int my) const
{
	for (int item = 0; item < NUM_ITEMS; ++item)
	{
		int x1, y1, x2, y2;
		GetItemBox(item, x1, y1, x2, y2);
		if (x1 <= mx && mx <= x2 && y1 <= my && my <= y2)
		{
			return item;
		}
	}
	return -1;
}

void TitleScreen::RunOneStep(void)
{
	FsPollDevice();
	auto key = FsInkey();

	if (FSKEY_ESC == key || FSKEY_Q == key)
	{
		endState = ENDSTATE_QUIT;
	}
	else if (FSKEY_S == key)
	{
		endState = ENDSTATE_START_GAME;
	}
	else if (FSKEY_UP == key)
	{
		cursor = (cursor + NUM_ITEMS - 1) % NUM_ITEMS;
	}
	else if (FSKEY_DOWN == key)
	{
		cursor = (cursor + 1) % NUM_ITEMS;
	}
	else if (FSKEY_LEFT == key && cursor <= ITEM_DIFFICULTY)
	{
		Activate(cursor, -1);
	}
	else if (FSKEY_RIGHT == key && cursor <= ITEM_DIFFICULTY)
	{
		Activate(cursor, +1);
	}
	else if (FSKEY_ENTER == key || FSKEY_SPACE == key)
	{
		Activate(cursor, +1);
	}

	int lb, mb, rb, mx, my;
	for (;;)
	{
		int evt = FsGetMouseEvent(lb, mb, rb, mx, my);
		if (FSMOUSEEVENT_NONE == evt)
		{
			break;
		}
		int item = ItemAt(mx, my);
		if (0 <= item)
		{
			cursor = item;   // hover highlights
			if (FSMOUSEEVENT_LBUTTONDOWN == evt)
			{
				Activate(item, +1);
			}
		}
	}
}

void TitleScreen::Render(void) const
{
	int wid, hei;
	FsGetWindowSize(wid, hei);
	int cx = wid / 2;

	glClearColor(0.12f, 0.14f, 0.20f, 1.0f);  
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	// Title
	glColor3ub(255, 255, 255);
	DrawCenteredText("AIR HOCKEY", cx, 150, 32, YsGlDrawFontBitmap32x48);
	glColor3ub(170, 190, 220);
	DrawCenteredText("Human vs Robot", cx, 200, 16, YsGlDrawFontBitmap16x20);

	for (int item = 0; item < NUM_ITEMS; ++item)
	{
		int x1, y1, x2, y2;
		GetItemBox(item, x1, y1, x2, y2);

		if (item == cursor)
		{
			glColor3ub(230, 80, 60);
			DrawRect(x1, y1, x2, y2, true);
		}
		glColor3ub(255, 255, 255);
		glLineWidth(2);
		DrawRect(x1, y1, x2, y2, false);
		glLineWidth(1);

		char label[64];
		switch (item)
		{
		case ITEM_ROBOT:
			snprintf(label, sizeof(label), "< %s >", RobotName(robotChoice));
			break;
		case ITEM_DIFFICULTY:
			snprintf(label, sizeof(label), "< %s >", DifficultyName(difficulty));
			break;
		case ITEM_START:
			snprintf(label, sizeof(label), "START");
			break;
		case ITEM_QUIT:
			snprintf(label, sizeof(label), "QUIT");
			break;
		}

		if (ITEM_ROBOT == item || ITEM_DIFFICULTY == item)
		{
			glColor3ub(170, 190, 220);
			glRasterPos2i(x1, y1 - 4);
			YsGlDrawFontBitmap8x12(ITEM_ROBOT == item ? "OPPONENT" : "DIFFICULTY");
		}

		glColor3ub(255, 255, 255);
		DrawCenteredText(label, cx, y1 + ITEM_HEIGHT / 2 + 10, 16, YsGlDrawFontBitmap16x20);
	}

	glColor3ub(170, 190, 220);
	DrawCenteredText("UP/DOWN: move   LEFT/RIGHT: change", cx, hei - 60, 8, YsGlDrawFontBitmap8x12);
	DrawCenteredText("ENTER: select   S: start   ESC: quit", cx, hei - 40, 8, YsGlDrawFontBitmap8x12);
	DrawCenteredText("Mouse: click to change or select", cx, hei - 20, 8, YsGlDrawFontBitmap8x12);

	FsSwapBuffers();
}
