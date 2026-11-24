#include <stdio.h>
#include <thread>
#include <chrono>

#include "fssimplewindow.h"
#include "ysglfontdata.h"
#include "shared.h"
#include "robot.h"
#include "robotarm.h"
#include "mobile_robot.h"
#include "opponent.h"
#include "table_puck.h"
#include "title.h"

const int WINDOW_WID = 500;
const int WINDOW_HEI = 950;

class GameScreen
{
public:
	Physics physics;        
	RobotArm arm;
	MobileRobot mobile;
	OpponentLogic opponent;

	Robot *robot = &arm;      
	TitleScreen::RobotChoice robotChoice = TitleScreen::ROBOT_ARM;
	Difficulty difficulty = MEDIUM;
	bool term = false;

	GameScreen();
	void Initialize(TitleScreen::RobotChoice choice, Difficulty level);
	bool Terminate(void) const;
	void RunOneStep(void);
	void Render(void) const;
};

GameScreen::GameScreen() :
	arm(physics.GetConfig()), mobile(physics.GetConfig()), opponent(physics.GetConfig())
{
}

void GameScreen::Initialize(TitleScreen::RobotChoice choice, Difficulty level)
{
	term = false;
	robotChoice = choice;
	difficulty = level;

	if (TitleScreen::ROBOT_ARM == choice)
	{
		robot = &arm;
	}
	else
	{
		robot = &mobile;
	}

	robot->SetDifficulty(level);
	robot->Reset();
	opponent.SetDifficulty(level);
	opponent.Reset();
	physics.ResetPuck(EV_NONE);

	printf("Game start: %s, %s\n", TitleScreen::RobotName(choice), TitleScreen::DifficultyName(level));
}

bool GameScreen::Terminate(void) const
{
	return term;
}

void GameScreen::RunOneStep(void)
{
	FsPollDevice();
	auto key = FsInkey();
	if (FSKEY_ESC == key)
	{
		term = true; 
		return;
	}

	// TODO (next step): measure dt, physics.Step, PredictPuckPath, opponent.Update, robot->Update, scoring
}

void GameScreen::Render(void) const
{
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);   
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	physics.Draw();
	glPushMatrix();
	glTranslated(TABLE_OX, TABLE_OY, 0.0);
	robot->Draw();
	glPopMatrix();

	char hud[128];
	snprintf(hud, sizeof(hud), "%s - %s", TitleScreen::RobotName(robotChoice), TitleScreen::DifficultyName(difficulty));
	glColor3ub(0, 0, 0);
	glRasterPos2i(20, WINDOW_HEI - 50);
	YsGlDrawFontBitmap12x16(hud);
	glRasterPos2i(20, WINDOW_HEI - 25);
	YsGlDrawFontBitmap8x12("ESC: back to title");

	FsSwapBuffers();
}


class MainProgram
{
public:
	enum
	{
		STATE_TITLE,
		STATE_GAME,
		STATE_QUIT
	};

	int state = STATE_TITLE;

	TitleScreen title;
	GameScreen game;

	bool Terminate(void) const;
	void RunOneStep(void);
	void Render(void) const;
};

bool MainProgram::Terminate(void) const
{
	return STATE_QUIT == state;
}

void MainProgram::RunOneStep(void)
{
	switch (state)
	{
	case STATE_TITLE:
		title.RunOneStep();
		if (TitleScreen::ENDSTATE_START_GAME == title.endState)
		{
			state = STATE_GAME;
			game.Initialize(title.robotChoice, title.difficulty);
		}
		else if (TitleScreen::ENDSTATE_QUIT == title.endState)
		{
			state = STATE_QUIT;
		}
		break;
	case STATE_GAME:
		game.RunOneStep();
		if (true == game.Terminate())
		{
			state = STATE_TITLE;
			title.Initialize();
		}
		break;
	}
}

void MainProgram::Render(void) const
{
	switch (state)
	{
	case STATE_TITLE:
		title.Render();
		break;
	case STATE_GAME:
		game.Render();
		break;
	}
}



int main(void)
{
	FsOpenWindow(0, 0, WINDOW_WID, WINDOW_HEI, 1, "Air Hockey");

	MainProgram program;
	while (true != program.Terminate())
	{
		program.RunOneStep();
		program.Render();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return 0;
}
