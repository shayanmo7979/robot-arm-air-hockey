#include <stdio.h>
#include <string.h>
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

const int WINDOW_WID = 750;
const int WINDOW_HEI = 1200;

const int WIN_SCORE = 7;                    // first to 7 goals wins
const double GOAL_PAUSE = 1.5;              // seconds the "GOAL!" message stays up before the serve
const double GAME_OVER_PAUSE = 4.0;         // seconds the result stays up before going back to the title
const int NUM_SUBSTEP = 4;                  // robot + physics sub-steps per frame, so the paddle velocity is smooth
const double PREDICT_HORIZON = 2.0;         // seconds of puck path the opponent looks ahead
const double PREDICT_DT = 1.0 / 120.0;      // spacing of the predicted puck samples

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

	int humanScore = 0;
	int robotScore = 0;
	PhysicsEvent lastGoal = EV_NONE;
	double pauseTimer = 0.0;                // > 0 while a goal message is shown (play is frozen)
	bool gameOver = false;
	const char *message = "";
	bool showDebug = false;                 // D key: opponent's defense line, shots and target

	GameScreen();
	void Initialize(TitleScreen::RobotChoice choice, Difficulty level);
	bool Terminate(void) const;
	void RunOneStep(void);
	void Render(void) const;

private:
	void PlayOneStep(double dt);
	void OnGoal(PhysicsEvent ev);
	void Serve(void);
	void DrawMessage(const char *msg) const;
	void DrawHud(void) const;
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

	humanScore = 0;
	robotScore = 0;
	lastGoal = EV_NONE;
	pauseTimer = 0.0;
	gameOver = false;
	message = "";
	FsPassedTime();

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

	if(key == FSKEY_D)
	{
		showDebug = !showDebug;
	}

	double dt = (double)FsPassedTime() / 1000.0;
	if (dt > 0.05) { dt = 0.05; }

	if(gameOver)
	{
		// Show the result, then go back to the title (any of SPACE / ENTER also skips the wait)
		pauseTimer = pauseTimer - dt;
		if(pauseTimer <= 0.0 || key == FSKEY_SPACE || key == FSKEY_ENTER)
		{
			term = true;
		}
		return;
	}

	if(pauseTimer > 0.0)
	{
		// Goal pause: play is frozen, then the puck is served to whoever was scored on
		pauseTimer = pauseTimer - dt;
		if(pauseTimer <= 0.0)
		{
			Serve();
		}
		return;
	}

	PlayOneStep(dt);
}

// One frame of play, in the order from the conceptual design:
// opponent decision -> robot motion -> puck physics -> events and scoring.
void GameScreen::PlayOneStep(double dt)
{
	PuckState puck = physics.GetPuck().s;
	MalletState human = physics.GetHumanState();
	PuckPath path = PredictPuckPath(puck, physics.GetConfig(), PREDICT_HORIZON, PREDICT_DT);
	opponent.Update(*robot, puck, human, path, dt);

	int lb;
	int mb;
	int rb;
	int mouseX;
	int mouseY;
	FsGetMouseState(lb, mb, rb, mouseX, mouseY);

	// The robot moves in the same sub-steps as the puck, so the paddle velocity the puck sees is right
	double h = dt / NUM_SUBSTEP;
	for(int i = 0; i < NUM_SUBSTEP; i++)
	{
		robot->Update(h);
		PhysicsEvent ev = physics.Step(h, mouseX, mouseY, robot->GetPaddleState());
		opponent.OnPhysicsEvent(ev);
		if(ev == EV_GOAL_HUMAN || ev == EV_GOAL_ROBOT)
		{
			OnGoal(ev);
			return;
		}
	}
}

// EV_GOAL_HUMAN: the puck went into the robot's goal (top), so the human scores.
void GameScreen::OnGoal(PhysicsEvent ev)
{
	lastGoal = ev;
	if(ev == EV_GOAL_HUMAN)
	{
		humanScore++;
		message = "YOU SCORE!";
	}
	else
	{
		robotScore++;
		message = "ROBOT SCORES!";
	}
	printf("Goal: You %d - %d Robot\n", humanScore, robotScore);

	if(humanScore >= WIN_SCORE || robotScore >= WIN_SCORE)
	{
		gameOver = true;
		message = (humanScore >= WIN_SCORE ? "YOU WIN!" : "ROBOT WINS!");
		pauseTimer = GAME_OVER_PAUSE;
	}
	else
	{
		pauseTimer = GOAL_PAUSE;
	}
}

// After a goal: puck, robot and opponent all reset together
void GameScreen::Serve(void)
{
	physics.ResetPuck(lastGoal);
	robot->Reset();
	opponent.Reset();
	message = "";
}

void GameScreen::Render(void) const
{
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	physics.Draw();
	glPushMatrix();
	glTranslated(TABLE_OX, TABLE_OY, 0.0);
	robot->Draw();
	if(showDebug)
	{
		opponent.DrawDebug();
	}
	glPopMatrix();

	DrawHud();
	if(pauseTimer > 0.0)
	{
		DrawMessage(message);
	}

	FsSwapBuffers();
}

// Score, opponent mode and controls below the table
void GameScreen::DrawHud(void) const
{
	const TableConfig &cfg = physics.GetConfig();
	int tableBottom = (int)(TABLE_OY + cfg.height);

	char str[128];
	snprintf(str, sizeof(str), "YOU %d  -  %d ROBOT", humanScore, robotScore);
	int textW = (int)strlen(str) * 20;
	glColor3ub(0, 0, 0);
	glRasterPos2i((int)(TABLE_OX + cfg.width / 2.0) - textW / 2, tableBottom + 60);
	YsGlDrawFontBitmap20x32(str);

	snprintf(str, sizeof(str), "%s - %s    Opponent: %s", TitleScreen::RobotName(robotChoice),
	         TitleScreen::DifficultyName(difficulty), opponent.GetModeName());
	glRasterPos2i(20, WINDOW_HEI - 50);
	YsGlDrawFontBitmap12x16(str);

	snprintf(str, sizeof(str), "Mouse: move mallet   D: debug view   ESC: back to title   First to %d wins", WIN_SCORE);
	glRasterPos2i(20, WINDOW_HEI - 25);
	YsGlDrawFontBitmap8x12(str);
}

// White box with centered text over the middle of the table
void GameScreen::DrawMessage(const char *msg) const
{
	const TableConfig &cfg = physics.GetConfig();
	int cx = (int)(TABLE_OX + cfg.width / 2.0);
	int cy = (int)(TABLE_OY + cfg.height / 2.0);
	int textW = (int)strlen(msg) * 20;
	int halfW = textW / 2 + 30;

	glColor3ub(255, 255, 255);
	DrawRect(cx - halfW, cy - 40, cx + halfW, cy + 40, true);
	glColor3ub(0, 0, 0);
	DrawRect(cx - halfW, cy - 40, cx + halfW, cy + 40, false);
	glRasterPos2i(cx - textW / 2, cy + 16);
	YsGlDrawFontBitmap20x32(msg);
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
