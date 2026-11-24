#ifndef TITLE_H_IS_INCLUDED
#define TITLE_H_IS_INCLUDED

#include "shared.h"


class TitleScreen
{
public:
	enum
	{
		ENDSTATE_NONE,
		ENDSTATE_START_GAME,
		ENDSTATE_QUIT
	};

	enum RobotChoice
	{
		ROBOT_ARM,
		ROBOT_MOBILE
	};

	int endState = ENDSTATE_NONE;
	RobotChoice robotChoice = ROBOT_ARM;   
	Difficulty difficulty = MEDIUM;       

	void Initialize(void);
	void RunOneStep(void);
	void Render(void) const;

	static const char *RobotName(RobotChoice choice);
	static const char *DifficultyName(Difficulty level);

private:
	enum
	{
		ITEM_ROBOT,
		ITEM_DIFFICULTY,
		ITEM_START,
		ITEM_QUIT,
		NUM_ITEMS
	};

	int cursor = ITEM_START;   

	void Activate(int item, int dir);

	void GetItemBox(int item, int &x1, int &y1, int &x2, int &y2) const;

	int ItemAt(int mx, int my) const;
};

#endif
