#include <stdio.h>

#include "fssimplewindow.h"
#include "shared.h"
#include "robot.h"
#include "robotarm.h"
#include "mobile_robot.h"
#include "opponent.h"
#include "table_puck.h"

int main(void)
{
	Physics physics;
	const TableConfig &cfg = physics.GetConfig();

	RobotArm arm(cfg);
	MobileRobot mobile(cfg);
	OpponentLogic opponent(cfg);

	Robot *robot = &arm;    // both robots share the Robot interface
	printf("All components constructed. T = switch robot, ESC = quit.\n");

	FsOpenWindow(0, 0, 500, 950, 1, "Air Hockey (beta build check)");

	for (;;)
	{
		FsPollDevice();
		int key = FsInkey();
		if (FSKEY_ESC == key)
		{
			break;
		}
		if (FSKEY_T == key)
		{
			robot = (robot == &arm ? (Robot *)&mobile : (Robot *)&arm);
		}

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		physics.Draw();

		glPushMatrix();
		glTranslated(TABLE_OX, TABLE_OY, 0.0);
		robot->Draw();
		glPopMatrix();

		FsSwapBuffers();
		FsSleep(10);
	}
	return 0;
}
