#include <stdio.h>
#include <iostream>

#include "shared.h"
#include "robotarm.h"

void TestRobotInterface(void) {
    RobotArm arm;
    Robot &robot = arm;
    robot.SetDifficulty(MEDIUM);
    robot.Update(0.01);
    MalletState p = robot.GetPaddleState();
    printf("Paddle = (%f, %f)\n", p.pos.x, p.pos.y);
    printf("Done = %d\n", robot.IsCommandDone());
    printf("TimeToReach = %f\n", robot.TimeToReach({100.0, 100.0}));
}

void TestSharedTypes(void)
{
    Vec2 a = {3.0, 4.0};
    Vec2 product = a*2.0;
    Vec2 division = a/2.0;

    printf("Length = %f\n", Length(a));
    printf("Product: (%f, %f)\n", product.x, product.y);
    printf("Division: (%f, %f)\n", division.x, division.y);
    printf("Difficulty = %d\n", HARD);
    printf("PI = %.10f\n", PI);
}

int main(void) {
    TestSharedTypes();
    TestRobotInterface();
    return 0;
}