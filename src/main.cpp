#include <stdio.h>
#include <iostream>

#include "shared.h"
#include "robotarm.h"
#include "fssimplewindow.h"

void SetTableProjection(const TableConfig &cfg) {
    const double margin = 40.0;
    int wid, hei;
    FsGetWindowSize(wid, hei);
    glViewport(0,0,wid,hei);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-margin, cfg.width+margin, cfg.height+margin, -margin, -1,1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void TestArmDrawing(void) {
    TableConfig cfg;
    RobotArm arm(cfg);
    double t1, t2;
    
    FsOpenWindow(0,0,360,660,1, "Robot Arm Test");

    for (;;) {
        FsPollDevice();
        int key = FsInkey();
        if (FSKEY_ESC==key) { break; }

        const double step = 5.0 * (PI / 180.0);
        arm.GetJointAngles(t1, t2);
        if (FSKEY_Q==key) { 
            arm.SetJointAngles(t1+step,t2);; 
        } else if (FSKEY_A==key) {
            arm.SetJointAngles(t1-step,t2);; 
        } else if (FSKEY_W==key) {
            arm.SetJointAngles(t1,t2+step);; 
        } else if (FSKEY_S==key) {
            arm.SetJointAngles(t1,t2-step);; 
        } else if (FSKEY_H==key) {
            arm.SetJointAngles(PI/4,PI/2);
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        SetTableProjection(cfg);
        arm.Draw();
        FsSwapBuffers();
        FsSleep(10);
    }
}

void TestForwardKinematics(void) {
    TableConfig cfg;
    RobotArm arm(cfg);
    Robot &robot = arm; 
    MalletState p = robot.GetPaddleState();
    printf("Paddle = (%f, %f)\n", p.pos.x, p.pos.y);

    arm.SetJointAngles(0,0);
    p = robot.GetPaddleState();
    printf("Paddle = (%f, %f)\n", p.pos.x, p.pos.y);

    arm.SetJointAngles(PI/2,0);
    p = robot.GetPaddleState();
    printf("Paddle = (%f, %f)\n", p.pos.x, p.pos.y);

    arm.SetJointAngles(PI/2,PI/2);
    p = robot.GetPaddleState();
    printf("Paddle = (%f, %f)\n", p.pos.x, p.pos.y);
}

void TestRobotInterface(void) {
    TableConfig cfg;
    RobotArm arm(cfg);
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
    TestForwardKinematics();
    TestArmDrawing();
    return 0;
}