#include <stdio.h>
#include <iostream>
#include <stdlib.h>         
#include <math.h>          
#include <initializer_list>

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

void DrawTestTable(const TableConfig &cfg) {
    glColor3ub(200, 0, 0);
    DrawRect(0.0, 0.0, cfg.width, cfg.height, false);

    glColor3ub(200, 0, 0);
    glBegin(GL_LINES);
    glVertex2d(0.0, cfg.height / 2.0);
    glVertex2d(cfg.width, cfg.height / 2.0);
    glEnd();

    double goalLeft = cfg.width/2.0 - cfg.goalWidth / 2.0;
    double goalRight = cfg.width/2.0 + cfg.goalWidth / 2.0;

    glLineWidth(6);
    glBegin(GL_LINES);
    glVertex2d(goalLeft, 0);
    glVertex2d(goalRight, 0);
    glVertex2d(goalLeft, cfg.height);
    glVertex2d(goalRight, cfg.height);
    glEnd();
    glLineWidth(1);
}

void TestInverseKinematics(void) {
    TableConfig cfg;
    RobotArm arm(cfg);
    double t1, t2;

    bool ok = arm.InverseKinematics({200.0, 316.228}, t1, t2);
    printf("IK home: ok=%d  t1=%f (expect 0.785398)  t2=%f (expect 1.570796)\n", ok, t1, t2);

    Vec2 points[] = {{200.0, 20.0}, {200.0, 50.0}, {200.0, 500.0}};
    const char *expected[] = {"0 (too close to base)", "1", "0 (past full reach)"};
    int i = 0;
    for (Vec2 p : points) {
        ok = arm.InverseKinematics(p, t1, t2);
        printf("IK (%.0f, %.0f): ok=%d  expect %s\n", p.x, p.y, ok, expected[i]);
        i++;
    }

    srand(1);   
    const int N = 10000;
    int failures = 0;
    double maxPosError = 0.0;
    double maxAngleError = 0.0;
    for (int k = 0; k < N; k++) {
        double a = RandRange(0.0, PI);
        double b = RandRange(-170.0*(PI/180.0), 170.0*(PI/180.0));

        arm.SetJointAngles(a, b);                  
        Vec2 p = arm.GetPaddleState().pos;

        if (!arm.InverseKinematics(p, t1, t2)) {   
            failures++;
            continue;
        }
        maxAngleError = fmax(maxAngleError, fmax(fabs(t1 - a), fabs(t2 - b)));

        arm.SetJointAngles(t1, t2);                
        Vec2 q = arm.GetPaddleState().pos;
        maxPosError = fmax(maxPosError, Length(q - p));
    }
    printf("IK round trip: %d poses, %d failures (expect 0)\n", N, failures);
    printf("IK max position error = %.3e px, max angle error = %.3e rad\n", maxPosError, maxAngleError);

    int reachable = 0, total = 0;
    for (int x = 125; x <= 275; x++) {
        total++;
        if (arm.InverseKinematics({(double)x, 80.0}, t1, t2)) {
            reachable++;
        }
    }
    printf("IK defense line y=80: %d / %d reachable (expect all)\n", reachable, total);
}

void TestIKMouse(void) {
    TableConfig cfg;
    RobotArm arm(cfg);
    const double margin = 40.0;  

    FsOpenWindow(0, 0, 360, 660, 1, "IK Mouse Test");

    for (;;) {
        FsPollDevice();
        int key = FsInkey();
        if (FSKEY_ESC == key) { break; }

        int lb, mb, rb, mx, my;
        FsGetMouseState(lb, mb, rb, mx, my);

        int wid, hei;
        FsGetWindowSize(wid, hei);
        Vec2 target;
        target.x = -margin + mx * (cfg.width  + 2.0*margin) / wid;
        target.y = -margin + my * (cfg.height + 2.0*margin) / hei;

        double t1, t2;
        bool reachable = arm.InverseKinematics(target, t1, t2);
        if (reachable) {
            arm.SetJointAngles(t1, t2);
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        SetTableProjection(cfg);
        DrawTestTable(cfg);
        arm.Draw();
        if (reachable) { glColor3ub(0, 180, 0); } 
        else { glColor3ub(220, 0, 0); }
        DrawCircle(target.x, target.y, 5.0);
        FsSwapBuffers();
        FsSleep(10);
    }
}

void TestArmChase(void) {
    TableConfig cfg;
    RobotArm arm(cfg);
    const double margin = 40.0;  

    FsOpenWindow(0, 0, 360, 660, 1, "Arm Chase Test");

    FsPassedTime();
    for (;;) {
        double dt = FsPassedTime() / 1000.0;
        if (dt > 0.05) { dt = 0.05; }
        FsPollDevice();
        int key = FsInkey();
        if (FSKEY_ESC == key) { break; }
        if (FSKEY_E == key) { arm.SetDifficulty(EASY); }
        if (FSKEY_M == key) { arm.SetDifficulty(MEDIUM); }
        if (FSKEY_H == key) { arm.SetDifficulty(HARD); }

        int lb, mb, rb, mx, my;
        FsGetMouseState(lb, mb, rb, mx, my);

        int wid, hei;
        FsGetWindowSize(wid, hei);
        Vec2 target;
        target.x = -margin + mx * (cfg.width  + 2.0*margin) / wid;
        target.y = -margin + my * (cfg.height + 2.0*margin) / hei;

        double t1, t2;
        bool reachable = arm.InverseKinematics(target, t1, t2);
        if (reachable) {
            arm.SetTargetAngles(t1, t2);
        }

        arm.Update(dt);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        SetTableProjection(cfg);
        DrawTestTable(cfg);
        arm.Draw();
        if (reachable) { glColor3ub(0, 180, 0); } 
        else { glColor3ub(220, 0, 0); }
        DrawCircle(target.x, target.y, 5.0);
        FsSwapBuffers();
        FsSleep(10);
    }
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
            arm.SetJointAngles(t1+step,t2);
        } else if (FSKEY_A==key) {
            arm.SetJointAngles(t1-step,t2);
        } else if (FSKEY_W==key) {
            arm.SetJointAngles(t1,t2+step); 
        } else if (FSKEY_S==key) {
            arm.SetJointAngles(t1,t2-step); 
        } else if (FSKEY_H==key) {
            arm.SetJointAngles(PI/4,PI/2);
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        SetTableProjection(cfg);
        DrawTestTable(cfg);
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

void TestJointSpeedLimit(void) {
    TableConfig cfg;
    struct Case { Difficulty level; double speed; double s1, s2, g1, g2; };
    Case cases[] = {
        {EASY, 2.0, 1.5, 0.2, 0.5, 1.2},
        {MEDIUM, 4.0, 0.5, 0.5, 1.5, 1.0},
        {HARD, 6.0, 0.3, 2.5, 2.5, 0.3},
    };
    const double dt = 0.01;

    for (const Case &c : cases) {
        RobotArm arm(cfg);
        arm.SetDifficulty(c.level);
        arm.SetJointAngles(c.s1, c.s2);
        arm.SetTargetAngles(c.g1, c.g2);
        int steps = 0;
        double maxRate = 0.0;
        bool arrived = false;
        while (steps < 1000) {
            double a0, b0;
            arm.GetJointAngles(a0,b0);
            arm.Update(dt);
            steps++;
            double a1, b1;
            arm.GetJointAngles(a1,b1);
            maxRate = fmax(maxRate, fmax(fabs(a1-a0), fabs(b1-b0))/dt);
            if (a1 == c.g1 && b1 == c.g2) {
                arrived = true;
                break;
            }
        }
        double predicted = fmax(fabs(c.g1 - c.s1), fabs(c.g2 - c.s2)) / c.speed;
        printf("speed %.0f: arrived=%d after %.2f s (predicted %.3f s), max joint rate %.3f\n",
               c.speed, arrived, steps * dt, predicted, maxRate);
    }
}

int main(void) {
    TestSharedTypes();
    TestRobotInterface();
    TestForwardKinematics();
    TestInverseKinematics();
    // TestArmDrawing();
    // TestIKMouse();
    TestJointSpeedLimit();
    TestArmChase();
    return 0;
}