#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>         
#include <math.h>

#include "shared.h"
#include "robotarm.h"
#include "fssimplewindow.h"

const double VIEW_SIDE = 40.0, VIEW_TOP = 270.0, VIEW_BOTTOM = 40.0;
const int TEST_WIN_WID = 378, TEST_WIN_HEI = 726;

void SetTableProjection(const TableConfig &cfg) {
    int wid, hei;
    FsGetWindowSize(wid, hei);
    glViewport(0,0,wid,hei);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-VIEW_SIDE, cfg.width + VIEW_SIDE, cfg.height + VIEW_BOTTOM, -VIEW_TOP, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

Vec2 MouseToTable(const TableConfig &cfg, int mx, int my) {
    int wid, hei;
    FsGetWindowSize(wid, hei);
    Vec2 p;
    p.x = -VIEW_SIDE + mx * (cfg.width + 2.0 * VIEW_SIDE) / wid;
    p.y = -VIEW_TOP + my * (cfg.height + VIEW_TOP + VIEW_BOTTOM) / hei;
    return p;
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

    bool ok = arm.InverseKinematics({275.0, 372.911}, t1, t2);
    printf("IK home: ok=%d  t1=%f (expect 0.785398)  t2=%f (expect 1.570796)\n", ok, t1, t2);

    Vec2 points[] = {{275.0, 20.0}, {275.0, 60.0}, {275.0, 560.0}};
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
        double a = RandRange(-PI/2.0, PI);
        double b = RandRange(0.0, 170.0*(PI/180.0));

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

    int goalLeft = (int)(cfg.width / 2.0 - cfg.goalWidth / 2.0);
    int goalRight = (int)(cfg.width / 2.0 + cfg.goalWidth / 2.0);
    int reachable = 0, total = 0;
    for (int x = goalLeft; x <= goalRight; x++) {
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

    FsOpenWindow(0, 0, TEST_WIN_WID, TEST_WIN_HEI, 1, "IK Mouse Test");

    for (;;) {
        FsPollDevice();
        int key = FsInkey();
        if (FSKEY_ESC == key) { break; }

        int lb, mb, rb, mx, my;
        FsGetMouseState(lb, mb, rb, mx, my);

        Vec2 target = MouseToTable(cfg, mx, my);

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

    FsOpenWindow(0, 0, TEST_WIN_WID, TEST_WIN_HEI, 1, "Arm Chase Test");

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

        Vec2 target = MouseToTable(cfg, mx, my);

        arm.CommandMoveTo(target);
        arm.Update(dt);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        SetTableProjection(cfg);
        DrawTestTable(cfg);
        arm.Draw();
        glColor3ub(220, 0, 0);
        DrawCircle(target.x, target.y, 4.0);
        Vec2 g = arm.GetGoal();
        glColor3ub(0, 180, 0);
        DrawCircle(g.x, g.y, 6.0);
        FsSwapBuffers();
        FsSleep(10);
    }
}

void TestArmDrawing(void) {
    TableConfig cfg;
    RobotArm arm(cfg);
    double t1, t2;
    
    FsOpenWindow(0, 0, TEST_WIN_WID, TEST_WIN_HEI, 1, "Robot Arm Test");

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

void TestOneElbowSide(void) {
    TableConfig cfg;
    double t1, t2;

    RobotArm arm(cfg);
    arm.SetDifficulty(MEDIUM);
    arm.InverseKinematics({275.0, 180.0}, t1, t2);
    arm.SetJointAngles(t1, t2);
    arm.SetTargetAngles(t1, t2);

    double g1, g2;
    arm.InverseKinematics({480.0, 80.0}, g1, g2);
    arm.SetTargetAngles(g1, g2);

    int steps = 0;
    double maxY = -1e9, minX = 1e9, maxX = -1e9;
    while (steps < 1000) {
        arm.Update(0.01);
        steps++;
        Vec2 p = arm.GetPaddleState().pos;
        maxY = fmax(maxY, p.y);
        minX = fmin(minX, p.x);
        maxX = fmax(maxX, p.x);
        double a, b;
        arm.GetJointAngles(a, b);
        if (a == g1 && b == g2) { break; }
    }
    printf("(275,180)->(480,80): %.2f s, max y %.0f, x %.0f..%.0f (expect 0.28 s, 213, 286..480)\n",
           steps * 0.01, maxY, minX, maxX);

    RobotArm arm2(cfg);
    arm2.SetDifficulty(MEDIUM);
    arm2.InverseKinematics({150.0, 80.0}, t1, t2);
    arm2.SetJointAngles(t1, t2);
    arm2.SetTargetAngles(t1, t2);

    maxY = -1e9; minX = 1e9; maxX = -1e9;
    for (int n = 0; n < 400; n++) {
        Vec2 target = {150.0 + 250.0 * fmin(1.0, n / 300.0), 80.0};
        if (arm2.InverseKinematics(target, t1, t2)) {
            arm2.SetTargetAngles(t1, t2);
        }
        arm2.Update(0.01);
        Vec2 p = arm2.GetPaddleState().pos;
        maxY = fmax(maxY, p.y);
        minX = fmin(minX, p.x);
        maxX = fmax(maxX, p.x);
    }
    Vec2 end = arm2.GetPaddleState().pos;
    printf("line y=80: end (%.1f, %.1f), max y %.0f, x %.0f..%.0f (expect (400.0, 80.0), 80, 150..400)\n",
           end.x, end.y, maxY, minX, maxX);
}

double BoxViolation(const TableConfig &cfg, Vec2 p) {
    double r = cfg.malletRadius;
    return fmax(fmax(r - p.x, p.x - (cfg.width - r)), fmax(r - p.y, p.y - (cfg.height / 2.0 - r)));
}

double OffLine(Vec2 a, Vec2 b, Vec2 p) {
    Vec2 d = b - a;
    double len = Length(d);
    if (len < 1e-9) { return Length(p - a); }
    Vec2 u = d / len;
    Vec2 rel = p - a;
    return Length(rel - u * Dot(rel, u));
}

void TestCommandMoveTo(void) {
    TableConfig cfg;

    RobotArm home(cfg);
    double h1, h2;
    home.GetJointAngles(h1, h2);
    Vec2 hp = home.GetPaddleState().pos;
    printf("home: angles (%.4f, %.4f), paddle (%.1f, %.1f), %s, done=%d (expect (0.3483, 2.4450), (275.0, 180.0), HOLD, 1)\n",
           h1, h2, hp.x, hp.y, home.GetStateName(), home.IsCommandDone());

    Vec2 clampPoints[] = {{300, 200}, {-50, -50}, {275, 600}, {275, 10}, {600, 200}, {30, 420}};
    const char *clampExpect[] = {"(300.0, 200.0)", "(30.0, 30.0)", "(275.0, 419.0)", "(275.0, 47.0)", "(520.0, 200.0)", "(63.9, 361.9)"};
    int i = 0;
    for (Vec2 p : clampPoints) {
        RobotArm arm(cfg);
        arm.CommandMoveTo(p);
        Vec2 g = arm.GetGoal();
        printf("clamp (%.0f, %.0f) -> (%.1f, %.1f) (expect %s)\n", p.x, p.y, g.x, g.y, clampExpect[i]);
        i++;
    }

    Vec2 targets[] = {{100, 300}, {480, 80}, {275, 600}, {-50, -50}, {275, 10}, {30, 420}, {520, 30}};
    for (Vec2 t : targets) {
        RobotArm arm(cfg);
        Vec2 start = arm.GetPaddleState().pos;
        arm.CommandMoveTo(t);
        Vec2 g = arm.GetGoal();
        int steps = 0;
        double maxOff = 0.0, maxOut = -1e9;
        while (!arm.IsCommandDone() && steps < 1000) {
            arm.Update(0.01);
            steps++;
            Vec2 p = arm.GetPaddleState().pos;
            maxOff = fmax(maxOff, OffLine(start, g, p));
            maxOut = fmax(maxOut, BoxViolation(cfg, p));
        }
        Vec2 p = arm.GetPaddleState().pos;
        printf("move to (%.0f, %.0f): end (%.1f, %.1f) after %.2f s, max off-line %.2f px, in box %s\n",
               t.x, t.y, p.x, p.y, steps * 0.01, maxOff, maxOut <= 1e-6 ? "yes" : "NO");
    }

    RobotArm arm(cfg);
    double maxOut = -1e9;
    for (int n = 0; n < 400; n++) {
        arm.CommandMoveTo({150.0 + 250.0 * fmin(1.0, n / 300.0), 80.0});
        arm.Update(0.01);
        maxOut = fmax(maxOut, BoxViolation(cfg, arm.GetPaddleState().pos));
    }
    Vec2 p = arm.GetPaddleState().pos;
    printf("every frame along y=80: end (%.1f, %.1f), %s, in box %s (expect (400.0, 80.0), HOLD, yes)\n",
           p.x, p.y, arm.GetStateName(), maxOut <= 1e-6 ? "yes" : "NO");
}

void TestMoveBounds(void) {
    TableConfig cfg;
    const int N = 5000;
    for (Difficulty level : {EASY, MEDIUM, HARD}) {
        srand(1);
        int leftBox = 0, notDone = 0;
        double worst = 0.0, sumTime = 0.0, maxTime = 0.0;
        for (int k = 0; k < N; k++) {
            RobotArm arm(cfg);
            arm.SetDifficulty(level);
            arm.CommandMoveTo({RandRange(-100.0, cfg.width + 100.0), RandRange(-100.0, cfg.height)});
            int steps = 0;
            while (!arm.IsCommandDone() && steps < 2000) { arm.Update(0.01); steps++; }

            arm.CommandMoveTo({RandRange(-100.0, cfg.width + 100.0), RandRange(-100.0, cfg.height)});
            double out = -1e9;
            steps = 0;
            while (!arm.IsCommandDone() && steps < 2000) {
                arm.Update(0.01);
                steps++;
                out = fmax(out, BoxViolation(cfg, arm.GetPaddleState().pos));
            }
            if (!arm.IsCommandDone()) { notDone++; }
            if (out > 1e-6) { leftBox++; worst = fmax(worst, out); }
            sumTime += steps * 0.01;
            maxTime = fmax(maxTime, steps * 0.01);
        }
        printf("bounds difficulty %d: %d moves, left box %d (worst %.2f px), never finished %d, avg %.2f s, max %.2f s\n",
               level, N, leftBox, worst, notDone, sumTime / N, maxTime);
    }
}

void TestTimeToReach(void) {
    TableConfig cfg;
    for (Difficulty level : {EASY, MEDIUM, HARD}) {
        srand(1);
        int skipped = 0;
        double maxErr = 0.0;
        for (int k = 0; k < 1000; k++) {
            RobotArm arm(cfg);
            arm.SetDifficulty(level);
            Vec2 p = {RandRange(30.0, 520.0), RandRange(30.0, 420.0)};
            double pred = arm.TimeToReach(p);
            if (pred < 0.0) {
                skipped++;
                continue;
            }
            arm.CommandMoveTo(p);
            int steps = 0;
            while (!arm.IsCommandDone() && steps < 100000) {
                arm.Update(0.001);
                steps++;
            }
            maxErr = fmax(maxErr, fabs(steps * 0.001 - pred));
        }
        printf("TimeToReach difficulty %d: max |sim - pred| = %.4f s, unreachable %d / 1000 (expect <= 0.0014 s, 61)\n",
               level, maxErr, skipped);
    }

    RobotArm arm(cfg);
    Vec2 named[] = {{275, 600}, {-50, 100}, {275, 10}, {30, 420}, {275, 180}, {480, 80}};
    const char *expect[] = {"-1", "-1", "-1", "-1", "0.0000", "0.2814"};
    int i = 0;
    for (Vec2 p : named) {
        printf("TimeToReach(%.0f, %.0f) = %.4f (expect %s)\n", p.x, p.y, arm.TimeToReach(p), expect[i]);
        i++;
    }
}

void TestStrikeMetrics(void) {
    TableConfig cfg;
    const double contact = cfg.malletRadius + cfg.puckRadius;
    Vec2 humanGoal = {cfg.width / 2.0, cfg.height};
    Vec2 hits[] = {{275.0, 250.0}, {175.0, 300.0}, {400.0, 200.0}};

    for (Difficulty level : {EASY, MEDIUM, HARD}) {
        for (Vec2 hit : hits) {
            RobotArm arm(cfg);
            arm.SetDifficulty(level);
            Vec2 dir = Normalize(humanGoal - hit);
            Vec2 windup = hit - dir * (contact + 50.0);
            arm.CommandMoveTo(windup);
            while (!arm.IsCommandDone()) { arm.Update(0.005); }

            arm.CommandStrike(hit, dir, 800.0);
            int steps = 0;
            double maxOff = 0.0, maxOut = -1e9, contactSpeed = -1.0;
            while (!arm.IsCommandDone() && steps < 2000) {
                arm.Update(0.005);
                steps++;
                MalletState m = arm.GetPaddleState();
                maxOut = fmax(maxOut, BoxViolation(cfg, m.pos));
                if (strcmp(arm.GetStateName(), "STRIKE") == 0) {
                    Vec2 rel = m.pos - hit;
                    maxOff = fmax(maxOff, Length(rel - dir * Dot(rel, dir)));
                    if (contactSpeed < 0.0 && Dot(rel, dir) >= -contact) {
                        contactSpeed = Dot(m.vel, dir);
                    }
                }
            }
            Vec2 p = arm.GetPaddleState().pos;
            printf("strike diff %d hit (%.0f, %.0f): contact speed %.0f px/s, max off-line %.2f px, total %.2f s, in box %s, end (%.1f, %.1f) %s\n",
                   level, hit.x, hit.y, contactSpeed, maxOff, steps * 0.005, maxOut <= 1e-6 ? "yes" : "NO",
                   p.x, p.y, arm.GetStateName());
        }
    }
}

void TestStrike(void) {
    TableConfig cfg;
    RobotArm arm(cfg);
    FsOpenWindow(0, 0, TEST_WIN_WID, TEST_WIN_HEI, 1, "Strike Test");

    Vec2 puck = {cfg.width / 2.0, 300.0};
    Vec2 humanGoal = {cfg.width / 2.0, cfg.height};
    const char *lastState = "";
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
        if (FSKEY_R == key) { arm.Reset(); }
        if (FSKEY_S == key) { arm.CommandStrike(puck, humanGoal - puck, 800.0); }

        int lb, mb, rb, mx, my;
        for (;;) {
            int evt = FsGetMouseEvent(lb, mb, rb, mx, my);
            if (FSMOUSEEVENT_NONE == evt) { break; }
            if (FSMOUSEEVENT_LBUTTONDOWN == evt) { puck = MouseToTable(cfg, mx, my); }
        }

        arm.Update(dt);
        if (strcmp(lastState, arm.GetStateName()) != 0) {
            lastState = arm.GetStateName();
            printf("state: %s\n", lastState);
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        SetTableProjection(cfg);
        DrawTestTable(cfg);

        glColor3ub(0, 0, 0);
        DrawCircle(puck.x, puck.y, cfg.puckRadius);

        Vec2 a, b;
        arm.GetStrikeLine(a, b);
        glColor3ub(0, 160, 0);
        glBegin(GL_LINES);
        glVertex2d(a.x, a.y);
        glVertex2d(b.x, b.y);
        glEnd();
        DrawCircle(a.x, a.y, 5.0);

        arm.Draw();
        FsSwapBuffers();
        FsSleep(10);
    }
}

void TestClickToMove(void) {
    TableConfig cfg;
    RobotArm arm(cfg);
    FsOpenWindow(0, 0, TEST_WIN_WID, TEST_WIN_HEI, 1, "Click To Move Test");

    const char *lastState = "";
    bool wasDone = true;
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
        if (FSKEY_R == key) { arm.Reset(); }

        int lb, mb, rb, mx, my;
        for (;;) {
            int evt = FsGetMouseEvent(lb, mb, rb, mx, my);
            if (FSMOUSEEVENT_NONE == evt) { break; }
            if (FSMOUSEEVENT_LBUTTONDOWN == evt) {
                Vec2 click = MouseToTable(cfg, mx, my);
                arm.CommandMoveTo(click);
                Vec2 g = arm.GetGoal();
                printf("click (%.0f, %.0f) -> goal (%.1f, %.1f)\n", click.x, click.y, g.x, g.y);
            }
        }

        arm.Update(dt);

        if (strcmp(lastState, arm.GetStateName()) != 0) {
            lastState = arm.GetStateName();
            printf("state: %s\n", lastState);
        }
        if (arm.IsCommandDone() && !wasDone) { printf("done\n"); }
        wasDone = arm.IsCommandDone();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        SetTableProjection(cfg);
        DrawTestTable(cfg);
        arm.Draw();
        Vec2 g = arm.GetGoal();
        glColor3ub(0, 180, 0);
        DrawCircle(g.x, g.y, 6.0);
        FsSwapBuffers();
        FsSleep(10);
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
    TestOneElbowSide();
    TestCommandMoveTo();
    TestMoveBounds();
    TestTimeToReach();
    TestStrikeMetrics();
    // TestArmChase();
    // TestClickToMove();
    TestStrike();
    return 0;
}