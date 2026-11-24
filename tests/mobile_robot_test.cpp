#include <cmath>
#include <cstdio>

#include "mobile_robot.h"
#include "fssimplewindow.h"

namespace {
const int WINDOW_WIDTH = 400;
const int WINDOW_HEIGHT = 800;
const double DT = 0.01;

enum DemoMode {
    WAITING,
    MOVING,
    STRIKING
};

void SetTableProjection(const TableConfig &cfg)
{
    const double margin = 40.0;
    int wid, hei;
    FsGetWindowSize(wid, hei);
    glViewport(0, 0, wid, hei);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-margin, cfg.width + margin,
            cfg.height + margin, -margin, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void DrawTestTable(const TableConfig &cfg)
{
    // Table boundary and center line.
    glColor3ub(190, 35, 35);
    DrawRect(0.0, 0.0, cfg.width, cfg.height, false);
    glBegin(GL_LINES);
    glVertex2d(0.0, cfg.height * 0.5);
    glVertex2d(cfg.width, cfg.height * 0.5);
    glEnd();

    // Goal openings.
    const double goalLeft = cfg.width * 0.5 - cfg.goalWidth * 0.5;
    const double goalRight = cfg.width * 0.5 + cfg.goalWidth * 0.5;
    glLineWidth(5.0f);
    glBegin(GL_LINES);
    glVertex2d(goalLeft, 0.0);
    glVertex2d(goalRight, 0.0);
    glVertex2d(goalLeft, cfg.height);
    glVertex2d(goalRight, cfg.height);
    glEnd();
    glLineWidth(1.0f);

    // Highlight the mobile robot's legal workspace (top half).
    glColor3ub(80, 120, 150);
    DrawRect(0.0, 0.0, cfg.width, cfg.height * 0.5, false);
}

void DrawTarget(Vec2 p, double radius)
{
    glColor3ub(220, 70, 50);
    DrawCircle(p.x, p.y, radius);
    glColor3ub(255, 220, 190);
    DrawCircle(p.x, p.y, radius * 0.35);
}

void StartMove(MobileRobot &robot, DemoMode &mode)
{
    robot.Reset();
    robot.CommandMoveTo({100.0, 250.0});
    mode = MOVING;
}

void StartStrike(MobileRobot &robot, DemoMode &mode)
{
    robot.Reset();
    robot.CommandStrike({180.0, 250.0}, {0.0, 1.0}, 120.0);
    mode = STRIKING;
}
}

int main()
{
    TableConfig cfg;
    MobileRobot robot(cfg);
    DemoMode mode = WAITING;

    FsOpenWindow(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, 1,
                 "Mobile Robot Visual Test");

    // Start with a visible move demo. Press M to move, S to strike,
    // R to reset, and ESC to close the window.
    StartMove(robot, mode);

    while (FsCheckWindowOpen()) {
        FsPollDevice();
        const int key = FsInkey();
        if (FSKEY_ESC == key) {
            break;
        } else if (FSKEY_M == key) {
            StartMove(robot, mode);
        } else if (FSKEY_S == key) {
            StartStrike(robot, mode);
        } else if (FSKEY_R == key) {
            robot.Reset();
            mode = WAITING;
        }

        if (MOVING == mode || STRIKING == mode) {
            robot.Update(DT);
            if (robot.IsCommandDone()) {
                mode = WAITING;
            }
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        SetTableProjection(cfg);
        DrawTestTable(cfg);

        if (MOVING == mode) {
            DrawTarget({100.0, 250.0}, 9.0);
        } else if (STRIKING == mode) {
            DrawTarget({180.0, 250.0}, cfg.puckRadius);
        }

        robot.Draw();
        FsSwapBuffers();
        FsSleep(10);
    }

    return 0;
}
