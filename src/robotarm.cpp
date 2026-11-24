#include <math.h>
#include <initializer_list>
#include <algorithm>

#include "robotarm.h"
#include "fssimplewindow.h"

RobotArm::RobotArm(const TableConfig &cfg) {
    base = {cfg.width/2, 0};
    L1 = 0.5 * Length(Vec2{cfg.width/2, cfg.height/2});
    L2 = 0.5 * Length(Vec2{cfg.width/2, cfg.height/2});
    minTheta1 = 0;
    maxTheta1 = PI;
    minTheta2 = -170*(PI/180);
    maxTheta2 = 170*(PI/180);
    maxJointSpeed = PI;
    theta1 = PI/4;
    theta2 = PI/2;
    target1 = theta1;
    target2 = theta2;
    paddle.pos = ForwardKinematics(theta1, theta2);
    paddle.vel = {0.0, 0.0};
    paddle.radius = cfg.malletRadius;

}

static double MoveToward(double current, double target, double maxStep) {
    double error = target - current;
    if (fabs(error) <= maxStep) { return target; }
    if (error > 0.0) { return current + maxStep; }
    return current - maxStep;
}

void RobotArm::SetDifficulty(Difficulty level) {
    switch (level) {
        case EASY: maxJointSpeed = 2.0; break;
        case MEDIUM: maxJointSpeed = 4.0; break;
        case HARD: maxJointSpeed = 6.0; break;
    }
}

void RobotArm::Reset(void){
    return;
}

void RobotArm::CommandMoveTo(Vec2 target) {
    return;
}

void RobotArm::CommandStrike(Vec2 hitPoint, Vec2 direction, double speed) {
    return;
}

double RobotArm::TimeToReach(Vec2 point) const {
    return -1.0;
}

bool RobotArm::IsCommandDone(void) const {
    return state == HOLD;
}

void RobotArm::Update(double dt) {
    Vec2 oldPos = paddle.pos;
    MoveJointsToward(dt);
    paddle.pos = ForwardKinematics(theta1, theta2);
    if (dt>0.0) {
        paddle.vel = (paddle.pos - oldPos) / dt;
    } else {
        paddle.vel = {0.0, 0.0};
    }
}

MalletState RobotArm::GetPaddleState(void) const {
    return paddle;
}

static Vec2 Perp(Vec2 d) {
    return Vec2{-d.y, d.x};
}

static void DrawTaperedQuad(Vec2 a, Vec2 b, double wA, double wB) {
    Vec2 n = Perp(Normalize(b - a));
    Vec2 a1 = a + n * (wA / 2.0), a2 = a - n * (wA / 2.0);
    Vec2 b1 = b + n * (wB / 2.0), b2 = b - n * (wB / 2.0);
    DrawQuad(a1.x, a1.y, b1.x, b1.y, b2.x, b2.y, a2.x, a2.y);
}

static void DrawLink(Vec2 a, Vec2 b, double wA, double wB) {
    Vec2 n = Perp(Normalize(b - a));

    glColor3ub(110, 50, 10);
    DrawTaperedQuad(a, b, wA + 4.0, wB + 4.0);

    const double across[] = {0.5, 0.15, -0.5};
    const unsigned char shade[3][3] = {{205, 95, 20}, {255, 190, 110}, {160, 70, 15}};
    glShadeModel(GL_SMOOTH);
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i < 3; i++) {
        glColor3ub(shade[i][0], shade[i][1], shade[i][2]);
        Vec2 pa = a + n * (across[i] * wA);
        Vec2 pb = b + n * (across[i] * wB);
        glVertex2d(pa.x, pa.y);
        glVertex2d(pb.x, pb.y);
    }
    glEnd();
    glShadeModel(GL_FLAT);

    glColor3ub(90, 40, 10);
    Vec2 s1 = a + (b - a) * 0.3 - n * (0.2 * wA);
    Vec2 s2 = a + (b - a) * 0.7 - n * (0.2 * wB);
    DrawTaperedQuad(s1, s2, 3.0, 3.0);

    glColor3ub(70, 70, 75);
    for (double t : {0.2, 0.8}) {
        Vec2 c = a + (b - a) * t + n * (0.25 * (wA + (wB - wA) * t));
        DrawCircle(c.x, c.y, 2.0);
    }
}

static void DrawJoint(Vec2 c, double r, double angle) {
    glColor3ub(40, 40, 45);
    DrawCircle(c.x, c.y, r);
    glColor3ub(120, 125, 135);
    DrawCircle(c.x, c.y, r * 0.78);
    glColor3ub(175, 180, 190);
    DrawCircle(c.x, c.y, r * 0.45);

    glColor3ub(55, 55, 60);
    for (int i = 0; i < 6; i++) {
        double a = angle + (double)i * PI / 3.0;
        DrawCircle(c.x + 0.62 * r * cos(a), c.y + 0.62 * r * sin(a), r * 0.09);
    }

    glColor3ub(235, 235, 240);
    DrawCircle(c.x - r * 0.15, c.y - r * 0.15, r * 0.12);
}

static void DrawBaseMount(Vec2 base) {
    glColor3ub(40, 40, 45);
    DrawRoundedRect(base.x - 50, base.y - 40, base.x + 50, base.y + 10, true, 8);
    glColor3ub(90, 90, 100);
    DrawRoundedRect(base.x - 46, base.y - 36, base.x + 46, base.y + 6, true, 6);

    glColor3ub(45, 45, 50);
    for (double dx : {-38.0, 38.0}) {
        for (double dy : {-28.0, -2.0}) {
            DrawCircle(base.x + dx, base.y + dy, 3.0);
        }
    }

    glColor3ub(240, 190, 20);
    for (int i = 0; i < 4; i++) {
        double x = base.x - 30 + i * 16;
        DrawQuad(x, base.y - 36, x + 8, base.y - 36, x + 2, base.y - 28, x - 6, base.y - 28);
    }
}

static void DrawMallet(Vec2 p, double r) {
    glColor3ub(255, 0, 0);
    DrawCircle(p.x, p.y, r);
    glColor3ub(125, 0, 0);
    DrawCircle(p.x, p.y, r * 0.85);
    glColor3ub(255, 0, 0);
    DrawCircle(p.x, p.y, r * 0.35);
}

static void DrawGripper(Vec2 palm, Vec2 p, Vec2 u, double knobR) {
    Vec2 n = Perp(u);

    for (double side : {1.0, -1.0}) {
        Vec2 root = palm + n * (side * (knobR + 4.0));
        Vec2 tip = p + u * (knobR * 0.6) + n * (side * (knobR + 2.5));
        glColor3ub(35, 35, 40);
        DrawTaperedQuad(root, tip, 8.0, 6.0);
        glColor3ub(140, 145, 155);
        DrawTaperedQuad(root, tip, 5.0, 3.0);
    }

    Vec2 left = palm + n * (knobR + 8.0);
    Vec2 right = palm - n * (knobR + 8.0);
    glColor3ub(35, 35, 40);
    DrawTaperedQuad(left, right, 11.0, 11.0);
    glColor3ub(110, 115, 125);
    DrawTaperedQuad(left, right, 7.0, 7.0);
}

void RobotArm::Draw(void) const {
    Vec2 elbow = base + Vec2{L1 * cos(theta1), L1 * sin(theta1)};
    Vec2 p = paddle.pos;
    Vec2 u = Normalize(p - elbow);
    double knobR = paddle.radius * 0.35;
    Vec2 palm = p - u * (knobR + 7.0);

    const double w1a = 30.0, w1b = 24.0, w2a = 22.0, w2b = 16.0;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4ub(0, 0, 0, 55);
    Vec2 off = {8.0, 10.0};
    DrawTaperedQuad(base + off, elbow + off, w1a, w1b);
    DrawTaperedQuad(elbow + off, palm + off, w2a, w2b);
    DrawCircle(base.x + off.x, base.y + off.y, 24.0);
    DrawCircle(elbow.x + off.x, elbow.y + off.y, 18.0);
    DrawCircle(p.x + 3.0, p.y + 4.0, paddle.radius);
    glDisable(GL_BLEND);

    DrawMallet(p, paddle.radius);
    DrawBaseMount(base);
    DrawLink(base, elbow, w1a, w1b);
    DrawLink(elbow, palm, w2a, w2b);
    DrawGripper(palm, p, u, knobR);
    DrawJoint(elbow, 17.0, theta1 + theta2);
    DrawJoint(base, 24.0, theta1);
    DrawJoint(palm, 8.0, theta1 + theta2);
}

Vec2 RobotArm::ForwardKinematics(double t1, double t2) const {
    Vec2 elbow = base + Vec2{L1*cos(t1), L1*sin(t1)};
    Vec2 paddlePos = elbow + Vec2{L2*cos(t1+t2), L2*sin(t1+t2)};
    return paddlePos;
}

bool RobotArm::InverseKinematics(Vec2 point, double &t1, double &t2) const {
    double dx = point.x - base.x;
    double dy = point.y - base.y;
    double D = (dx*dx + dy*dy - L1*L1 - L2*L2) / (2.0*L1*L2);

    if (D > 1.0 + 1e-9 || D < -1.0 - 1e-9) {
        return false;
    }
    D = Clamp(D, -1.0, 1.0);

    bool found = false;
    double best1 = 0.0, best2 = 0.0;
    double bestCost = 0.0;

    for (double sign : {1.0, -1.0}) {
        double cand2 = sign * acos(D);
        double cand1 = atan2(dy, dx) - atan2(L2*sin(cand2), L1 + L2*cos(cand2));   

        while (cand1 > PI) { cand1 -= 2.0*PI; }
        while (cand1 <= -PI) { cand1 += 2.0*PI; }

        if (cand1 < minTheta1 || cand1 > maxTheta1 || cand2 < minTheta2 || cand2 > maxTheta2) { continue; }

        double cost = fmax(fabs(cand1 - theta1), fabs(cand2 - theta2));

        if (!found || cost < bestCost) {
            found = true;
            best1 = cand1;
            best2 = cand2;
            bestCost = cost;
        }
    }

    if (!found) { return false; }

    t1 = best1;
    t2 = best2;
    return true;
}

void RobotArm::MoveJointsToward(double dt) {
    double maxStep = maxJointSpeed*dt;
    theta1 = MoveToward(theta1,target1, maxStep);
    theta2 = MoveToward(theta2,target2, maxStep);
    theta1 = Clamp(theta1, minTheta1, maxTheta1);
    theta2 = Clamp(theta2, minTheta2, maxTheta2);
}

// Debug helper functions
void RobotArm::SetJointAngles(double t1, double t2) {
    theta1 = t1;
    theta2 = t2;
    paddle.pos = ForwardKinematics(theta1, theta2);
}

void RobotArm::GetJointAngles(double &t1, double &t2) const {
    t1 = theta1;
    t2 = theta2;
}

void RobotArm::SetTargetAngles(double t1, double t2) {
    target1 = t1;
    target2 = t2;
}