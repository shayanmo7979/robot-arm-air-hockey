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

void RobotArm::Draw(void) const {
    Vec2 elbow = base + Vec2{L1*cos(theta1), L1*sin(theta1)};
    glColor3ub(60,60,60);
    glLineWidth(8);
    glBegin(GL_LINES);
    glVertex2d(base.x, base.y);
    glVertex2d(elbow.x, elbow.y);
    glVertex2d(elbow.x, elbow.y);
    glVertex2d(paddle.pos.x, paddle.pos.y);
    glEnd();
    glLineWidth(1);

    glColor3ub(200, 0, 0);
    DrawCircle(paddle.pos.x, paddle.pos.y, paddle.radius);
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