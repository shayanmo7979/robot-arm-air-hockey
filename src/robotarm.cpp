#include <math.h>

#include "robotarm.h"
#include "fssimplewindow.h"

RobotArm::RobotArm(const TableConfig &cfg) {
    base = {cfg.width/2, 0};
    L1 = 0.5 * Length(Vec2{cfg.width/2, cfg.height/2});
    L2 = 0.5 * Length(Vec2{cfg.width/2, cfg.height/2});
    minTheta1 = 0;
    maxTheta1 = PI;
    minTheta2 = -150*(PI/180);
    maxTheta2 = 150*(PI/180);
    maxJointSpeed = PI;
    theta1 = PI/4;
    theta2 = PI/2;
    target1 = theta1;
    target2 = theta2;
    paddle.pos = ForwardKinematics(theta1, theta2);
    paddle.vel = {0.0, 0.0};
    paddle.radius = cfg.malletRadius;

}

void RobotArm::SetDifficulty(Difficulty level) {
    return;
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
    return;
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
    return false;
}

void RobotArm::MoveJointsToward(double dt) {
    return;
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