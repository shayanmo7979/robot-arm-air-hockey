#include "robotarm.h"

RobotArm::RobotArm() {
    paddle.pos = {0.0, 0.0};
    paddle.vel = {0.0, 0.0};
    paddle.radius = 0.0;
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
    return true;
}

void RobotArm::Update(double dt) {
    return;
}

MalletState RobotArm::GetPaddleState(void) const {
    return paddle;
}

void RobotArm::Draw(void) const {
    return;
}

Vec2 RobotArm::ForwardKinematics(double t1, double t2) const {
    return base;
}

bool RobotArm::InverseKinematics(Vec2 point, double &t1, double &t2) const {
    return true;
}

void RobotArm::MoveJointsToward(double dt) {
    return;
}