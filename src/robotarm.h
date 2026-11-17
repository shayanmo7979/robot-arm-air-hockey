#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include "shared.h"
#include "robot.h"

class RobotArm : public Robot {
private:
    enum ArmState {HOLD, MOVING, APPROACH, STRIKE, RECOVER};
    ArmState state = HOLD;
    Vec2 base;
    double L1, L2;
    double theta1, theta2;
    double target1, target2;
    double minTheta1, minTheta2;
    double maxTheta1, maxTheta2;
    double maxJointSpeed;
    Vec2 strikePoint, strikeDirection;
    double strikeSpeed;
    MalletState paddle;
    Vec2 ForwardKinematics(double t1, double t2) const;
    bool InverseKinematics(Vec2 point, double &t1, double &t2) const;
    void MoveJointsToward(double dt);

public:
    RobotArm(const TableConfig &cfg);
    void SetDifficulty(Difficulty level) override;
    void Reset(void) override;
    void CommandMoveTo(Vec2 target) override;
    void CommandStrike(Vec2 hitPoint, Vec2 direction, double speed) override;
    double TimeToReach(Vec2 point) const override;
    bool IsCommandDone(void) const override;
    void Update(double dt) override;
    MalletState GetPaddleState(void) const override;
    void Draw(void) const override;

    // Debug helper functions
    void SetJointAngles(double t1, double t2);
    void GetJointAngles(double &t1, double &t2) const;
};

#endif