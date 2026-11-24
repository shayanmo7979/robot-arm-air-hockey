#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include "shared.h"
#include "robot.h"

class RobotArm : public Robot {
private:
    enum ArmState {HOLD, JOINT_MOVE, MOVING, APPROACH, STRIKE, RECOVER};
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
    TableConfig cfg;
    Vec2 homePoint;
    double home1, home2;
    Vec2 carrot, goal;
    Vec2 approachPoint, strikeEnd;
    Vec2 ForwardKinematics(double t1, double t2) const;
    void MoveJointsToward(double dt);
    Vec2 ClampToWorkspace(Vec2 p) const;
    bool AtTarget(void) const;
    bool StepAlongLine(Vec2 &point, Vec2 end, double a1, double a2, double maxStep, double &n1, double &n2) const;
    bool FollowLine(Vec2 end, double dt);

public:
    RobotArm(const TableConfig &config);
    void SetDifficulty(Difficulty level) override;
    void Reset(void) override;
    void CommandMoveTo(Vec2 target) override;
    void CommandStrike(Vec2 hitPoint, Vec2 direction, double speed) override;
    double TimeToReach(Vec2 point) const override;
    bool IsCommandDone(void) const override;
    void Update(double dt) override;
    MalletState GetPaddleState(void) const override;
    bool InverseKinematics(Vec2 point, double &t1, double &t2) const;
    void Draw(void) const override;

    // Debug helper functions
    void SetJointAngles(double t1, double t2);
    void SetTargetAngles(double t1, double t2);
    void GetJointAngles(double &t1, double &t2) const;
    const char *GetStateName(void) const;
    Vec2 GetGoal(void) const;
    void GetStrikeLine(Vec2 &start, Vec2 &end) const;
};

#endif