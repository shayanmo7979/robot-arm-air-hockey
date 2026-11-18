#pragma once

#include "robot.h"

// Four-wheel X-drive omnidirectional robot.
// Coordinates use the same table coordinate system as TableConfig:
// (0, 0) is the top-left corner and the robot stays on the top half.
class MobileRobot : public Robot {
private:
    enum RobotState { HOLD, MOVING, APPROACH, STRIKE };

    RobotState state = HOLD;

    Vec2 pos;       // Mallet center / robot position
    Vec2 vel;
    Vec2 homePos;
    double minX, maxX, minY, maxY;

    // Wheel order: front-left, front-right, rear-left, rear-right.
    double wheelSpeed[4] = {0.0, 0.0, 0.0, 0.0};
    double maxWheelSpeed;
    double maxWheelAccel;

    Vec2 target;
    Vec2 strikePoint;
    Vec2 strikeDir;
    Vec2 strikeApproach;
    Vec2 strikeEnd;
    double strikeSpeed;
    double activeSpeedLimit;

    MalletState paddle;

    void VelocityToWheelSpeeds(Vec2 v, double w[4]) const;
    Vec2 WheelSpeedsToVelocity(const double w[4]) const;
    void DriveToward(Vec2 goal, double dt);
    Vec2 ClampToWorkspace(Vec2 p) const;
    void Stop(void);

public:
    explicit MobileRobot(const TableConfig &cfg);

    void SetDifficulty(Difficulty level) override;
    void Reset(void) override;
    void CommandMoveTo(Vec2 target) override;
    void CommandStrike(Vec2 hitPoint, Vec2 direction, double speed) override;
    double TimeToReach(Vec2 point) const override;
    bool IsCommandDone(void) const override;
    void Update(double dt) override;
    MalletState GetPaddleState(void) const override;
    void Draw(void) const override;

    // Debug/test accessors. These do not change the Robot interface.
    Vec2 GetPosition(void) const { return pos; }
    Vec2 GetVelocity(void) const { return vel; }
    void GetWheelSpeeds(double out[4]) const;
};
