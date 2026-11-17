#ifndef ROBOT_H
#define ROBOT_H

#include "shared.h"

class Robot {
public: 
    virtual ~Robot() = default;
    virtual void SetDifficulty(Difficulty level) = 0;
    virtual void Reset(void) = 0;
    virtual void CommandMoveTo(Vec2 target) = 0;
    virtual void CommandStrike(Vec2 hitPoint, Vec2 direction, double speed) = 0;
    virtual double TimeToReach(Vec2 point) const = 0;
    virtual bool IsCommandDone(void) const = 0;
    virtual void Update(double dt) = 0;
    virtual MalletState GetPaddleState(void) const = 0;
    virtual void Draw(void) const = 0;
};

#endif