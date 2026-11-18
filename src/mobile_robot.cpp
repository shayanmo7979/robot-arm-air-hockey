#include <algorithm>
#include <cmath>

#include "mobile_robot.h"
#include "fssimplewindow.h"

namespace {
constexpr double SQRT2 = 1.4142135623730951;
constexpr double POSITION_EPSILON = 2.0;
constexpr double VELOCITY_EPSILON = 0.5;
constexpr double STRIKE_TRAVEL = 2.5; // mallet radii beyond the contact point
}

MobileRobot::MobileRobot(const TableConfig &cfg)
    : maxWheelSpeed(260.0), maxWheelAccel(800.0), strikeSpeed(260.0), activeSpeedLimit(260.0)
{
    // The opponent occupies the upper half of the table. Keep the complete
    // mallet inside that half, with a small margin from the side/end walls.
    const double margin = cfg.malletRadius;
    minX = margin;
    maxX = cfg.width - margin;
    minY = margin;
    maxY = cfg.height * 0.5 - margin;

    homePos = {cfg.width * 0.5, cfg.height * 0.20};
    pos = homePos;
    vel = {0.0, 0.0};
    target = homePos;

    paddle.pos = pos;
    paddle.vel = vel;
    paddle.radius = cfg.malletRadius;

    SetDifficulty(MEDIUM);
}

void MobileRobot::SetDifficulty(Difficulty level)
{
    // Values are in table-coordinate units per second and per second.
    switch (level) {
    case EASY:
        maxWheelSpeed = 180.0;
        maxWheelAccel = 450.0;
        break;
    case HARD:
        maxWheelSpeed = 340.0;
        maxWheelAccel = 1100.0;
        break;
    case MEDIUM:
    default:
        maxWheelSpeed = 260.0;
        maxWheelAccel = 800.0;
        break;
    }

    if (strikeSpeed > maxWheelSpeed) {
        strikeSpeed = maxWheelSpeed;
    }
    activeSpeedLimit = maxWheelSpeed;
}

void MobileRobot::Reset(void)
{
    pos = homePos;
    vel = {0.0, 0.0};
    target = homePos;
    strikePoint = homePos;
    strikeDir = {0.0, 1.0};
    strikeApproach = homePos;
    strikeEnd = homePos;
    state = HOLD;
    Stop();

    paddle.pos = pos;
    paddle.vel = vel;
}

Vec2 MobileRobot::ClampToWorkspace(Vec2 p) const
{
    p.x = Clamp(p.x, minX, maxX);
    p.y = Clamp(p.y, minY, maxY);
    return p;
}

void MobileRobot::CommandMoveTo(Vec2 newTarget)
{
    target = ClampToWorkspace(newTarget);
    activeSpeedLimit = maxWheelSpeed;
    state = MOVING;
}

void MobileRobot::CommandStrike(Vec2 hitPoint, Vec2 direction, double speed)
{
    strikePoint = ClampToWorkspace(hitPoint);
    strikeDir = Normalize(direction);

    // A zero direction cannot define an approach/strike path.
    if (Length(strikeDir) == 0.0) {
        strikeDir = {0.0, 1.0};
    }

    strikeSpeed = Clamp(std::fabs(speed), 0.0, maxWheelSpeed);
    activeSpeedLimit = maxWheelSpeed;

    // First approach from behind the puck, then drive through the contact
    // point. Both positions are clamped to the robot's legal half.
    const double approachDistance = paddle.radius * 1.5;
    strikeApproach = ClampToWorkspace(
        strikePoint - strikeDir * approachDistance);
    strikeEnd = ClampToWorkspace(
        strikePoint + strikeDir * STRIKE_TRAVEL * paddle.radius);

    target = strikeApproach;
    state = APPROACH;
}

void MobileRobot::VelocityToWheelSpeeds(Vec2 v, double w[4]) const
{
    // X-drive inverse kinematics. This simplified model controls translation
    // only (no chassis rotation), which is sufficient for the mallet.
    const double invSqrt2 = 1.0 / SQRT2;
    w[0] = (v.x - v.y) * invSqrt2; // front-left
    w[1] = (v.x + v.y) * invSqrt2; // front-right
    w[2] = (v.x + v.y) * invSqrt2; // rear-left
    w[3] = (v.x - v.y) * invSqrt2; // rear-right
}

Vec2 MobileRobot::WheelSpeedsToVelocity(const double w[4]) const
{
    const double vx = (w[0] + w[1] + w[2] + w[3]) * SQRT2 * 0.25;
    const double vy = (-w[0] + w[1] + w[2] - w[3]) * SQRT2 * 0.25;
    return {vx, vy};
}

void MobileRobot::Stop(void)
{
    for (double &speed : wheelSpeed) {
        speed = 0.0;
    }
    vel = {0.0, 0.0};
}

void MobileRobot::DriveToward(Vec2 goal, double dt)
{
    if (dt <= 0.0) {
        return;
    }

    goal = ClampToWorkspace(goal);
    Vec2 displacement = goal - pos;
    const double distance = Length(displacement);

    Vec2 desiredVelocity = {0.0, 0.0};
    if (distance > POSITION_EPSILON) {
        const Vec2 direction = displacement / distance;

        // Slow down early enough to stop at the target. The wheel-space
        // acceleration limit is enforced below.
        const double desiredSpeed = std::min(
            activeSpeedLimit,
            std::sqrt(2.0 * maxWheelAccel * distance));
        desiredVelocity = direction * desiredSpeed;
    }

    double desiredWheelSpeed[4];
    VelocityToWheelSpeeds(desiredVelocity, desiredWheelSpeed);

    // Enforce both wheel-speed and wheel-acceleration limits.
    const double maxDelta = maxWheelAccel * dt;
    for (int i = 0; i < 4; ++i) {
        desiredWheelSpeed[i] =
            Clamp(desiredWheelSpeed[i], -maxWheelSpeed, maxWheelSpeed);

        const double delta = Clamp(
            desiredWheelSpeed[i] - wheelSpeed[i], -maxDelta, maxDelta);
        wheelSpeed[i] += delta;
    }

    vel = WheelSpeedsToVelocity(wheelSpeed);
    Vec2 nextPos = pos + vel * dt;

    // Clamp at the workspace boundary and remove velocity directed farther
    // outside the legal region.
    if (nextPos.x < minX || nextPos.x > maxX) {
        nextPos.x = Clamp(nextPos.x, minX, maxX);
        vel.x = 0.0;
    }
    if (nextPos.y < minY || nextPos.y > maxY) {
        nextPos.y = Clamp(nextPos.y, minY, maxY);
        vel.y = 0.0;
    }

    pos = nextPos;

    // Re-synchronize wheel state after a boundary clamp.
    if (vel.x == 0.0 || vel.y == 0.0) {
        VelocityToWheelSpeeds(vel, wheelSpeed);
    }
}

double MobileRobot::TimeToReach(Vec2 point) const
{
    point = ClampToWorkspace(point);
    const double distance = Length(point - pos);
    if (distance <= POSITION_EPSILON) {
        return 0.0;
    }

    // Conservative triangular/trapezoidal estimate. The translational
    // limits are approximated by the configured wheel limits.
    const double acceleration = std::max(1.0, maxWheelAccel);
    const double topSpeed = std::max(1.0, maxWheelSpeed);
    const double rampDistance = topSpeed * topSpeed / acceleration;

    if (distance <= rampDistance) {
        return 2.0 * std::sqrt(distance / acceleration);
    }
    return 2.0 * topSpeed / acceleration
         + (distance - rampDistance) / topSpeed;
}

bool MobileRobot::IsCommandDone(void) const
{
    return state == HOLD;
}

void MobileRobot::Update(double dt)
{
    if (dt <= 0.0) {
        return;
    }

    switch (state) {
    case HOLD:
        // Keep the robot stationary while waiting for the next command.
        DriveToward(pos, dt);
        break;

    case MOVING:
        DriveToward(target, dt);
        if (Length(target - pos) <= POSITION_EPSILON &&
            Length(vel) <= VELOCITY_EPSILON) {
            pos = target;
            Stop();
            state = HOLD;
        }
        break;

    case APPROACH:
        DriveToward(strikeApproach, dt);
        if (Length(strikeApproach - pos) <= POSITION_EPSILON &&
            Length(vel) <= VELOCITY_EPSILON) {
            pos = strikeApproach;
            Stop();
            target = strikeEnd;
            activeSpeedLimit = strikeSpeed;
            state = STRIKE;
        }
        break;

    case STRIKE:
        DriveToward(strikeEnd, dt);
        if (Length(strikeEnd - pos) <= POSITION_EPSILON &&
            Length(vel) <= VELOCITY_EPSILON) {
            pos = strikeEnd;
            Stop();
            state = HOLD;
        }
        break;
    }

    paddle.pos = pos;
    paddle.vel = vel;
}

MalletState MobileRobot::GetPaddleState(void) const
{
    return paddle;
}

void MobileRobot::GetWheelSpeeds(double out[4]) const
{
    if (out == nullptr) {
        return;
    }
    for (int i = 0; i < 4; ++i) {
        out[i] = wheelSpeed[i];
    }
}

void MobileRobot::Draw(void) const
{
    // Robot chassis.
    const double halfWidth = 42.0;
    const double halfHeight = 34.0;

    glColor3ub(45, 55, 70);
    DrawRoundedRect(pos.x - halfWidth, pos.y - halfHeight,
                    pos.x + halfWidth, pos.y + halfHeight, true, 9.0);

    // Four omni wheels around the chassis.
    const double wheelX = 30.0;
    const double wheelY = 27.0;
    const Vec2 wheelCenters[4] = {
        {pos.x - wheelX, pos.y - wheelY},
        {pos.x + wheelX, pos.y - wheelY},
        {pos.x - wheelX, pos.y + wheelY},
        {pos.x + wheelX, pos.y + wheelY}
    };

    for (const Vec2 &wheel : wheelCenters) {
        glColor3ub(25, 25, 25);
        DrawCircle(wheel.x, wheel.y, 12.0);
        glColor3ub(150, 160, 170);
        DrawCircle(wheel.x, wheel.y, 5.0);
    }

    // Mallet is drawn on top of the chassis to show the physics contact area.
    glColor3ub(20, 130, 220);
    DrawCircle(paddle.pos.x, paddle.pos.y, paddle.radius);
    glColor3ub(220, 240, 255);
    DrawCircle(paddle.pos.x, paddle.pos.y, paddle.radius * 0.45);
}
