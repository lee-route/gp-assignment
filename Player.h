#pragma once

#include "Vec2.h"

class Player
{
public:
    Vec2 pos;
    Vec2 vel;
    Vec2 spawnPos;

    float radius;

    // Two-segment arm (shoulder -> elbow -> hammer tip)
    Vec2 shoulderOffset;
    float upperArmLength;
    float forearmLength;

    float shoulderAngle;
    float elbowAngle;
    float hammerAngle;
    float prevHammerAngle;
    float hammerAngularVelocity;

    float hammerTipRadius;

    Vec2 prevHammerTip;
    Vec2 prevElbow;

    // Hammer tip pinned to terrain while gripping
    bool hammerAttached;
    Vec2 hammerAttachPoint;

public:
    Player();

    void Reset();
    void Update(float dt);

    Vec2 ShoulderWorld() const;
    Vec2 ElbowWorld() const;
    Vec2 HammerDir() const;
    Vec2 HammerTip() const;

    // Reach toward mouse with a bendable elbow
    void SolveArmIK(const Vec2& targetWorld, const Vec2& poleHint);

    // Tip locked on terrain; mouse rotates the handle around the contact point
    void SolveArmFromAttachedTip(const Vec2& mouseWorld);
};