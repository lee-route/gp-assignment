#pragma once

#include "Vec2.h"

class Player
{
public:
    Vec2 pos;
    Vec2 vel;
    Vec2 spawnPos;

    float radius;

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

public:
    Player();

    void Reset();
    void Update(float dt);

    Vec2 ShoulderWorld() const;
    Vec2 ElbowWorld() const;
    Vec2 HammerDir() const;
    Vec2 HammerTip() const;

    void SolveArmIK(const Vec2& targetWorld, const Vec2& poleHint);
    void SolveArmFromPinnedTip(const Vec2& attachPoint, const Vec2& mouseWorld);
    Vec2 BodyPosFromPinnedTip(const Vec2& attachPoint, const Vec2& mouseWorld) const;
};