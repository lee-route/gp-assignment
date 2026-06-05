#pragma once

#include "Vec2.h"

class Player
{
public:
    Vec2 pos;
    Vec2 vel;
    Vec2 spawnPos;

    float radius;

    float hammerAngle;
    float targetHammerAngle;
    float prevHammerAngle;
    float hammerAngularVelocity;

    float hammerLength;
    float hammerTipRadius;

    Vec2 prevHammerTip;
    Vec2 currentHammerTip;

public:
    Player();

    void Reset();

    Vec2 HammerDir() const;
    Vec2 HammerTip() const;
};