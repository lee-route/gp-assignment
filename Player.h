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
    float hammerLength;
    float hammerTipRadius;

    bool hammerAttached;
    Vec2 hammerAttachPoint;

public:
    Player();

    void Reset();
    void Update(float dt);

    Vec2 HammerDir() const;
    Vec2 HammerTip() const;
};