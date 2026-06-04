#include "Player.h"
#include <cmath>

Player::Player()
{
    spawnPos = Vec2(90.0f, 640.0f);
    pos = spawnPos;
    vel = Vec2(0.0f, 0.0f);

    radius = 24.0f;

    hammerAngle = 0.0f;
    hammerLength = 140.0f;
    hammerTipRadius = 10.0f;

    hammerAttached = false;
    hammerAttachPoint = Vec2(0.0f, 0.0f);
}

void Player::Reset()
{
    pos = spawnPos;
    vel = Vec2(0.0f, 0.0f);
    hammerAngle = 0.0f;
    hammerAttached = false;
}

void Player::Update(float dt)
{
    (void)dt;
}

Vec2 Player::HammerDir() const
{
    return Vec2(std::cos(hammerAngle), std::sin(hammerAngle));
}

Vec2 Player::HammerTip() const
{
    return pos + HammerDir() * hammerLength;
}