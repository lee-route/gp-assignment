#include "../Player.h"

#include <cmath>

Player::Player()
{
    spawnPos = Vec2(120.0f, 640.0f);
    pos = spawnPos;
    vel = Vec2(0.0f, 0.0f);

    radius = 28.0f;

    hammerAngle = -0.35f;
    targetHammerAngle = hammerAngle;
    prevHammerAngle = hammerAngle;
    hammerAngularVelocity = 0.0f;

    hammerLength = 145.0f;
    hammerTipRadius = 6.0f;

    prevHammerTip = HammerTip();
    currentHammerTip = HammerTip();
}

void Player::Reset()
{
    pos = spawnPos;
    vel = Vec2(0.0f, 0.0f);

    hammerAngle = -0.35f;
    targetHammerAngle = hammerAngle;
    prevHammerAngle = hammerAngle;
    hammerAngularVelocity = 0.0f;

    prevHammerTip = HammerTip();
    currentHammerTip = HammerTip();
}

Vec2 Player::HammerDir() const
{
    return Vec2(std::cos(hammerAngle), std::sin(hammerAngle));
}

Vec2 Player::HammerTip() const
{
    return pos + HammerDir() * hammerLength;
}