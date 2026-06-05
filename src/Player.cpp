#include "../Player.h"

#include <cmath>



namespace

{

    constexpr float PI = 3.1415926535f;



    float Clamp(float value, float minValue, float maxValue)

    {

        if (value < minValue) return minValue;

        if (value > maxValue) return maxValue;

        return value;

    }



    float Length(const Vec2& v)

    {

        return std::sqrt(v.x * v.x + v.y * v.y);

    }



    Vec2 Normalize(const Vec2& v)

    {

        float len = Length(v);

        if (len <= 0.0001f)

        {

            return Vec2(0.0f, -1.0f);

        }

        return v / len;

    }

}



Player::Player()

{

    spawnPos = Vec2(-80.0f, 668.0f);

    pos = spawnPos;

    vel = Vec2(0.0f, 0.0f);



    radius = 26.0f;



    shoulderOffset = Vec2(10.0f, -20.0f);

    upperArmLength = 48.0f;

    forearmLength = 97.0f;



    shoulderAngle = -0.6f;

    elbowAngle = 0.9f;

    hammerAngle = shoulderAngle + elbowAngle;

    prevHammerAngle = hammerAngle;

    hammerAngularVelocity = 0.0f;



    hammerTipRadius = 11.0f;



    prevHammerTip = HammerTip();

    prevElbow = ElbowWorld();

}



void Player::Reset()

{

    pos = spawnPos;

    vel = Vec2(0.0f, 0.0f);



    shoulderAngle = -0.6f;

    elbowAngle = 0.9f;

    hammerAngle = shoulderAngle + elbowAngle;

    prevHammerAngle = hammerAngle;

    hammerAngularVelocity = 0.0f;

    prevHammerTip = HammerTip();

    prevElbow = ElbowWorld();

}



void Player::Update(float dt)

{

    (void)dt;

}



Vec2 Player::ShoulderWorld() const

{

    return pos + shoulderOffset;

}



Vec2 Player::ElbowWorld() const

{

    Vec2 upperDir(std::cos(shoulderAngle), std::sin(shoulderAngle));

    return ShoulderWorld() + upperDir * upperArmLength;

}



Vec2 Player::HammerDir() const

{

    return Vec2(std::cos(hammerAngle), std::sin(hammerAngle));

}



Vec2 Player::HammerTip() const

{

    return ElbowWorld() + HammerDir() * forearmLength;

}



void Player::SolveArmIK(const Vec2& targetWorld, const Vec2& poleHint)

{

    Vec2 shoulder = ShoulderWorld();

    Vec2 toTarget = targetWorld - shoulder;

    float dist = Length(toTarget);



    const float minDist = std::abs(upperArmLength - forearmLength) + 2.0f;

    const float maxDist = upperArmLength + forearmLength - 2.0f;

    dist = Clamp(dist, minDist, maxDist);



    if (dist <= 0.001f)

    {

        return;

    }



    float cosElbowInterior =

        (upperArmLength * upperArmLength + forearmLength * forearmLength - dist * dist) /

        (2.0f * upperArmLength * forearmLength);

    cosElbowInterior = Clamp(cosElbowInterior, -1.0f, 1.0f);



    float elbowInterior = std::acos(cosElbowInterior);

    elbowAngle = PI - elbowInterior;



    float cosShoulderOffset =

        (upperArmLength * upperArmLength + dist * dist - forearmLength * forearmLength) /

        (2.0f * upperArmLength * dist);

    cosShoulderOffset = Clamp(cosShoulderOffset, -1.0f, 1.0f);



    float shoulderOffsetAngle = std::acos(cosShoulderOffset);

    float baseAngle = std::atan2(toTarget.y, toTarget.x);



    Vec2 targetDir = Normalize(toTarget);

    Vec2 poleDir = poleHint - shoulder;

    float cross = targetDir.x * poleDir.y - targetDir.y * poleDir.x;



    if (cross >= 0.0f)

    {

        shoulderAngle = baseAngle + shoulderOffsetAngle;

    }

    else

    {

        shoulderAngle = baseAngle - shoulderOffsetAngle;

    }



    hammerAngle = shoulderAngle + elbowAngle;

}



Vec2 Player::BodyPosFromPinnedTip(const Vec2& attachPoint, const Vec2& mouseWorld) const

{

    Vec2 toMouse = mouseWorld - attachPoint;

    if (Length(toMouse) <= 0.001f)

    {

        return pos;

    }



    float forearmAngle = std::atan2(toMouse.y, toMouse.x);

    Vec2 forearmDir(std::cos(forearmAngle), std::sin(forearmAngle));

    Vec2 elbow = attachPoint - forearmDir * forearmLength;



    Vec2 currentShoulder = pos + shoulderOffset;

    Vec2 toShoulder = currentShoulder - elbow;

    float dist = Length(toShoulder);



    Vec2 upperDir = (dist > 0.001f) ? (toShoulder / dist) : Vec2(0.0f, -1.0f);

    Vec2 newShoulder = elbow + upperDir * upperArmLength;



    return newShoulder - shoulderOffset;

}



void Player::SolveArmFromPinnedTip(const Vec2& attachPoint, const Vec2& mouseWorld)

{

    Vec2 toMouse = mouseWorld - attachPoint;

    if (Length(toMouse) > 0.001f)

    {

        hammerAngle = std::atan2(toMouse.y, toMouse.x);

    }



    Vec2 forearmDir = HammerDir();

    Vec2 elbow = attachPoint - forearmDir * forearmLength;



    Vec2 shoulder = ShoulderWorld();

    Vec2 toElbow = elbow - shoulder;

    float dist = Length(toElbow);



    if (dist > 0.001f)

    {

        shoulderAngle = std::atan2(toElbow.y, toElbow.x);



        float cosElbowInterior =

            (upperArmLength * upperArmLength + forearmLength * forearmLength - dist * dist) /

            (2.0f * upperArmLength * forearmLength);

        cosElbowInterior = Clamp(cosElbowInterior, -1.0f, 1.0f);

        elbowAngle = PI - std::acos(cosElbowInterior);

    }

}
