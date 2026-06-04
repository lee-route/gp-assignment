#pragma once

struct Vec2
{
    float x;
    float y;

    Vec2()
    {
        x = 0.0f;
        y = 0.0f;
    }

    Vec2(float _x, float _y)
    {
        x = _x;
        y = _y;
    }
};

inline Vec2 operator+(const Vec2& a, const Vec2& b)
{
    return Vec2(a.x + b.x, a.y + b.y);
}

inline Vec2 operator-(const Vec2& a, const Vec2& b)
{
    return Vec2(a.x - b.x, a.y - b.y);
}

inline Vec2 operator*(const Vec2& a, float s)
{
    return Vec2(a.x * s, a.y * s);
}

inline Vec2 operator/(const Vec2& a, float s)
{
    return Vec2(a.x / s, a.y / s);
}

inline Vec2& operator+=(Vec2& a, const Vec2& b)
{
    a.x += b.x;
    a.y += b.y;
    return a;
}

inline Vec2& operator-=(Vec2& a, const Vec2& b)
{
    a.x -= b.x;
    a.y -= b.y;
    return a;
}