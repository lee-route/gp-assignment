#include <SDL.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include "../Player.h"

// ------------------------------------------------------------
// Boo Hammer Climb Prototype (SDL2)
// ------------------------------------------------------------
// Concept
// - Getting Over It style 2D climbing game
// - Player moves mainly by rotating a hammer / pickaxe with the mouse
// - No left-click pushing required
// - Slow mouse movement creates gentle pushing
// - Fast mouse movement creates strong physical reaction
//
// Technical Features
// - Fixed timestep physics
// - Gravity and damping
// - Circle body physics
// - Circle vs AABB collision detection
// - Collision normal based resolution
// - Hammer tip contact detection
// - Mouse angular velocity based reaction force
// - Camera following player
//
// Controls
// - Mouse: rotate hammer around the player
// - R: reset player
// - ESC / close button: quit
// ------------------------------------------------------------

constexpr int WINDOW_WIDTH = 1280;
constexpr int WINDOW_HEIGHT = 720;
constexpr float FIXED_DT = 1.0f / 120.0f;
constexpr int TARGET_FPS = 60;
constexpr float PI = 3.1415926535f;

static float Dot(const Vec2& a, const Vec2& b)
{
    return a.x * b.x + a.y * b.y;
}

static float Length(const Vec2& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y);
}

static Vec2 Normalize(const Vec2& v)
{
    float len = Length(v);
    if (len <= 0.0001f)
    {
        return Vec2(0.0f, 0.0f);
    }

    return v / len;
}

static float Clamp(float value, float minValue, float maxValue)
{
    return std::max(minValue, std::min(value, maxValue));
}

static float WrapAngleDiff(float current, float previous)
{
    float diff = current - previous;

    while (diff > PI)
    {
        diff -= 2.0f * PI;
    }

    while (diff < -PI)
    {
        diff += 2.0f * PI;
    }

    return diff;
}

struct Circle
{
    Vec2 center;
    float radius;
};

struct RectF
{
    float x;
    float y;
    float w;
    float h;
};

struct Camera
{
    float x;
    float y;
};

struct ContactInfo
{
    bool hit = false;
    Vec2 normal = Vec2(0.0f, -1.0f);
    float penetration = 0.0f;
    Vec2 point;
};

static int ScreenX(float worldX, const Camera& camera)
{
    return static_cast<int>(worldX - camera.x);
}

static int ScreenY(float worldY, const Camera& camera)
{
    return static_cast<int>(worldY - camera.y);
}

static Vec2 WorldToScreen(const Vec2& world, const Camera& camera)
{
    return Vec2(world.x - camera.x, world.y - camera.y);
}

static void DrawCircle(SDL_Renderer* renderer, int cx, int cy, int radius)
{
    for (int w = -radius; w <= radius; ++w)
    {
        for (int h = -radius; h <= radius; ++h)
        {
            if (w * w + h * h <= radius * radius)
            {
                SDL_RenderDrawPoint(renderer, cx + w, cy + h);
            }
        }
    }
}

static void DrawCircleOutline(SDL_Renderer* renderer, int cx, int cy, int radius)
{
    const int segments = 64;

    for (int i = 0; i < segments; ++i)
    {
        float a0 = (2.0f * PI * i) / segments;
        float a1 = (2.0f * PI * (i + 1)) / segments;

        int x0 = static_cast<int>(cx + std::cos(a0) * radius);
        int y0 = static_cast<int>(cy + std::sin(a0) * radius);
        int x1 = static_cast<int>(cx + std::cos(a1) * radius);
        int y1 = static_cast<int>(cy + std::sin(a1) * radius);

        SDL_RenderDrawLine(renderer, x0, y0, x1, y1);
    }
}

static void DrawWorldRect(SDL_Renderer* renderer, const RectF& r, const Camera& camera)
{
    SDL_Rect rect{
        ScreenX(r.x, camera),
        ScreenY(r.y, camera),
        static_cast<int>(r.w),
        static_cast<int>(r.h)
    };

    SDL_RenderFillRect(renderer, &rect);
}

static ContactInfo CircleVsRect(const Circle& c, const RectF& r)
{
    ContactInfo info;

    float closestX = Clamp(c.center.x, r.x, r.x + r.w);
    float closestY = Clamp(c.center.y, r.y, r.y + r.h);

    Vec2 closest(closestX, closestY);
    Vec2 delta = c.center - closest;

    float distSq = Dot(delta, delta);

    if (distSq < c.radius * c.radius)
    {
        float dist = std::sqrt(std::max(0.00001f, distSq));

        info.hit = true;
        info.point = closest;

        if (dist > 0.0001f)
        {
            info.normal = delta / dist;
            info.penetration = c.radius - dist;
        }
        else
        {
            float leftPen = std::abs(c.center.x - r.x);
            float rightPen = std::abs((r.x + r.w) - c.center.x);
            float topPen = std::abs(c.center.y - r.y);
            float bottomPen = std::abs((r.y + r.h) - c.center.y);

            float minPen = leftPen;
            info.normal = Vec2(-1.0f, 0.0f);

            if (rightPen < minPen)
            {
                minPen = rightPen;
                info.normal = Vec2(1.0f, 0.0f);
            }

            if (topPen < minPen)
            {
                minPen = topPen;
                info.normal = Vec2(0.0f, -1.0f);
            }

            if (bottomPen < minPen)
            {
                minPen = bottomPen;
                info.normal = Vec2(0.0f, 1.0f);
            }

            info.penetration = c.radius;
        }
    }

    return info;
}

static ContactInfo HammerTipVsWorld(const Circle& tip, const std::vector<RectF>& world)
{
    ContactInfo best;
    float bestPen = -1.0f;

    for (const RectF& rect : world)
    {
        ContactInfo hit = CircleVsRect(tip, rect);

        if (hit.hit && hit.penetration > bestPen)
        {
            best = hit;
            bestPen = hit.penetration;
        }
    }

    return best;
}

static void ResolveBodyWorld(Player& player, const std::vector<RectF>& world)
{
    constexpr int ITERATIONS = 8;

    for (int i = 0; i < ITERATIONS; ++i)
    {
        bool anyHit = false;

        for (const RectF& rect : world)
        {
            Circle body{ player.pos, player.radius };
            ContactInfo hit = CircleVsRect(body, rect);

            if (hit.hit)
            {
                anyHit = true;

                // Push body out of terrain
                player.pos += hit.normal * hit.penetration;

                // Remove velocity toward the surface
                float vn = Dot(player.vel, hit.normal);
                if (vn < 0.0f)
                {
                    player.vel -= hit.normal * vn;
                }

                if (std::abs(hit.normal.y) > 0.7f)
                {
                    player.vel.x *= 0.985f;
                }

                if (std::abs(hit.normal.x) > 0.7f)
                {
                    player.vel.y *= 0.99f;
                }
            }
        }

        if (!anyHit)
        {
            break;
        }
    }
}

static void BuildLevel(std::vector<RectF>& world)
{
    world.clear();

    const float thickness = 34.0f;
    const float stepWidth = 130.0f;
    const float stepRise = 48.0f;
    const float overlap = 28.0f;

    // Left boundary wall (extends all the way down to the bottom floor)
    world.push_back({ -260.0f, -2200.0f, 44.0f, 5000.0f });

    // Continuous sloped climb: overlapping steps with no gaps on top
    float x = -220.0f;
    float topY = 700.0f;
    const float slopeStartX = x;
    const float slopeStartY = topY;

    for (int i = 0; i < 28; ++i)
    {
        world.push_back({ x, topY, stepWidth + overlap, thickness });
        x += stepWidth;
        topY -= stepRise;
    }

    const float slopeEndX = x + stepWidth;
    const float slopeEndY = topY;

    // Solid backing mass under the slope so nothing can fall through gaps
    world.push_back({
        slopeStartX - 40.0f,
        slopeEndY - 20.0f,
        (slopeEndX - slopeStartX) + 120.0f,
        (slopeStartY + thickness) - (slopeEndY - 20.0f)
    });

    // Upper ridge and goal plateau
    world.push_back({ x - overlap * 0.5f, topY, 360.0f, thickness + 8.0f });
    world.push_back({ x + 220.0f, topY - 90.0f, 240.0f, thickness });

    // Side ridges for hammer hooks
    world.push_back({ x + 80.0f, topY - 260.0f, 36.0f, 220.0f });
    world.push_back({ x + 360.0f, topY - 180.0f, 36.0f, 180.0f });

    // Wide thick bottom floor
    world.push_back({ -900.0f, 1900.0f, 5200.0f, 700.0f });
}

static void DrawArmSegment(SDL_Renderer* renderer, const Vec2& a, const Vec2& b, const Camera& camera)
{
    Vec2 aScreen = WorldToScreen(a, camera);
    Vec2 bScreen = WorldToScreen(b, camera);

    SDL_RenderDrawLine(
        renderer,
        static_cast<int>(aScreen.x),
        static_cast<int>(aScreen.y),
        static_cast<int>(bScreen.x),
        static_cast<int>(bScreen.y)
    );
}

static void DrawPickaxe(SDL_Renderer* renderer, const Player& player, const Camera& camera, bool contact)
{
    Vec2 shoulder = player.ShoulderWorld();
    Vec2 elbow = player.ElbowWorld();
    Vec2 tip = player.HammerTip();
    Vec2 dir = player.HammerDir();

    Vec2 shoulderScreen = WorldToScreen(shoulder, camera);
    Vec2 elbowScreen = WorldToScreen(elbow, camera);
    Vec2 tipScreen = WorldToScreen(tip, camera);

    // Upper arm + forearm
    SDL_SetRenderDrawColor(renderer, 200, 175, 140, 255);
    DrawArmSegment(renderer, shoulder, elbow, camera);

    SDL_SetRenderDrawColor(renderer, 220, 220, 225, 255);
    DrawArmSegment(renderer, elbow, tip, camera);

    // Elbow joint
    SDL_SetRenderDrawColor(renderer, 180, 150, 110, 255);
    DrawCircle(
        renderer,
        static_cast<int>(elbowScreen.x),
        static_cast<int>(elbowScreen.y),
        5
    );

    // Shoulder joint
    DrawCircle(
        renderer,
        static_cast<int>(shoulderScreen.x),
        static_cast<int>(shoulderScreen.y),
        4
    );

    // Pickaxe head
    Vec2 normal(-dir.y, dir.x);
    Vec2 headCenter = tip - dir * 18.0f;

    Vec2 leftHead = headCenter + normal * 28.0f;
    Vec2 rightHead = headCenter - normal * 28.0f;

    Vec2 leftScreen = WorldToScreen(leftHead, camera);
    Vec2 rightScreen = WorldToScreen(rightHead, camera);
    Vec2 headScreen = WorldToScreen(headCenter, camera);

    if (contact)
    {
        SDL_SetRenderDrawColor(renderer, 255, 90, 90, 255);
    }
    else
    {
        SDL_SetRenderDrawColor(renderer, 120, 200, 255, 255);
    }

    SDL_RenderDrawLine(
        renderer,
        static_cast<int>(leftScreen.x),
        static_cast<int>(leftScreen.y),
        static_cast<int>(rightScreen.x),
        static_cast<int>(rightScreen.y)
    );

    SDL_RenderDrawLine(
        renderer,
        static_cast<int>(headScreen.x),
        static_cast<int>(headScreen.y),
        static_cast<int>(tipScreen.x),
        static_cast<int>(tipScreen.y)
    );

    DrawCircle(
        renderer,
        static_cast<int>(tipScreen.x),
        static_cast<int>(tipScreen.y),
        static_cast<int>(player.hammerTipRadius)
    );
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
    {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Boo Hammer Climb Prototype",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN
    );

    if (!window)
    {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(
        window,
        -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );

    if (!renderer)
    {
        std::cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << "\n";
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    bool running = true;

    Player player;

    std::vector<RectF> world;
    BuildLevel(world);

    Camera camera{ 0.0f, 0.0f };

    const float gravity = 3200.0f;
    const float airDamping = 0.9992f;
    const float maxSpeed = 1800.0f;

    const float pinEnterSwingSpeed = 160.0f;
    const float pinExitSwingSpeed = 520.0f;
    const float impactSwingSpeed = 520.0f;
    const float impactImpulseScale = 2.2f;
    const float maxImpactImpulse = 750.0f;

    Uint64 previousCounter = SDL_GetPerformanceCounter();
    double accumulator = 0.0;

    while (running)
    {
        Uint64 currentCounter = SDL_GetPerformanceCounter();
        double deltaSeconds =
            static_cast<double>(currentCounter - previousCounter) /
            static_cast<double>(SDL_GetPerformanceFrequency());

        previousCounter = currentCounter;

        if (deltaSeconds > 0.25)
        {
            deltaSeconds = 0.25;
        }

        accumulator += deltaSeconds;

        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }

            if (event.type == SDL_KEYDOWN)
            {
                if (event.key.keysym.sym == SDLK_ESCAPE)
                {
                    running = false;
                }

                if (event.key.keysym.sym == SDLK_r)
                {
                    player.Reset();
                }
            }
        }

        // Camera follows the player. It is updated before mouse-world conversion
        // so that mouse direction matches the current camera view.
        camera.x = player.pos.x - WINDOW_WIDTH * 0.42f;
        camera.y = player.pos.y - WINDOW_HEIGHT * 0.58f;

        int mouseX = 0;
        int mouseY = 0;
        SDL_GetMouseState(&mouseX, &mouseY);

        Vec2 mouseWorld(
            static_cast<float>(mouseX) + camera.x,
            static_cast<float>(mouseY) + camera.y
        );

        while (accumulator >= FIXED_DT)
        {
            player.prevHammerAngle = player.hammerAngle;

            Vec2 prevBodyPos = player.pos;
            Vec2 prevTip = player.prevHammerTip;

            if (!player.hammerPinned)
            {
                player.vel.y += gravity * FIXED_DT;
                player.vel = player.vel * airDamping;
                player.SolveArmIK(mouseWorld, mouseWorld);
            }

            Vec2 currentTip = player.HammerTip();
            Circle tipCollider{ currentTip, player.hammerTipRadius };
            ContactInfo tipHit = HammerTipVsWorld(tipCollider, world);

            Vec2 rawTipVelocity = (currentTip - prevTip) / FIXED_DT;
            Vec2 relativeTipVelocity = rawTipVelocity - player.vel;
            float swingSpeed = Length(relativeTipVelocity);

            if (player.hammerPinned)
            {
                ContactInfo pinHit = HammerTipVsWorld(
                    Circle{ player.pinnedAttachPoint, player.hammerTipRadius },
                    world
                );

                if (pinHit.hit && swingSpeed < pinExitSwingSpeed)
                {
                    player.pinnedAttachPoint = pinHit.point + pinHit.normal * (player.hammerTipRadius * 0.2f);
                    player.SolveArmFromPinnedTip(player.pinnedAttachPoint, mouseWorld);
                    player.pos = player.BodyPosFromPinnedTip(player.pinnedAttachPoint, mouseWorld);
                    player.vel = (player.pos - prevBodyPos) / FIXED_DT;
                }
                else
                {
                    player.hammerPinned = false;
                    player.SolveArmIK(mouseWorld, mouseWorld);
                    player.pos += player.vel * FIXED_DT;
                }
            }
            else
            {
                if (tipHit.hit)
                {
                    Vec2 attachPoint = tipHit.point + tipHit.normal * (player.hammerTipRadius * 0.2f);
                    float approachSpeed = Dot(relativeTipVelocity, tipHit.normal * -1.0f);

                    if (swingSpeed < pinEnterSwingSpeed || approachSpeed < 220.0f)
                    {
                        player.hammerPinned = true;
                        player.pinnedAttachPoint = attachPoint;
                        player.SolveArmFromPinnedTip(attachPoint, mouseWorld);
                        player.pos = player.BodyPosFromPinnedTip(attachPoint, mouseWorld);
                        player.vel = (player.pos - prevBodyPos) / FIXED_DT;
                    }
                    else if (swingSpeed >= impactSwingSpeed && approachSpeed > 180.0f)
                    {
                        Vec2 impulseDir = Normalize(relativeTipVelocity * -1.0f);
                        float impulse = Clamp((swingSpeed - impactSwingSpeed) * impactImpulseScale, 0.0f, maxImpactImpulse);
                        player.vel += impulseDir * impulse;
                        player.pos += player.vel * FIXED_DT;
                    }
                    else
                    {
                        player.pos += player.vel * FIXED_DT;
                    }
                }
                else
                {
                    player.pos += player.vel * FIXED_DT;
                }
            }

            ResolveBodyWorld(player, world);

            if (player.hammerPinned)
            {
                player.SolveArmFromPinnedTip(player.pinnedAttachPoint, mouseWorld);
            }

            float speed = Length(player.vel);
            if (speed > maxSpeed)
            {
                player.vel = Normalize(player.vel) * maxSpeed;
            }

            float angleDiff = WrapAngleDiff(player.hammerAngle, player.prevHammerAngle);
            player.hammerAngularVelocity = angleDiff / FIXED_DT;

            player.prevHammerTip = player.HammerTip();
            player.prevElbow = player.ElbowWorld();

            accumulator -= FIXED_DT;
        }

        // Rendering
        SDL_SetRenderDrawColor(renderer, 18, 18, 26, 255);
        SDL_RenderClear(renderer);

        // Draw background guide lines
        SDL_SetRenderDrawColor(renderer, 35, 35, 48, 255);
        for (int y = -1200; y <= 800; y += 120)
        {
            SDL_RenderDrawLine(
                renderer,
                0,
                ScreenY(static_cast<float>(y), camera),
                WINDOW_WIDTH,
                ScreenY(static_cast<float>(y), camera)
            );
        }

        // Draw level
        SDL_SetRenderDrawColor(renderer, 110, 110, 130, 255);
        for (const RectF& rect : world)
        {
            DrawWorldRect(renderer, rect, camera);
        }

        // Draw goal marker at climb summit
        SDL_SetRenderDrawColor(renderer, 255, 220, 80, 255);
        SDL_Rect goalRect{
            ScreenX(3380.0f, camera),
            ScreenY(-670.0f, camera),
            100,
            45
        };
        SDL_RenderFillRect(renderer, &goalRect);

        // Draw pickaxe / hammer
        DrawPickaxe(renderer, player, camera, player.hammerPinned);

        // Draw player body. This will later be replaced by Boo sprite.
        Vec2 playerScreen = WorldToScreen(player.pos, camera);

        SDL_SetRenderDrawColor(renderer, 214, 161, 90, 255);
        DrawCircle(
            renderer,
            static_cast<int>(playerScreen.x),
            static_cast<int>(playerScreen.y),
            static_cast<int>(player.radius)
        );

        SDL_SetRenderDrawColor(renderer, 60, 35, 18, 255);
        DrawCircleOutline(
            renderer,
            static_cast<int>(playerScreen.x),
            static_cast<int>(playerScreen.y),
            static_cast<int>(player.radius)
        );

        // Draw simple face
        SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
        DrawCircle(renderer, static_cast<int>(playerScreen.x - 8), static_cast<int>(playerScreen.y - 6), 3);
        DrawCircle(renderer, static_cast<int>(playerScreen.x + 8), static_cast<int>(playerScreen.y - 6), 3);
        SDL_RenderDrawLine(
            renderer,
            static_cast<int>(playerScreen.x - 8),
            static_cast<int>(playerScreen.y + 8),
            static_cast<int>(playerScreen.x + 8),
            static_cast<int>(playerScreen.y + 8)
        );

        // Draw spawn marker
        SDL_SetRenderDrawColor(renderer, 80, 220, 120, 255);
        SDL_Rect spawnRect{
            ScreenX(player.spawnPos.x - 8.0f, camera),
            ScreenY(player.spawnPos.y - 8.0f, camera),
            16,
            16
        };
        SDL_RenderFillRect(renderer, &spawnRect);

        SDL_RenderPresent(renderer);

        // VSync is enabled, but this prevents a tight loop if VSync is ignored.
        SDL_Delay(1);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}