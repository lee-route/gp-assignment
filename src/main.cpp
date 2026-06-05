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
    constexpr int ITERATIONS = 5;

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

                // Simple friction on mostly horizontal surfaces
                if (std::abs(hit.normal.y) > 0.7f)
                {
                    player.vel.x *= 0.94f;
                }

                // Small damping on side wall contact
                if (std::abs(hit.normal.x) > 0.7f)
                {
                    player.vel.y *= 0.98f;
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

    // Starting floor
    world.push_back({ -200.0f, 680.0f, 900.0f, 40.0f });

    // Left safety wall
    world.push_back({ -240.0f, -1600.0f, 40.0f, 2320.0f });

    // Getting Over It style climbing route
    world.push_back({ 200.0f, 610.0f, 160.0f, 24.0f });
    world.push_back({ 430.0f, 540.0f, 120.0f, 24.0f });
    world.push_back({ 620.0f, 455.0f, 170.0f, 24.0f });
    world.push_back({ 880.0f, 365.0f, 110.0f, 24.0f });

    // Small vertical obstacles
    world.push_back({ 520.0f, 470.0f, 28.0f, 70.0f });
    world.push_back({ 760.0f, 375.0f, 28.0f, 80.0f });

    // Higher platforms
    world.push_back({ 1040.0f, 270.0f, 180.0f, 24.0f });
    world.push_back({ 940.0f, 130.0f, 110.0f, 24.0f });
    world.push_back({ 720.0f, 10.0f, 150.0f, 24.0f });
    world.push_back({ 500.0f, -120.0f, 120.0f, 24.0f });
    world.push_back({ 300.0f, -260.0f, 170.0f, 24.0f });

    // Narrow challenge
    world.push_back({ 580.0f, -430.0f, 90.0f, 22.0f });
    world.push_back({ 760.0f, -560.0f, 120.0f, 22.0f });
    world.push_back({ 980.0f, -700.0f, 160.0f, 22.0f });

    // Goal area
    world.push_back({ 760.0f, -920.0f, 280.0f, 28.0f });

    // Some side blocks for hammer contact
    world.push_back({ 1120.0f, -880.0f, 35.0f, 180.0f });
    world.push_back({ 660.0f, -900.0f, 35.0f, 140.0f });
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

    SDL_RenderDrawLine(
        renderer,
        static_cast<int>(aScreen.x + 1),
        static_cast<int>(aScreen.y),
        static_cast<int>(bScreen.x + 1),
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

static void ApplyFallRecovery(Player& player, float dt)
{
    const float fallStartY = player.spawnPos.y + 180.0f;
    if (player.pos.y <= fallStartY)
    {
        return;
    }

    float fallDepth = player.pos.y - fallStartY;
    float depthRatio = Clamp(fallDepth / 520.0f, 0.0f, 1.0f);

    Vec2 toSpawn = player.spawnPos - player.pos;
    Vec2 pullDir = Normalize(toSpawn + Vec2(0.0f, -120.0f));

    float pullStrength = 900.0f + depthRatio * 2400.0f;
    float liftStrength = 1400.0f + depthRatio * 3200.0f;

    player.vel += pullDir * pullStrength * dt;
    player.vel.y -= liftStrength * dt;

    player.vel.x *= 1.0f - 0.35f * depthRatio * dt;
    player.vel = player.vel * (1.0f - 0.12f * depthRatio * dt);

    if (player.pos.y > player.spawnPos.y + 760.0f)
    {
        player.pos = player.spawnPos + Vec2(0.0f, -40.0f);
        player.vel = Vec2(0.0f, -420.0f);
        player.hammerAttached = false;
    }
}

static void ApplyAttachedLeverPhysics(Player& player, float dt)
{
    if (!player.hammerAttached)
    {
        return;
    }

    float angleDelta = WrapAngleDiff(player.hammerAngle, player.prevHammerAngle);
    if (std::abs(angleDelta) < 0.00001f)
    {
        return;
    }

    Vec2 shoulder = player.ShoulderWorld();
    Vec2 attach = player.hammerAttachPoint;
    Vec2 leverArm = shoulder - attach;
    float leverLength = Length(leverArm);

    if (leverLength < 8.0f)
    {
        return;
    }

    Vec2 tangent(-leverArm.y, leverArm.x);
    tangent = Normalize(tangent);

    float sign = (angleDelta >= 0.0f) ? 1.0f : -1.0f;
    float leverStrength = leverLength * std::abs(angleDelta) * 5200.0f;
    leverStrength = Clamp(leverStrength, 0.0f, 4200.0f);

    player.vel += tangent * sign * leverStrength * dt;

    Vec2 radial = Normalize(leverArm);
    float radialPush = angleDelta * leverLength * 18.0f;
    player.vel += radial * radialPush * dt;
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

    const float gravity = 2100.0f;
    const float airDamping = 0.9975f;
    const float maxSpeed = 1350.0f;

    // Main tuning values for hammer physics
    const float minSwingSpeed = 28.0f;
    const float attachMaxSwingSpeed = 520.0f;
    const float baseHammerForce = 1200.0f;
    const float swingForceMultiplier = 4.6f;
    const float pressForceMultiplier = 4.2f;
    const float maxHammerForce = 6800.0f;

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

            if (player.hammerAttached)
            {
                player.SolveArmFromAttachedTip(mouseWorld);
            }
            else
            {
                player.SolveArmIK(mouseWorld, mouseWorld);
            }

            float angleDiff = WrapAngleDiff(player.hammerAngle, player.prevHammerAngle);
            player.hammerAngularVelocity = angleDiff / FIXED_DT;

            Vec2 currentTip = player.HammerTip();
            Vec2 currentElbow = player.ElbowWorld();
            Vec2 rawTipVelocity = (currentTip - player.prevHammerTip) / FIXED_DT;
            Vec2 rawElbowVelocity = (currentElbow - player.prevElbow) / FIXED_DT;

            Vec2 relativeTipVelocity = rawTipVelocity - player.vel;
            Vec2 relativeElbowVelocity = rawElbowVelocity - player.vel;

            // Gravity and damping
            player.vel.y += gravity * FIXED_DT;
            player.vel = player.vel * airDamping;

            ApplyFallRecovery(player, FIXED_DT);

            float speed = Length(player.vel);
            if (speed > maxSpeed)
            {
                player.vel = Normalize(player.vel) * maxSpeed;
            }

            Circle tipCollider{ currentTip, player.hammerTipRadius };
            ContactInfo tipHit = HammerTipVsWorld(tipCollider, world);

            float swingSpeed = Length(relativeTipVelocity);
            float elbowSwingSpeed = Length(relativeElbowVelocity);

            if (player.hammerAttached)
            {
                if (!tipHit.hit || swingSpeed > attachMaxSwingSpeed)
                {
                    player.hammerAttached = false;
                }
                else
                {
                    player.hammerAttachPoint = tipHit.point;
                    ApplyAttachedLeverPhysics(player, FIXED_DT);
                }
            }
            else if (tipHit.hit)
            {
                Vec2 surfaceNormal = tipHit.normal;
                float pressSpeed = std::max(0.0f, Dot(relativeTipVelocity, surfaceNormal * -1.0f));

                bool canGrip =
                    swingSpeed < attachMaxSwingSpeed &&
                    pressSpeed > 18.0f &&
                    elbowSwingSpeed < attachMaxSwingSpeed * 0.85f;

                if (canGrip)
                {
                    player.hammerAttached = true;
                    player.hammerAttachPoint = tipHit.point;
                }
                else if (swingSpeed > minSwingSpeed)
                {
                    Vec2 oppositeSwing = Normalize(relativeTipVelocity * -1.0f);
                    Vec2 reactionDir = Normalize(oppositeSwing * 0.72f + surfaceNormal * 0.28f);

                    float forceFromSwing = swingSpeed * swingForceMultiplier;
                    float forceFromPress = pressSpeed * pressForceMultiplier;
                    float finalForce = baseHammerForce + forceFromSwing + forceFromPress;
                    finalForce = Clamp(finalForce, 0.0f, maxHammerForce);

                    player.vel += reactionDir * finalForce * FIXED_DT;
                    player.pos += surfaceNormal * (tipHit.penetration * 0.42f);
                }
            }

            player.pos += player.vel * FIXED_DT;
            ResolveBodyWorld(player, world);

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

        // Draw goal marker
        SDL_SetRenderDrawColor(renderer, 255, 220, 80, 255);
        SDL_Rect goalRect{
            ScreenX(850.0f, camera),
            ScreenY(-970.0f, camera),
            100,
            45
        };
        SDL_RenderFillRect(renderer, &goalRect);

        // Hammer contact check for rendering color
        Vec2 hammerTip = player.HammerTip();
        Circle tipCollider{ hammerTip, player.hammerTipRadius };
        ContactInfo tipHit = HammerTipVsWorld(tipCollider, world);

        // Draw pickaxe / hammer
        DrawPickaxe(renderer, player, camera, tipHit.hit);

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