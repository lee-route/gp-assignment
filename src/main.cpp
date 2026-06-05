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

static Vec2 TipOnSurface(const ContactInfo& hit, float tipRadius)
{
    return hit.point + hit.normal * (tipRadius - 0.5f);
}

static ContactInfo FindHammerTipContact(const Vec2& tipPos, float tipRadius, const std::vector<RectF>& world)
{
    return HammerTipVsWorld(Circle{ tipPos, tipRadius * 0.92f }, world);
}

static ContactInfo BodyVsWorld(const Vec2& pos, float radius, const std::vector<RectF>& world)
{
    ContactInfo best;
    float bestPen = -1.0f;

    for (const RectF& rect : world)
    {
        ContactInfo hit = CircleVsRect(Circle{ pos, radius }, rect);

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
    constexpr int ITERATIONS = 14;

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

static void SettlePlayerOnGround(Player& player, const std::vector<RectF>& world)
{
    const float floorTop = 700.0f;
    player.pos.y = floorTop - player.radius - 1.0f;
    player.vel = Vec2(0.0f, 0.0f);
    player.hammerPinned = false;

    for (int i = 0; i < 8; ++i)
    {
        ResolveBodyWorld(player, world);
    }

    player.vel = Vec2(0.0f, 0.0f);
    player.hammerPinned = false;
    player.prevHammerTip = player.HammerTip();
    player.prevHammerAngle = player.hammerAngle;
    player.prevElbow = player.ElbowWorld();
}

static bool ApplyPinnedLever(
    Player& player,
    const Vec2& attachPoint,
    const Vec2& mouseWorld,
    const Vec2& prevBodyPos,
    float dt,
    const std::vector<RectF>& world,
    float mouseMoveDistance)
{
    if (mouseMoveDistance < 3.0f)
    {
        return false;
    }

    player.SolveArmFromPinnedTip(attachPoint, mouseWorld);

    Vec2 desiredPos = player.BodyPosFromPinnedTip(attachPoint, mouseWorld);
    Vec2 delta = desiredPos - prevBodyPos;

    const float maxStepMove = 36.0f;
    float deltaLen = Length(delta);
    if (deltaLen > maxStepMove)
    {
        delta = delta * (maxStepMove / deltaLen);
    }

    if (deltaLen < 0.4f)
    {
        return false;
    }

    float moveT = 1.0f;
    for (int attempt = 0; attempt < 5; ++attempt)
    {
        player.pos = prevBodyPos + delta * moveT;
        ResolveBodyWorld(player, world);

        ContactInfo bodyHit = BodyVsWorld(player.pos, player.radius, world);
        if (!bodyHit.hit || bodyHit.penetration <= 2.0f)
        {
            break;
        }

        moveT *= 0.5f;
    }

    if (moveT < 0.25f)
    {
        player.pos = prevBodyPos;
        return false;
    }

    player.hammerPinned = true;
    player.pinnedAttachPoint = attachPoint;

    Vec2 constraintVel = (player.pos - prevBodyPos) / dt;

    float angleDelta = WrapAngleDiff(player.hammerAngle, player.prevHammerAngle);
    Vec2 shoulder = player.ShoulderWorld();
    Vec2 leverArm = shoulder - attachPoint;
    float leverLength = Length(leverArm);

    if (leverLength > 12.0f && std::abs(angleDelta) > 0.0002f)
    {
        Vec2 tangent = Normalize(Vec2(-leverArm.y, leverArm.x));
        float sign = (angleDelta >= 0.0f) ? 1.0f : -1.0f;
        Vec2 leverVel = tangent * sign * (std::abs(angleDelta) * leverLength / dt);
        constraintVel = constraintVel * 0.7f + leverVel * 0.3f;
    }

    float constraintSpeed = Length(constraintVel);
    if (constraintSpeed > 650.0f)
    {
        constraintVel = Normalize(constraintVel) * 650.0f;
    }

    player.vel = constraintVel;
    return true;
}

static void BuildLevel(std::vector<RectF>& world)
{
    world.clear();

    const float thickness = 44.0f;
    const float stepWidth = 72.0f;
    const float stepRise = 16.0f;
    const float overlap = 40.0f;

    // Left boundary wall
    world.push_back({ -260.0f, -2200.0f, 44.0f, 5000.0f });

    // Starting flat ground
    world.push_back({ -240.0f, 700.0f, 420.0f, thickness });

    // Gentle continuous slope (fine steps read as a smooth incline)
    float x = 140.0f;
    float topY = 700.0f;
    const float slopeStartX = -240.0f;
    const float slopeStartY = 700.0f;

    for (int i = 0; i < 52; ++i)
    {
        world.push_back({ x, topY, stepWidth + overlap, thickness });
        x += stepWidth;
        topY -= stepRise;
    }

    const float slopeEndX = x;
    const float slopeEndY = topY;

    // Solid fill under the slope only (keeps spawn area clear)
    const float fillTop = slopeStartY + thickness + 4.0f;
    world.push_back({
        200.0f,
        slopeEndY - 40.0f,
        (slopeEndX - 200.0f) + 80.0f,
        fillTop - (slopeEndY - 40.0f)
    });

    // Goal plateau
    world.push_back({ slopeEndX - 60.0f, slopeEndY, 320.0f, thickness + 6.0f });

    // Scattered obstacles on the slope (not removing terrain, adding challenge)
    world.push_back({ 520.0f, 590.0f, 36.0f, 110.0f });
    world.push_back({ 1180.0f, 410.0f, 140.0f, 28.0f });
    world.push_back({ 1680.0f, 250.0f, 36.0f, 130.0f });
    world.push_back({ 2280.0f, 90.0f, 100.0f, 28.0f });
    world.push_back({ 2860.0f, -70.0f, 36.0f, 150.0f });

    // Side hook rocks
    world.push_back({ 2100.0f, 60.0f, 32.0f, 160.0f });
    world.push_back({ 3400.0f, -120.0f, 32.0f, 140.0f });

    // Wide thick bottom floor
    world.push_back({ -900.0f, 1700.0f, 5600.0f, 900.0f });
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

static void GetArmPose(
    const Player& player,
    const Vec2& mouseWorld,
    Vec2& shoulder,
    Vec2& elbow,
    Vec2& tip,
    Vec2& dir)
{
    if (player.hammerPinned)
    {
        tip = player.pinnedAttachPoint;
        Vec2 shaft = mouseWorld - tip;
        if (Length(shaft) > 0.001f)
        {
            shaft = Normalize(shaft);
        }
        else
        {
            shaft = player.HammerDir();
        }

        elbow = tip + shaft * player.forearmLength;
        dir = Normalize(tip - elbow);
        shoulder = player.ShoulderWorld();
        return;
    }

    shoulder = player.ShoulderWorld();
    elbow = player.ElbowWorld();
    tip = player.HammerTip();
    dir = player.HammerDir();
}

static void DrawPickaxe(SDL_Renderer* renderer, const Player& player, const Camera& camera, const Vec2& mouseWorld, bool contact)
{
    Vec2 shoulder;
    Vec2 elbow;
    Vec2 tip;
    Vec2 dir;
    GetArmPose(player, mouseWorld, shoulder, elbow, tip, dir);

    Vec2 shoulderScreen = WorldToScreen(shoulder, camera);
    Vec2 elbowScreen = WorldToScreen(elbow, camera);
    Vec2 tipScreen = WorldToScreen(tip, camera);

    SDL_SetRenderDrawColor(renderer, 200, 175, 140, 255);
    DrawArmSegment(renderer, shoulder, elbow, camera);

    SDL_SetRenderDrawColor(renderer, 220, 220, 225, 255);
    DrawArmSegment(renderer, elbow, tip, camera);

    SDL_SetRenderDrawColor(renderer, 180, 150, 110, 255);
    DrawCircleOutline(renderer, static_cast<int>(elbowScreen.x), static_cast<int>(elbowScreen.y), 5);

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

    DrawCircleOutline(
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

    const float pinEnterSwingSpeed = 420.0f;
    constexpr int MAX_PHYSICS_STEPS = 2;

    Uint64 previousCounter = SDL_GetPerformanceCounter();
    double accumulator = 0.0;
    Vec2 mouseWorld(0.0f, 0.0f);
    Vec2 prevMouseWorld(0.0f, 0.0f);
    bool mouseWorldInitialized = false;

    SettlePlayerOnGround(player, world);

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
                    SettlePlayerOnGround(player, world);
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

        mouseWorld = Vec2(
            static_cast<float>(mouseX) + camera.x,
            static_cast<float>(mouseY) + camera.y
        );

        if (!mouseWorldInitialized)
        {
            prevMouseWorld = mouseWorld;
            mouseWorldInitialized = true;
        }

        float mouseMoveDistance = Length(mouseWorld - prevMouseWorld);

        int physicsSteps = 0;
        while (accumulator >= FIXED_DT && physicsSteps < MAX_PHYSICS_STEPS)
        {
            player.prevHammerAngle = player.hammerAngle;
            player.hammerPinned = false;

            Vec2 prevBodyPos = player.pos;
            Vec2 prevTip = player.prevHammerTip;

            player.vel.y += gravity * FIXED_DT;
            player.vel = player.vel * airDamping;
            player.SolveArmIK(mouseWorld, mouseWorld);

            Vec2 currentTip = player.HammerTip();
            ContactInfo tipHit = FindHammerTipContact(currentTip, player.hammerTipRadius, world);
            ContactInfo sweptHit = FindHammerTipContact(prevTip, player.hammerTipRadius, world);

            if (!tipHit.hit && sweptHit.hit)
            {
                tipHit = sweptHit;
            }

            Vec2 rawTipVelocity = (currentTip - prevTip) / FIXED_DT;
            float swingSpeed = Length(rawTipVelocity - player.vel);

            bool pinApplied = false;
            if (tipHit.hit && swingSpeed < pinEnterSwingSpeed && mouseMoveDistance > 3.0f)
            {
                Vec2 attachPoint = TipOnSurface(tipHit, player.hammerTipRadius);
                pinApplied = ApplyPinnedLever(
                    player,
                    attachPoint,
                    mouseWorld,
                    prevBodyPos,
                    FIXED_DT,
                    world,
                    mouseMoveDistance
                );
            }

            if (!pinApplied)
            {
                player.pos += player.vel * FIXED_DT;
            }

            ResolveBodyWorld(player, world);

            if (player.hammerPinned)
            {
                player.SolveArmFromPinnedTip(player.pinnedAttachPoint, mouseWorld);
            }
            else
            {
                player.SolveArmIK(mouseWorld, mouseWorld);
            }

            float speed = Length(player.vel);
            if (speed > maxSpeed)
            {
                player.vel = Normalize(player.vel) * maxSpeed;
            }

            float angleDiff = WrapAngleDiff(player.hammerAngle, player.prevHammerAngle);
            player.hammerAngularVelocity = angleDiff / FIXED_DT;

            player.prevHammerTip = player.hammerPinned ? player.pinnedAttachPoint : player.HammerTip();
            player.prevElbow = player.ElbowWorld();

            accumulator -= FIXED_DT;
            physicsSteps++;
        }

        prevMouseWorld = mouseWorld;

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
            ScreenX(3580.0f, camera),
            ScreenY(-150.0f, camera),
            100,
            45
        };
        SDL_RenderFillRect(renderer, &goalRect);

        // Draw pickaxe / hammer (single pose synced to mouse when pinned)
        DrawPickaxe(renderer, player, camera, mouseWorld, player.hammerPinned);

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
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}