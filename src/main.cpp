#include <SDL.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include "../Player.h"

constexpr int WINDOW_WIDTH = 1280;
constexpr int WINDOW_HEIGHT = 720;
constexpr float FIXED_DT = 1.0f / 120.0f;
constexpr float PI = 3.1415926535f;

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
    Vec2 point = Vec2(0.0f, 0.0f);
};

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

static Vec2 DirectionFromAngle(float angle)
{
    return Vec2(std::cos(angle), std::sin(angle));
}

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

static Vec2 PickaxeSharpPoint(const Player& player)
{
    Vec2 dir = player.HammerDir();
    return player.HammerTip() + dir * 12.0f;
}

static Vec2 PreviousPickaxeSharpPoint(const Player& player)
{
    Vec2 previousDir = DirectionFromAngle(player.prevHammerAngle);
    return player.prevHammerTip + previousDir * 12.0f;
}

static void DrawCircle(SDL_Renderer* renderer, int cx, int cy, int radius)
{
    for (int x = -radius; x <= radius; ++x)
    {
        for (int y = -radius; y <= radius; ++y)
        {
            if (x * x + y * y <= radius * radius)
            {
                SDL_RenderDrawPoint(renderer, cx + x, cy + y);
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
    float radiusSq = c.radius * c.radius;

    if (distSq > 0.0001f && distSq < radiusSq)
    {
        float dist = std::sqrt(distSq);

        info.hit = true;
        info.point = closest;
        info.normal = delta / dist;
        info.penetration = c.radius - dist;

        return info;
    }

    bool inside =
        c.center.x >= r.x &&
        c.center.x <= r.x + r.w &&
        c.center.y >= r.y &&
        c.center.y <= r.y + r.h;

    if (inside)
    {
        float distLeft = c.center.x - r.x;
        float distRight = (r.x + r.w) - c.center.x;
        float distTop = c.center.y - r.y;
        float distBottom = (r.y + r.h) - c.center.y;

        float minDist = distLeft;
        info.normal = Vec2(-1.0f, 0.0f);

        if (distRight < minDist)
        {
            minDist = distRight;
            info.normal = Vec2(1.0f, 0.0f);
        }

        if (distTop < minDist)
        {
            minDist = distTop;
            info.normal = Vec2(0.0f, -1.0f);
        }

        if (distBottom < minDist)
        {
            minDist = distBottom;
            info.normal = Vec2(0.0f, 1.0f);
        }

        info.hit = true;
        info.point = c.center;
        info.penetration = minDist + c.radius + 0.5f;

        return info;
    }

    return info;
}

static ContactInfo BestCircleWorldContact(const Circle& c, const std::vector<RectF>& world)
{
    ContactInfo best;
    float bestPenetration = -1.0f;

    for (const RectF& rect : world)
    {
        ContactInfo hit = CircleVsRect(c, rect);

        if (hit.hit && hit.penetration > bestPenetration)
        {
            best = hit;
            bestPenetration = hit.penetration;
        }
    }

    return best;
}

static ContactInfo SweptSharpTipContact(
    const Vec2& previousTip,
    const Vec2& currentTip,
    float radius,
    const std::vector<RectF>& world)
{
    ContactInfo best;
    float bestPenetration = -1.0f;

    Vec2 sweep = currentTip - previousTip;
    float sweepLength = Length(sweep);

    int samples = 1;

    if (sweepLength > 4.0f)
    {
        samples = static_cast<int>(std::ceil(sweepLength / 4.0f));
    }

    if (samples < 1)
    {
        samples = 1;
    }

    if (samples > 48)
    {
        samples = 48;
    }

    for (int i = 0; i <= samples; ++i)
    {
        float t = static_cast<float>(i) / static_cast<float>(samples);
        Vec2 p = previousTip + sweep * t;

        ContactInfo hit = BestCircleWorldContact(Circle{ p, radius }, world);

        if (hit.hit && hit.penetration > bestPenetration)
        {
            best = hit;
            bestPenetration = hit.penetration;
        }
    }

    return best;
}

static void ResolveBodyWorld(Player& player, const std::vector<RectF>& world)
{
    const int iterations = 16;

    for (int i = 0; i < iterations; ++i)
    {
        bool anyHit = false;

        for (const RectF& rect : world)
        {
            ContactInfo hit = CircleVsRect(Circle{ player.pos, player.radius }, rect);

            if (hit.hit)
            {
                anyHit = true;

                player.pos += hit.normal * (hit.penetration + 0.4f);

                float vn = Dot(player.vel, hit.normal);

                if (vn < 0.0f)
                {
                    player.vel -= hit.normal * vn;
                }

                if (std::abs(hit.normal.y) > 0.7f)
                {
                    player.vel.x *= 0.82f;
                }

                if (std::abs(hit.normal.x) > 0.7f)
                {
                    player.vel.y *= 0.90f;
                }
            }
        }

        if (!anyHit)
        {
            break;
        }
    }
}

static void MoveBodySafely(Player& player, const std::vector<RectF>& world, float dt)
{
    Vec2 totalMove = player.vel * dt;
    float moveLength = Length(totalMove);

    int steps = 1;

    if (moveLength > 1.5f)
    {
        steps = static_cast<int>(std::ceil(moveLength / 1.5f));
    }

    if (steps < 1)
    {
        steps = 1;
    }

    if (steps > 96)
    {
        steps = 96;
    }

    Vec2 stepMove = totalMove / static_cast<float>(steps);

    for (int i = 0; i < steps; ++i)
    {
        Vec2 beforeMove = player.pos;

        player.pos += stepMove;
        ResolveBodyWorld(player, world);

        ContactInfo bodyHit = BestCircleWorldContact(
            Circle{ player.pos, player.radius },
            world
        );

        if (bodyHit.hit)
        {
            float vn = Dot(player.vel, bodyHit.normal);

            if (vn < 0.0f)
            {
                player.vel -= bodyHit.normal * vn;
            }

            ContactInfo stillHit = BestCircleWorldContact(
                Circle{ player.pos, player.radius },
                world
            );

            if (stillHit.hit && stillHit.penetration > player.radius * 0.35f)
            {
                player.pos = beforeMove;
                player.vel = player.vel * 0.25f;
                ResolveBodyWorld(player, world);
                break;
            }
        }
    }

    ResolveBodyWorld(player, world);
}

static void MoveBodyByCorrection(Player& player, const std::vector<RectF>& world, Vec2 correction)
{
    float correctionLength = Length(correction);

    if (correctionLength <= 0.001f)
    {
        return;
    }

    float maxCorrection = 9.0f;

    if (correctionLength > maxCorrection)
    {
        correction = correction * (maxCorrection / correctionLength);
        correctionLength = maxCorrection;
    }

    int steps = static_cast<int>(std::ceil(correctionLength / 1.5f));

    if (steps < 1)
    {
        steps = 1;
    }

    if (steps > 12)
    {
        steps = 12;
    }

    Vec2 stepMove = correction / static_cast<float>(steps);

    for (int i = 0; i < steps; ++i)
    {
        Vec2 before = player.pos;

        player.pos += stepMove;
        ResolveBodyWorld(player, world);

        ContactInfo bodyHit = BestCircleWorldContact(
            Circle{ player.pos, player.radius },
            world
        );

        if (bodyHit.hit && bodyHit.penetration > player.radius * 0.45f)
        {
            player.pos = before;
            player.vel = player.vel * 0.35f;
            ResolveBodyWorld(player, world);
            break;
        }
    }
}

static void BuildLevel(std::vector<RectF>& world)
{
    world.clear();

    const float T = 34.0f;

    world.push_back({ -500.0f, 690.0f, 900.0f, 60.0f });
    world.push_back({ -540.0f, -300.0f, 40.0f, 1100.0f });

    world.push_back({ 360.0f, 665.0f, 230.0f, T });
    world.push_back({ 590.0f, 635.0f, 170.0f, T });
    world.push_back({ 760.0f, 610.0f, 220.0f, T });
    world.push_back({ 980.0f, 575.0f, 160.0f, T });
    world.push_back({ 1140.0f, 545.0f, 240.0f, T });

    world.push_back({ 500.0f, 620.0f, 26.0f, 45.0f });
    world.push_back({ 705.0f, 585.0f, 24.0f, 50.0f });
    world.push_back({ 910.0f, 555.0f, 26.0f, 55.0f });
    world.push_back({ 1290.0f, 495.0f, 28.0f, 50.0f });

    world.push_back({ 1420.0f, 510.0f, 190.0f, T });
    world.push_back({ 1605.0f, 475.0f, 120.0f, T });
    world.push_back({ 1725.0f, 430.0f, 210.0f, T });
    world.push_back({ 1935.0f, 395.0f, 130.0f, T });
    world.push_back({ 2070.0f, 350.0f, 230.0f, T });

    world.push_back({ 1515.0f, 450.0f, 28.0f, 60.0f });
    world.push_back({ 1830.0f, 365.0f, 28.0f, 65.0f });
    world.push_back({ 2180.0f, 285.0f, 28.0f, 65.0f });

    world.push_back({ 2360.0f, 315.0f, 180.0f, T });
    world.push_back({ 2520.0f, 285.0f, 80.0f, 80.0f });
    world.push_back({ 2620.0f, 255.0f, 210.0f, T });

    world.push_back({ 2870.0f, 220.0f, 160.0f, T });
    world.push_back({ 3025.0f, 175.0f, 90.0f, 90.0f });
    world.push_back({ 3150.0f, 155.0f, 220.0f, T });

    world.push_back({ 3440.0f, 120.0f, 260.0f, T });
    world.push_back({ 3710.0f, 80.0f, 180.0f, T });
    world.push_back({ 3890.0f, 40.0f, 380.0f, 46.0f });

    world.push_back({ 4070.0f, -50.0f, 30.0f, 90.0f });

    world.push_back({ 650.0f, 560.0f, 70.0f, 24.0f });
    world.push_back({ 1040.0f, 505.0f, 75.0f, 24.0f });
    world.push_back({ 1690.0f, 390.0f, 70.0f, 24.0f });
    world.push_back({ 2430.0f, 250.0f, 70.0f, 24.0f });
    world.push_back({ 2950.0f, 140.0f, 70.0f, 24.0f });
    world.push_back({ 3550.0f, 45.0f, 80.0f, 24.0f });
}

static void UpdateHammerAngle(Player& player, const Vec2& mouseWorld)
{
    Vec2 toMouse = mouseWorld - player.pos;

    if (Length(toMouse) > 0.001f)
    {
        player.targetHammerAngle = std::atan2(toMouse.y, toMouse.x);
    }

    float diff = WrapAngleDiff(player.targetHammerAngle, player.hammerAngle);

    const float stiffness = 250.0f;
    const float damping = 0.84f;
    const float maxAngularVelocity = 48.0f;

    player.hammerAngularVelocity += diff * stiffness * FIXED_DT;
    player.hammerAngularVelocity *= damping;

    player.hammerAngularVelocity = Clamp(
        player.hammerAngularVelocity,
        -maxAngularVelocity,
        maxAngularVelocity
    );

    player.hammerAngle += player.hammerAngularVelocity * FIXED_DT;

    float remainingDiff = WrapAngleDiff(player.targetHammerAngle, player.hammerAngle);
    player.hammerAngle += remainingDiff * 0.22f;
}

static void ApplyHammerPhysics(Player& player, const std::vector<RectF>& world)
{
    Vec2 sharpTip = PickaxeSharpPoint(player);
    Vec2 previousSharpTip = PreviousPickaxeSharpPoint(player);

    Vec2 tipVelocity = (sharpTip - previousSharpTip) / FIXED_DT;
    Vec2 relativeTipVelocity = tipVelocity - player.vel;

    ContactInfo hit = SweptSharpTipContact(
        previousSharpTip,
        sharpTip,
        player.hammerTipRadius,
        world
    );

    if (!hit.hit)
    {
        return;
    }

    float intoSurfaceSpeed = Dot(relativeTipVelocity, hit.normal * -1.0f);

    Vec2 oppositeSwing = Normalize(relativeTipVelocity * -1.0f);
    Vec2 reactionDir = hit.normal;

    if (Length(oppositeSwing) > 0.001f)
    {
        reactionDir = Normalize(hit.normal * 0.72f + oppositeSwing * 0.28f);
    }

    if (intoSurfaceSpeed < 0.0f)
    {
        intoSurfaceSpeed = 0.0f;
    }

    intoSurfaceSpeed = Clamp(intoSurfaceSpeed, 0.0f, 1200.0f);

    float correctionStrength = 0.85f;
    float correctionAmount = Clamp(hit.penetration + intoSurfaceSpeed * 0.004f, 0.0f, 10.0f);
    Vec2 correction = reactionDir * correctionAmount * correctionStrength;

    MoveBodyByCorrection(player, world, correction);

    float force = 900.0f + intoSurfaceSpeed * 4.6f;
    force = Clamp(force, 0.0f, 5600.0f);

    player.vel += reactionDir * force * FIXED_DT;

    float speed = Length(player.vel);

    if (speed > 1450.0f)
    {
        player.vel = Normalize(player.vel) * 1450.0f;
    }
}

static void DrawPickaxe(SDL_Renderer* renderer, const Player& player, const Camera& camera, bool contact)
{
    Vec2 bodyWorld = player.pos;
    Vec2 tipWorld = player.HammerTip();

    Vec2 body = WorldToScreen(bodyWorld, camera);
    Vec2 tip = WorldToScreen(tipWorld, camera);

    Vec2 dir = player.HammerDir();
    Vec2 normal(-dir.y, dir.x);

    SDL_SetRenderDrawColor(renderer, 210, 210, 215, 255);

    SDL_RenderDrawLine(
        renderer,
        static_cast<int>(body.x),
        static_cast<int>(body.y),
        static_cast<int>(tip.x),
        static_cast<int>(tip.y)
    );

    SDL_RenderDrawLine(
        renderer,
        static_cast<int>(body.x + 1),
        static_cast<int>(body.y),
        static_cast<int>(tip.x + 1),
        static_cast<int>(tip.y)
    );

    Vec2 headCenterWorld = tipWorld - dir * 14.0f;
    Vec2 sharpPointWorld = tipWorld + dir * 12.0f;
    Vec2 backPointWorld = headCenterWorld - dir * 6.0f - normal * 34.0f;
    Vec2 upperHookWorld = headCenterWorld - dir * 4.0f + normal * 34.0f;

    Vec2 headCenter = WorldToScreen(headCenterWorld, camera);
    Vec2 sharpPoint = WorldToScreen(sharpPointWorld, camera);
    Vec2 backPoint = WorldToScreen(backPointWorld, camera);
    Vec2 upperHook = WorldToScreen(upperHookWorld, camera);

    if (contact)
    {
        SDL_SetRenderDrawColor(renderer, 255, 70, 70, 255);
    }
    else
    {
        SDL_SetRenderDrawColor(renderer, 120, 200, 255, 255);
    }

    SDL_RenderDrawLine(
        renderer,
        static_cast<int>(backPoint.x),
        static_cast<int>(backPoint.y),
        static_cast<int>(sharpPoint.x),
        static_cast<int>(sharpPoint.y)
    );

    SDL_RenderDrawLine(
        renderer,
        static_cast<int>(upperHook.x),
        static_cast<int>(upperHook.y),
        static_cast<int>(sharpPoint.x),
        static_cast<int>(sharpPoint.y)
    );

    SDL_RenderDrawLine(
        renderer,
        static_cast<int>(headCenter.x),
        static_cast<int>(headCenter.y),
        static_cast<int>(sharpPoint.x),
        static_cast<int>(sharpPoint.y)
    );

    SDL_RenderDrawPoint(renderer, static_cast<int>(sharpPoint.x), static_cast<int>(sharpPoint.y));
    SDL_RenderDrawPoint(renderer, static_cast<int>(sharpPoint.x + 1), static_cast<int>(sharpPoint.y));
    SDL_RenderDrawPoint(renderer, static_cast<int>(sharpPoint.x), static_cast<int>(sharpPoint.y + 1));
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

    const float gravity = 2350.0f;
    const float airDamping = 0.999f;
    const float maxSpeed = 1450.0f;

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

        camera.x = player.pos.x - WINDOW_WIDTH * 0.35f;
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
            player.prevHammerTip = player.HammerTip();

            UpdateHammerAngle(player, mouseWorld);
            ApplyHammerPhysics(player, world);

            player.vel.y += gravity * FIXED_DT;
            player.vel = player.vel * airDamping;

            float speed = Length(player.vel);

            if (speed > maxSpeed)
            {
                player.vel = Normalize(player.vel) * maxSpeed;
            }

            MoveBodySafely(player, world, FIXED_DT);

            player.currentHammerTip = player.HammerTip();

            if (player.pos.y > 1200.0f)
            {
                player.Reset();
            }

            accumulator -= FIXED_DT;
        }

        SDL_SetRenderDrawColor(renderer, 18, 18, 26, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 35, 35, 48, 255);

        for (int y = -100; y <= 900; y += 120)
        {
            SDL_RenderDrawLine(
                renderer,
                0,
                ScreenY(static_cast<float>(y), camera),
                WINDOW_WIDTH,
                ScreenY(static_cast<float>(y), camera)
            );
        }

        SDL_SetRenderDrawColor(renderer, 110, 110, 130, 255);

        for (const RectF& rect : world)
        {
            DrawWorldRect(renderer, rect, camera);
        }

        SDL_SetRenderDrawColor(renderer, 255, 220, 80, 255);

        SDL_Rect goalRect{
            ScreenX(4040.0f, camera),
            ScreenY(-90.0f, camera),
            140,
            50
        };

        SDL_RenderFillRect(renderer, &goalRect);

        Vec2 hammerTip = PickaxeSharpPoint(player);

        ContactInfo tipHit = SweptSharpTipContact(
            PreviousPickaxeSharpPoint(player),
            hammerTip,
            player.hammerTipRadius,
            world
        );

        DrawPickaxe(renderer, player, camera, tipHit.hit);

        Vec2 playerScreen = WorldToScreen(player.pos, camera);

        SDL_SetRenderDrawColor(renderer, 174, 218, 238, 255);

        DrawCircle(
            renderer,
            static_cast<int>(playerScreen.x),
            static_cast<int>(playerScreen.y),
            static_cast<int>(player.radius)
        );

        SDL_SetRenderDrawColor(renderer, 20, 45, 75, 255);

        DrawCircleOutline(
            renderer,
            static_cast<int>(playerScreen.x),
            static_cast<int>(playerScreen.y),
            static_cast<int>(player.radius)
        );

        SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);

        DrawCircle(
            renderer,
            static_cast<int>(playerScreen.x - 8),
            static_cast<int>(playerScreen.y - 7),
            3
        );

        DrawCircle(
            renderer,
            static_cast<int>(playerScreen.x + 8),
            static_cast<int>(playerScreen.y - 7),
            3
        );

        SDL_SetRenderDrawColor(renderer, 255, 150, 20, 255);

        SDL_RenderDrawLine(
            renderer,
            static_cast<int>(playerScreen.x),
            static_cast<int>(playerScreen.y - 2),
            static_cast<int>(playerScreen.x - 5),
            static_cast<int>(playerScreen.y + 5)
        );

        SDL_RenderDrawLine(
            renderer,
            static_cast<int>(playerScreen.x),
            static_cast<int>(playerScreen.y - 2),
            static_cast<int>(playerScreen.x + 5),
            static_cast<int>(playerScreen.y + 5)
        );

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}