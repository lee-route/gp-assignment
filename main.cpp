#include <SDL.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include "Player.h"

// ------------------------------------------------------------
// Pot Hammer Physics Prototype (SDL2 Single File)
// ------------------------------------------------------------
// Features
// - Gravity
// - Circle body physics
// - Circle vs Rect collision resolution
// - Hammer controlled by mouse angle
// - Hammer tip contact detection
// - Reaction force movement while holding left mouse button
// - Simple level made of rectangles
//
// Controls
// - Mouse: aim hammer
// - Left Mouse Button: push when hammer tip touches terrain
// - R: reset player
// - ESC / close button: quit
// ------------------------------------------------------------

constexpr int WINDOW_WIDTH = 1280;
constexpr int WINDOW_HEIGHT = 720;
constexpr float FIXED_DT = 1.0f / 120.0f;
constexpr int TARGET_FPS = 60;
constexpr float PI = 3.1415926535f;

static float Dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

static float Length(const Vec2& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

static Vec2 Normalize(const Vec2& v) {
    float len = Length(v);
    if (len <= 0.0001f) {
        return Vec2(0.0f, 0.0f);
    }
    return v / len;
}

static float Clamp(float value, float minValue, float maxValue) {
    return std::max(minValue, std::min(value, maxValue));
}

struct Circle {
    Vec2 center;
    float radius;
};

struct RectF {
    float x;
    float y;
    float w;
    float h;
};

struct Camera {
    float x;
    float y;
};

static int ScreenX(float worldX, const Camera& camera)
{
    return static_cast<int>(worldX - camera.x);
}

static int ScreenY(float worldY, const Camera& camera)
{
    return static_cast<int>(worldY - camera.y);
}

struct ContactInfo {
    bool hit = false;
    Vec2 normal = Vec2(0.0f, -1.0f);
    float penetration = 0.0f;
    Vec2 point;
};

static void DrawCircle(SDL_Renderer* renderer, int cx, int cy, int radius) {
    for (int w = -radius; w <= radius; ++w) {
        for (int h = -radius; h <= radius; ++h) {
            if (w * w + h * h <= radius * radius) {
                SDL_RenderDrawPoint(renderer, cx + w, cy + h);
            }
        }
    }
}

static void DrawCircleOutline(SDL_Renderer* renderer, int cx, int cy, int radius) {
    const int segments = 64;
    for (int i = 0; i < segments; ++i) {
        float a0 = (2.0f * PI * i) / segments;
        float a1 = (2.0f * PI * (i + 1)) / segments;
        int x0 = static_cast<int>(cx + std::cos(a0) * radius);
        int y0 = static_cast<int>(cy + std::sin(a0) * radius);
        int x1 = static_cast<int>(cx + std::cos(a1) * radius);
        int y1 = static_cast<int>(cy + std::sin(a1) * radius);
        SDL_RenderDrawLine(renderer, x0, y0, x1, y1);
    }
}

static void DrawRect(SDL_Renderer* renderer, const RectF& r) {
    SDL_Rect rect{
        static_cast<int>(r.x),
        static_cast<int>(r.y),
        static_cast<int>(r.w),
        static_cast<int>(r.h)
    };
    SDL_RenderFillRect(renderer, &rect);
}

static ContactInfo CircleVsRect(const Circle& c, const RectF& r) {
    ContactInfo info;

    float closestX = Clamp(c.center.x, r.x, r.x + r.w);
    float closestY = Clamp(c.center.y, r.y, r.y + r.h);
    Vec2 closest(closestX, closestY);
    Vec2 delta = c.center - closest;
    float distSq = Dot(delta, delta);

    if (distSq < c.radius * c.radius) {
        float dist = std::sqrt(std::max(0.00001f, distSq));
        info.hit = true;
        info.point = closest;

        if (dist > 0.0001f) {
            info.normal = delta / dist;
            info.penetration = c.radius - dist;
        }
        else {
            float leftPen = std::abs(c.center.x - r.x);
            float rightPen = std::abs((r.x + r.w) - c.center.x);
            float topPen = std::abs(c.center.y - r.y);
            float bottomPen = std::abs((r.y + r.h) - c.center.y);

            float minPen = leftPen;
            info.normal = Vec2(-1.0f, 0.0f);

            if (rightPen < minPen) {
                minPen = rightPen;
                info.normal = Vec2(1.0f, 0.0f);
            }
            if (topPen < minPen) {
                minPen = topPen;
                info.normal = Vec2(0.0f, -1.0f);
            }
            if (bottomPen < minPen) {
                minPen = bottomPen;
                info.normal = Vec2(0.0f, 1.0f);
            }

            info.penetration = c.radius;
        }
    }

    return info;
}

static bool PointInsideRect(const Vec2& p, const RectF& r) {
    return p.x >= r.x && p.x <= r.x + r.w && p.y >= r.y && p.y <= r.y + r.h;
}

static ContactInfo HammerTipVsWorld(const Circle& tip, const std::vector<RectF>& world) {
    ContactInfo best;
    float bestPen = -1.0f;

    for (const RectF& rect : world) {
        ContactInfo hit = CircleVsRect(tip, rect);
        if (hit.hit && hit.penetration > bestPen) {
            best = hit;
            bestPen = hit.penetration;
        }
    }

    return best;
}

static void ResolveBodyWorld(Player& player, const std::vector<RectF>& world) {
    constexpr int ITERATIONS = 4;

    for (int i = 0; i < ITERATIONS; ++i) {
        bool anyHit = false;

        for (const RectF& rect : world) {
            Circle body{ player.pos, player.radius };
            ContactInfo hit = CircleVsRect(body, rect);

            if (hit.hit) {
                anyHit = true;
                player.pos += hit.normal * hit.penetration;

                float vn = Dot(player.vel, hit.normal);
                if (vn < 0.0f) {
                    player.vel -= hit.normal * vn;
                }

                if (std::abs(hit.normal.y) > 0.7f) {
                    player.vel.x *= 0.96f;
                }
            }
        }

        if (!anyHit) {
            break;
        }
    }
}

static void BuildLevel(std::vector<RectF>& world) {
    world.clear();

    // floor
    world.push_back({ 0.0f, 680.0f, 1280.0f, 40.0f });

    // left wall
    world.push_back({ -40.0f, 0.0f, 40.0f, 720.0f });

    // sample climbing platforms / obstacles
    world.push_back({ 180.0f, 610.0f, 120.0f, 24.0f });
    world.push_back({ 340.0f, 550.0f, 140.0f, 24.0f });
    world.push_back({ 520.0f, 500.0f, 110.0f, 24.0f });
    world.push_back({ 670.0f, 440.0f, 180.0f, 24.0f });
    world.push_back({ 920.0f, 380.0f, 120.0f, 24.0f });

    // vertical blockers
    world.push_back({ 430.0f, 470.0f, 28.0f, 80.0f });
    world.push_back({ 780.0f, 360.0f, 28.0f, 80.0f });

    // top goal-ish area
    world.push_back({ 1080.0f, 300.0f, 140.0f, 24.0f });

    // extra challenge
    world.push_back({ 1030.0f, 520.0f, 140.0f, 24.0f });
    world.push_back({ 880.0f, 590.0f, 80.0f, 20.0f });
}

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Pot Hammer Physics Prototype",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN
    );

    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(
        window,
        -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );

    if (!renderer) {
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

    const float gravity = 2200.0f;
    const float airDamping = 0.999f;
    const float maxSpeed = 1000.0f;
    const float pushStrength = 2600.0f;
    const float contactBoost = 1.15f;

    Uint64 previousCounter = SDL_GetPerformanceCounter();
    double accumulator = 0.0;

    while (running) {
        Uint64 currentCounter = SDL_GetPerformanceCounter();
        double deltaSeconds = static_cast<double>(currentCounter - previousCounter) /
            static_cast<double>(SDL_GetPerformanceFrequency());
        previousCounter = currentCounter;

        if (deltaSeconds > 0.25) {
            deltaSeconds = 0.25;
        }

        accumulator += deltaSeconds;

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
                if (event.key.keysym.sym == SDLK_r) {
                    player.Reset();
                }
            }
        }

        int mouseX = 0;
        int mouseY = 0;
        Uint32 mouseState = SDL_GetMouseState(&mouseX, &mouseY);
        bool pushing = (mouseState & SDL_BUTTON(SDL_BUTTON_LEFT)) != 0;

        Vec2 mouseWorld(
            static_cast<float>(mouseX) + camera.x,
            static_cast<float>(mouseY) + camera.y
        );

        Vec2 toMouse = mouseWorld - player.pos;
        if (Length(toMouse) > 0.001f) {
            player.hammerAngle = std::atan2(toMouse.y, toMouse.x);
        }

        while (accumulator >= FIXED_DT) {
            player.vel.y += gravity * FIXED_DT;
            player.vel = player.vel * airDamping;

            const Uint8* keys = SDL_GetKeyboardState(nullptr);
            const float moveForce = 1800.0f;

            if (keys[SDL_SCANCODE_A]) {
                player.vel.x -= moveForce * FIXED_DT;
            }
            if (keys[SDL_SCANCODE_D]) {
                player.vel.x += moveForce * FIXED_DT;
            }
            if (keys[SDL_SCANCODE_W]) {
                player.vel.y -= moveForce * FIXED_DT;
            }
            if (keys[SDL_SCANCODE_S]) {
                player.vel.y += moveForce * FIXED_DT;
            }

            // Clamp max speed for stability
            float speed = Length(player.vel);
            if (speed > maxSpeed) {
                player.vel = Normalize(player.vel) * maxSpeed;
            }

            // Hammer reaction force when touching terrain
            Vec2 hammerDir = player.HammerDir();
            Vec2 hammerTip = player.HammerTip();
            Circle tipCollider{ hammerTip, player.hammerTipRadius };
            ContactInfo tipHit = HammerTipVsWorld(tipCollider, world);

            if (pushing && tipHit.hit) {
                // Reaction direction: opposite of hammer direction.
                Vec2 reaction = hammerDir * -1.0f;

                // If normal exists, bias the push to feel stronger when really pressing into a surface.
                float pressFactor = std::max(0.0f, Dot(reaction, tipHit.normal));
                float finalStrength = pushStrength * (1.0f + pressFactor * (contactBoost - 1.0f));

                player.vel += reaction * finalStrength * FIXED_DT;

                // Small separation to reduce sticky contact.
                player.pos += tipHit.normal * (20.0f * FIXED_DT);
            }

            // Integrate motion
            player.pos += player.vel * FIXED_DT;

            // Resolve body collisions
            ResolveBodyWorld(player, world);

            // If player falls too low, respawn
            if (player.pos.y > WINDOW_HEIGHT + 200.0f) {
                player.Reset();
            }

            accumulator -= FIXED_DT;
        }

        camera.x = player.pos.x - WINDOW_WIDTH * 0.35f;
        camera.y = player.pos.y - WINDOW_HEIGHT * 0.65f;

        // Rendering
        SDL_SetRenderDrawColor(renderer, 18, 18, 26, 255);
        SDL_RenderClear(renderer);

        // Draw level
        SDL_SetRenderDrawColor(renderer, 110, 110, 130, 255);
        for (const RectF& rect : world) {
            DrawRect(renderer, rect);
        }

        // Draw hammer line
        Vec2 hammerTip = player.HammerTip();
        Circle tipCollider{ hammerTip, player.hammerTipRadius };
        ContactInfo tipHit = HammerTipVsWorld(tipCollider, world);

        SDL_SetRenderDrawColor(renderer, 230, 230, 230, 255);
        SDL_RenderDrawLine(renderer,
            static_cast<int>(player.pos.x),
            static_cast<int>(player.pos.y),
            static_cast<int>(hammerTip.x),
            static_cast<int>(hammerTip.y));

        // Draw player body
        SDL_SetRenderDrawColor(renderer, 214, 161, 90, 255);
        DrawCircle(renderer,
            static_cast<int>(player.pos.x),
            static_cast<int>(player.pos.y),
            static_cast<int>(player.radius));

        SDL_SetRenderDrawColor(renderer, 60, 35, 18, 255);
        DrawCircleOutline(renderer,
            static_cast<int>(player.pos.x),
            static_cast<int>(player.pos.y),
            static_cast<int>(player.radius));

        // Draw hammer tip
        if (tipHit.hit) {
            SDL_SetRenderDrawColor(renderer, 255, 80, 80, 255);
        }
        else {
            SDL_SetRenderDrawColor(renderer, 90, 200, 255, 255);
        }
        DrawCircle(renderer,
            static_cast<int>(hammerTip.x),
            static_cast<int>(hammerTip.y),
            static_cast<int>(player.hammerTipRadius));

        // Simple spawn marker
        SDL_SetRenderDrawColor(renderer, 80, 220, 120, 255);
        SDL_Rect spawnRect{
            static_cast<int>(player.spawnPos.x - 8),
            static_cast<int>(player.spawnPos.y - 8),
            16,
            16
        };
        SDL_RenderFillRect(renderer, &spawnRect);

        SDL_RenderPresent(renderer);
        SDL_Delay(1000 / TARGET_FPS / 4);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
