#if defined(__has_include)
    #if __has_include("raylib.h")
        #include "raylib.h"
    #elif __has_include(<raylib.h>)
        #include <raylib.h>
    #elif __has_include("/opt/homebrew/include/raylib.h")
        #include "/opt/homebrew/include/raylib.h"
    #elif __has_include("/usr/local/include/raylib.h")
        #include "/usr/local/include/raylib.h"
    #else
        #error "raylib.h not found. Add the raylib include directory to your project/compiler settings."
    #endif
#else
    #include "raylib.h"
#endif

#include "rlgl.h"
#include "car_physics.h"

#include <cmath>

namespace {

// Rita en låda med egen yaw/pitch/roll (raylibs DrawCube stödjer bara
// axel-parallella lådor, så vi lägger på en egen matris-transform).
void DrawTiltedBox(Vector3 position, float yawRad, float pitchRad, float rollRad,
                    float sx, float sy, float sz, Color color, Color wireColor) {
    rlPushMatrix();
        rlTranslatef(position.x, position.y, position.z);
        rlRotatef(yawRad * RAD2DEG, 0.0f, 1.0f, 0.0f);
        rlRotatef(pitchRad * RAD2DEG, 1.0f, 0.0f, 0.0f);
        rlRotatef(rollRad * RAD2DEG, 0.0f, 0.0f, 1.0f);
        DrawCube(Vector3{0, 0, 0}, sx, sy, sz, color);
        DrawCubeWires(Vector3{0, 0, 0}, sx, sy, sz, wireColor);
    rlPopMatrix();
}

// Ett hjul: liten svart låda, roterad för styrvinkel (endast fram).
void DrawWheel(Vector3 carPosition, float carYawRad, float localX, float localZ,
               float extraYawRad) {
    float forwardX = std::sin(carYawRad);
    float forwardZ = std::cos(carYawRad);
    float rightX = std::cos(carYawRad);
    float rightZ = -std::sin(carYawRad);

    Vector3 pos = {
        carPosition.x + rightX * localX + forwardX * localZ,
        carPosition.y,
        carPosition.z + rightZ * localX + forwardZ * localZ
    };

    rlPushMatrix();
        rlTranslatef(pos.x, pos.y, pos.z);
        rlRotatef((carYawRad + extraYawRad) * RAD2DEG, 0.0f, 1.0f, 0.0f);
        DrawCube(Vector3{0, 0, 0}, 0.35f, 0.6f, 0.7f, DARKGRAY);
    rlPopMatrix();
}

} // namespace

int main() {
    InitWindow(1280, 720, "Low Poly Racing - Realistic Sim Prototype");

    CarConfig config;
    CarState car = MakeDefaultCarState(Vector3{0.0f, 0.0f, 0.0f});

    Camera3D camera = { 0 };
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
    camera.fovy = 55.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    camera.position = Vector3{ 0.0f, 4.0f, -8.0f };
    camera.target = car.position;

    SetTargetFPS(60);

    // Fast fysik-tidssteg för stabil simulering oavsett bildfrekvens.
    const float fixedDt = 1.0f / 120.0f;
    float accumulator = 0.0f;

    Vector3 smoothedCameraPos = camera.position;

    while (!WindowShouldClose()) {
        float frameDt = GetFrameTime();
        if (frameDt > 0.25f) frameDt = 0.25f; // undvik "spiral of death" efter t.ex. fönsterflytt

        // --- Indata -> CarInput ---
        CarInput input;
        bool wantForward = IsKeyDown(KEY_W) || IsKeyDown(KEY_UP);
        bool wantBack = IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN);
        float forwardSpeed = GetCarForwardSpeed(car);

        if (wantForward) {
            input.throttle = 1.0f;
        } else if (wantBack) {
            if (forwardSpeed > 0.5f) {
                input.brake = 1.0f; // bromsa tills stillastående
            } else {
                input.throttle = -0.6f; // sedan backa
            }
        }

        float steer = 0.0f;
        if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) steer -= 1.0f;
        if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) steer += 1.0f;
        input.steer = steer;
        input.handbrake = IsKeyDown(KEY_SPACE);

        // --- Fysik i fasta delsteg ---
        accumulator += frameDt;
        int safetyIterations = 0;
        while (accumulator >= fixedDt && safetyIterations < 10) {
            UpdateCarPhysics(car, config, input, fixedDt);
            accumulator -= fixedDt;
            safetyIterations++;
        }

        // --- Kamera: följer bakom bilen, mjukt utjämnad ---
        float cameraDistance = 8.0f;
        float cameraHeight = 3.2f;
        float h = car.heading;
        Vector3 desiredCameraPos = {
            car.position.x - std::sin(h) * cameraDistance,
            car.position.y + cameraHeight,
            car.position.z - std::cos(h) * cameraDistance
        };
        float camLerp = 1.0f - std::exp(-10.0f * frameDt);
        smoothedCameraPos.x += (desiredCameraPos.x - smoothedCameraPos.x) * camLerp;
        smoothedCameraPos.y += (desiredCameraPos.y - smoothedCameraPos.y) * camLerp;
        smoothedCameraPos.z += (desiredCameraPos.z - smoothedCameraPos.z) * camLerp;
        camera.position = smoothedCameraPos;
        camera.target = Vector3{ car.position.x, car.position.y + 0.6f, car.position.z };

        // --- Rita ---
        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                DrawGrid(80, 2.0f);

                DrawTiltedBox(
                    Vector3{ car.position.x, car.position.y + 0.5f, car.position.z },
                    car.heading, car.visualPitch, car.visualRoll,
                    1.8f, 1.0f, 4.2f, RED, MAROON
                );

                float half = config.trackWidth / 2.0f;
                float a = config.wheelBase * (1.0f - config.frontWeightFraction);
                float b = config.wheelBase * config.frontWeightFraction;
                DrawWheel(car.position, car.heading, -half, a, car.steerAngle);
                DrawWheel(car.position, car.heading, half, a, car.steerAngle);
                DrawWheel(car.position, car.heading, -half, -b, 0.0f);
                DrawWheel(car.position, car.heading, half, -b, 0.0f);

            EndMode3D();

            float kmh = std::fabs(GetCarForwardSpeed(car)) * 3.6f;
            DrawText(TextFormat("%.0f km/h", kmh), 10, 10, 30, DARKGRAY);
            DrawText("WASD/Pilar: gas/broms/styr  |  Mellanslag: handbroms", 10, 50, 18, GRAY);
            DrawText(TextFormat("Framlast: %.0f N   Baklast: %.0f N", car.lastFrontLoad, car.lastRearLoad), 10, 75, 18, GRAY);
            DrawFPS(10, 100);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
