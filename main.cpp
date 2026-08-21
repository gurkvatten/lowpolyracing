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

#include <cmath> // För sin() och cos()

int main() {
    InitWindow(1280, 720, "Low Poly Racing Prototype");

    // --- BILENS TILLSTÅND ---
    Vector3 carPosition = { 0.0f, 0.5f, 0.0f };
    float carRotation = 0.0f;  // Vinkel i grader
    float speed = 0.0f;

    // --- KONSTANTER FÖR HANTERING ---
    const float acceleration = 15.0f;
    const float maxSpeed = 25.0f;
    const float friction = 8.0f;      // Markmotstånd som sänker farten
    const float turnSpeed = 120.0f;    // Hur snabbt bilen svänger (grader/sek)

    Camera3D camera = { 0 };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        // deltaTime gör att rörelsen blir lika snabb oavsett bilduppdateringsfrekvens
        float dt = GetFrameTime();

        // 1. HANTERA INDATA (STYRNING & GAS)
        if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) {
            speed += acceleration * dt;
        } else if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) {
            speed -= acceleration * dt;
        } else {
            // Friktion: Minska hastigheten gradvis mot 0 när man inte gasar
            if (speed > 0) {
                speed -= friction * dt;
                if (speed < 0) speed = 0;
            } else if (speed < 0) {
                speed += friction * dt;
                if (speed > 0) speed = 0;
            }
        }

        // Begränsa maxhastighet (framåt och bakåt)
        if (speed > maxSpeed) speed = maxSpeed;
        if (speed < -maxSpeed / 2.0f) speed = -maxSpeed / 2.0f;

        // Sväng endast om bilen faktiskt rör sig
        if (std::abs(speed) > 0.1f) {
            float dir = (speed > 0) ? 1.0f : -1.0f; // Sväng åt rätt håll om vi backar
            if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) carRotation += turnSpeed * dir * dt;
            if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) carRotation -= turnSpeed * dir * dt;
        }

        // 2. UPPDATERA POSITION
        // Omvandla vinkel till radianer för trig-beräkning
        float radians = carRotation * (PI / 180.0f);
        carPosition.x += std::sin(radians) * speed * dt;
        carPosition.z += std::cos(radians) * speed * dt;

        // 3. KAMERA SOM FÖLJER BILEN
        // Placera kameran bakom och ovanför bilen baserat på bilens rotation
        float cameraDistance = 8.0f;
        float cameraHeight = 3.5f;

        camera.position.x = carPosition.x - std::sin(radians) * cameraDistance;
        camera.position.y = carPosition.y + cameraHeight;
        camera.position.z = carPosition.z - std::cos(radians) * cameraDistance;
        camera.target = carPosition; // Kameran titta alltid på bilen

        // 4. RITA SCENEN
        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);

                // Rita bilen
                DrawCube(
                    carPosition,
                    2.0f, 1.0f, 4.0f,
                    RED
                );

                // Rita kontur på bilen så man ser riktningen tydligare
                DrawCubeWires(
                    carPosition,
                    2.0f, 1.0f, 4.0f,
                    DARKGRAY
                );

                DrawGrid(40, 1.0f); // Markstöd

            EndMode3D();

            DrawText("Styr med WASD eller Piltangenterna", 10, 10, 20, DARKGRAY);
            DrawFPS(10, 40);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}