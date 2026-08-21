#pragma once

#if defined(__has_include)
    #if __has_include("raylib.h")
        #include "raylib.h"
    #elif __has_include(<raylib.h>)
        #include <raylib.h>
    #else
        #error "raylib.h not found."
    #endif
#else
    #include "raylib.h"
#endif

// Enkel "bicycle model" (fram-/bakaxel slås ihop till en punkt vardera) med
// slip-vinkel-baserad däckkraft (magic-formula/Pacejka-approximation),
// viktöverföring och friktionscirkel. Ger realistisk översteer/understeer-
// känsla utan att behöva simulera fyra oberoende hjul/fjädring.
//
// Nästa steg om man vill gå djupare: fyra oberoende hjul med egen
// fjädring/raycast mot marken, slip-ratio-baserad längsgående däckkraft
// (kräver hjul-RPM/moment), samt en riktig motor/växellåda-modell.

struct CarConfig {
    float mass = 1200.0f;              // kg
    float wheelBase = 2.6f;            // m, avstånd fram-/bakaxel
    float frontWeightFraction = 0.5f;  // andel av statisk vikt på framaxeln
    float cgHeight = 0.5f;             // m, tyngdpunktens höjd över marken
    float trackWidth = 1.9f;           // m, spårvidd (fram/bak-hjulens avstånd, styr även visning)

    // Pacejka-liknande däckkurva: D (topkraft) sätts dynamiskt varje frame
    // till muFriktion * hjullast, så kurvformen (B, C, E) är gemensam.
    float tireFrictionFront = 1.05f;   // μ, framdäck
    float tireFrictionRear = 1.05f;    // μ, bakdäck
    float tireStiffnessB = 8.0f;
    float tireShapeC = 1.6f;
    float tireCurvatureE = 0.97f;

    float maxEngineForce = 8000.0f;    // N, vridmomentgräns (lågfart/start)
    float maxEnginePower = 130000.0f;  // W, effektgräns (~174 hk) - begränsar kraften vid hög fart
    float maxBrakeForce = 12000.0f;    // N, totalt över bägge axlar
    float driveBiasRear = 1.0f;        // 1 = RWD, 0 = FWD, 0.5 = AWD
    float brakeBiasFront = 0.6f;       // andel bromskraft fram

    float dragCoeff = 0.42f;           // kvadratiskt luftmotstånd
    float rollingResistanceCoeff = 0.015f; // rullmotstånd (andel av last)

    float maxSteerAngleDeg = 35.0f;    // vid stillastående
    float minSteerAngleDeg = 8.0f;     // vid hög fart (fartkänslig styrning)
    float steerRate = 4.0f;            // rad/s, hur snabbt styrvinkeln följer input

    float gravity = 9.81f;
};

struct CarInput {
    float throttle = 0.0f;  // 0..1
    float brake = 0.0f;     // 0..1
    float steer = 0.0f;     // -1..1 (positivt = höger)
    bool handbrake = false;
};

struct CarState {
    Vector3 position = { 0.0f, 0.0f, 0.0f };
    float heading = 0.0f;      // rad, world yaw
    float yawRate = 0.0f;      // rad/s

    // Hastighet i bilens eget koordinatsystem (roterar med heading).
    float velocityLocalX = 0.0f; // lateral (positiv = höger)
    float velocityLocalZ = 0.0f; // längsgående (positiv = framåt)

    float steerAngle = 0.0f;   // aktuell (utjämnad) styrvinkel, rad

    // Rent kosmetiska värden för fjädrings-/lut-effekt vid rendering.
    float visualRoll = 0.0f;
    float visualPitch = 0.0f;

    // Senaste beräknade last per axel (N), exponerad för debug/HUD.
    float lastFrontLoad = 0.0f;
    float lastRearLoad = 0.0f;
    float lastLongitudinalAccel = 0.0f;
};

CarState MakeDefaultCarState(Vector3 startPosition);

// Kör en fysik-substep. dt bör vara liten (se FixedUpdateCarPhysics för
// hur man delar upp ett större frame-dt i flera substeps).
void UpdateCarPhysics(CarState& state, const CarConfig& config, const CarInput& input, float dt);

// World-space hastighetsvektor, härledd från lokal hastighet + heading.
Vector3 GetCarWorldVelocity(const CarState& state);

// Fart längs bilens egen färdriktning (m/s), positiv = framåt.
float GetCarForwardSpeed(const CarState& state);
