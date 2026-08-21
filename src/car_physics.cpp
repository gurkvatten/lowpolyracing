#include "car_physics.h"
#include <cmath>
#include <algorithm>

namespace {

// Förenklad "magic formula" (Pacejka). D = toppkraft (μ * hjullast) sätts
// av anroparen varje frame utifrån aktuell viktöverföring.
float PacejkaLateralForce(float slipAngle, float B, float C, float D, float E) {
    float Bx = B * slipAngle;
    float inner = Bx - E * (Bx - std::atan(Bx));
    return D * std::sin(C * std::atan(inner));
}

float Clamp(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

// Skalar ner (lat, lon) så att den kombinerade kraften ryms inom
// friktionscirkeln (√(lat² + lon²) ≤ maxGrip). Modellerar att fullgas
// eller hård broms äter upp sidogrepp (och tvärtom).
void ApplyFrictionCircle(float& lat, float& lon, float maxGrip) {
    float mag = std::sqrt(lat * lat + lon * lon);
    if (mag > maxGrip && mag > 1e-5f) {
        float scale = maxGrip / mag;
        lat *= scale;
        lon *= scale;
    }
}

} // namespace

CarState MakeDefaultCarState(Vector3 startPosition) {
    CarState s;
    s.position = startPosition;
    return s;
}

void UpdateCarPhysics(CarState& state, const CarConfig& cfg, const CarInput& input, float dt) {
    if (dt <= 0.0f) return;

    const float a = cfg.wheelBase * (1.0f - cfg.frontWeightFraction); // CoM -> framaxel
    const float b = cfg.wheelBase * cfg.frontWeightFraction;          // CoM -> bakaxel
    const float L = cfg.wheelBase;

    // Boxapproximation av girtröghetsmoment.
    const float yawInertia = cfg.mass * (L * L + cfg.trackWidth * cfg.trackWidth) / 12.0f;

    float vx = state.velocityLocalX;
    float vz = state.velocityLocalZ;
    float omega = state.yawRate;

    // --- Styrvinkel: utjämnad mot mål, med fartkänslig maxvinkel ---
    float speed = std::sqrt(vx * vx + vz * vz);
    float speedFactor = Clamp(speed / 30.0f, 0.0f, 1.0f);
    float maxSteerDeg = cfg.maxSteerAngleDeg + (cfg.minSteerAngleDeg - cfg.maxSteerAngleDeg) * speedFactor;
    float targetSteer = Clamp(input.steer, -1.0f, 1.0f) * (maxSteerDeg * DEG2RAD);
    float steerDelta = targetSteer - state.steerAngle;
    float maxStep = cfg.steerRate * dt;
    state.steerAngle += Clamp(steerDelta, -maxStep, maxStep);
    float delta = state.steerAngle;

    // --- Statisk hjullast + longitudinell viktöverföring ---
    float staticFrontLoad = cfg.mass * cfg.gravity * cfg.frontWeightFraction;
    float staticRearLoad = cfg.mass * cfg.gravity * (1.0f - cfg.frontWeightFraction);
    // Använder föregående frames längsgående acceleration (undviker algebraisk loop).
    float weightTransfer = cfg.mass * state.lastLongitudinalAccel * cfg.cgHeight / L;
    float frontLoad = std::max(0.0f, staticFrontLoad - weightTransfer);
    float rearLoad = std::max(0.0f, staticRearLoad + weightTransfer);

    // --- Slip-vinklar (se härledning i README: hastighet vid axel = v_com + ω×r) ---
    const float lowSpeedEps = 1.5f; // m/s, undviker atan2-brus vid stillastående
    float vzSafe = std::fabs(vz) < lowSpeedEps ? (vz >= 0 ? lowSpeedEps : -lowSpeedEps) : vz;

    float vxFront = vx + omega * a;
    float vxRear = vx - omega * b;

    float slipFront = delta - std::atan2(vxFront, vzSafe);
    float slipRear = -std::atan2(vxRear, vzSafe);

    // Vid mycket låg fart har lateral friktion inte mycket att "greppa i" ännu
    // (hjulet är i praktiken stillastående) - dämpa bort slip-kraften för att
    // undvika att bilen "vibrerar" i stillastående läge.
    float lowSpeedBlend = Clamp(speed / lowSpeedEps, 0.0f, 1.0f);

    float gripFront = cfg.tireFrictionFront * frontLoad;
    float gripRear = cfg.tireFrictionRear * rearLoad;

    float latFront = PacejkaLateralForce(slipFront, cfg.tireStiffnessB, cfg.tireShapeC, gripFront, cfg.tireCurvatureE) * lowSpeedBlend;
    float latRear = PacejkaLateralForce(slipRear, cfg.tireStiffnessB, cfg.tireShapeC, gripRear, cfg.tireCurvatureE) * lowSpeedBlend;

    // --- Längsgående kraft: motor + broms, fördelat per axel ---
    // Negativt throttle = back (svagare kraft, som en verklig backväxel).
    float throttle = Clamp(input.throttle, -1.0f, 1.0f);
    float brake = Clamp(input.brake, 0.0f, 1.0f);
    if (input.handbrake) brake = 1.0f;

    // Motorn är både vridmoment- och effektbegränsad: vid låg fart är kraften
    // konstant (rawTorqueForce), men vid hög fart tar effekttaket över
    // (P = F*v => F = P/v), precis som en riktig motor tappar dragkraft
    // ju fortare man kör. Utan detta blir toppfarten orealistiskt hög.
    float rawTorqueForce = std::fabs(throttle) * cfg.maxEngineForce * (throttle >= 0.0f ? 1.0f : 0.5f);
    float speedForPower = std::max(std::fabs(vz), 3.0f); // undvik division nära 0 vid start
    float powerLimitedForce = cfg.maxEnginePower / speedForPower;
    float driveMagnitude = std::min(rawTorqueForce, powerLimitedForce);
    float driveForce = (throttle >= 0.0f ? 1.0f : -1.0f) * driveMagnitude;
    float driveFront = driveForce * (1.0f - cfg.driveBiasRear);
    float driveRear = driveForce * cfg.driveBiasRear;

    float brakeForceTotal = brake * cfg.maxBrakeForce;
    float vzSign = (vz > 0.05f) ? 1.0f : (vz < -0.05f ? -1.0f : 0.0f);
    float brakeFront = brakeForceTotal * cfg.brakeBiasFront * vzSign;
    float brakeRear = brakeForceTotal * (1.0f - cfg.brakeBiasFront) * vzSign;
    if (input.handbrake) {
        // Handbroms låser bara bakhjulen, ingen effekt fram.
        brakeFront = 0.0f;
        brakeRear = brakeForceTotal * vzSign;
    }

    float lonFront = driveFront - brakeFront;
    float lonRear = driveRear - brakeRear;

    // --- Friktionscirkel per axel (fullgas/hård broms äter sidogrepp) ---
    ApplyFrictionCircle(latFront, lonFront, gripFront);
    ApplyFrictionCircle(latRear, lonRear, gripRear);

    // --- Motstånd: luft (kvadratiskt) + rullmotstånd ---
    float dragForce = -cfg.dragCoeff * vz * std::fabs(vz);
    float rollingForce = -cfg.rollingResistanceCoeff * (frontLoad + rearLoad) * (vzSign != 0.0f ? vzSign : 0.0f);

    // --- Summera krafter och moment i bilens lokala koordinatsystem ---
    float Fx = latFront + latRear;                        // lateral (sido-)kraft
    float Fz = lonFront + lonRear + dragForce + rollingForce; // längsgående kraft
    float Mz = a * latFront - b * latRear;                 // girmoment

    // Rörelseekvationer i roterande kroppsfast system (se README för härledning):
    //   dvx/dt = Fx/m - ω*vz   (Coriolis-koppling)
    //   dvz/dt = Fz/m + ω*vx
    float ax = Fx / cfg.mass - omega * vz;
    float az = Fz / cfg.mass + omega * vx;
    float angularAccel = Mz / yawInertia;

    vx += ax * dt;
    vz += az * dt;
    omega += angularAccel * dt;

    // Om bilen bromsar till nästan stillastående, undvik att bromsen knuffar
    // bilen baklänges (broms ska bara kunna bromsa till 0, inte reversera).
    if (brake > 0.0f && !input.handbrake) {
        if (vzSign > 0.0f && vz < 0.0f) vz = 0.0f;
        if (vzSign < 0.0f && vz > 0.0f) vz = 0.0f;
    }

    state.velocityLocalX = vx;
    state.velocityLocalZ = vz;
    state.yawRate = omega;
    state.lastLongitudinalAccel = az - omega * vx; // "ren" längsgående accel (utan Coriolis-term), för viktöverföring nästa frame
    state.lastFrontLoad = frontLoad;
    state.lastRearLoad = rearLoad;

    state.heading += omega * dt;

    // --- Integrera world-position ---
    Vector3 worldVel = GetCarWorldVelocity(state);
    state.position.x += worldVel.x * dt;
    state.position.z += worldVel.z * dt;

    // --- Kosmetisk lut för rendering (inte del av rigid-body-simuleringen) ---
    float targetRoll = Clamp(-Fx / (cfg.mass * cfg.gravity), -0.5f, 0.5f) * 0.14f;
    float targetPitch = Clamp(-az / cfg.gravity, -1.0f, 1.0f) * 0.08f;
    state.visualRoll += (targetRoll - state.visualRoll) * Clamp(8.0f * dt, 0.0f, 1.0f);
    state.visualPitch += (targetPitch - state.visualPitch) * Clamp(8.0f * dt, 0.0f, 1.0f);
}

Vector3 GetCarWorldVelocity(const CarState& state) {
    float h = state.heading;
    // forward = (sin h, 0, cos h), right = (cos h, 0, -sin h) - se main.cpp/README.
    Vector3 forward = { std::sin(h), 0.0f, std::cos(h) };
    Vector3 right = { std::cos(h), 0.0f, -std::sin(h) };
    return {
        forward.x * state.velocityLocalZ + right.x * state.velocityLocalX,
        0.0f,
        forward.z * state.velocityLocalZ + right.z * state.velocityLocalX
    };
}

float GetCarForwardSpeed(const CarState& state) {
    return state.velocityLocalZ;
}
