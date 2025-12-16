#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_Sensor.h>
#include <BleMouse.h>
#include <math.h>

#define SDA_PIN 21
#define SCL_PIN 22

#define IMU_ADDR_A 0x6A   // SA0/SDO = GND
#define IMU_ADDR_B 0x6B   // SA0/SDO = 3V3

Adafruit_LSM6DS3 imuA;
Adafruit_LSM6DS3 imuB;

BleMouse bleMouse("AIR", "ESP32", 100);

// ================== BLE flood protection ==================
static const uint32_t REPORT_MS  = 8;    // ~125 Hz; higher rates can overflow BLE notify
static const uint32_t WARMUP_MS  = 600;  // wait after connect for CCC/ready
static const uint32_t IDLE_MS    = 5;

uint32_t lastReportMs = 0;
uint32_t connectedSinceMs = 0;
bool wasConnected = false;

// Accumulate fractional motion; only send at REPORT_MS cadence
float accDx = 0.0f;
float accDy = 0.0f;

// ================== IMU + cursor tunables ==================
#ifdef LSM6DS_GYRO_RANGE_125_DPS
static const float IMU_B_RANGE_DPS  = 125.0f;   // low-range / high precision
#else
static const float IMU_B_RANGE_DPS  = 250.0f;   // fallback if 125 DPS not supported
#endif
static const float DEADZONE_DPS     = 0.0f;     // deadzone disabled
static const float POST_DZ_DPS_Y    = 0.0f;     // deadzone disabled
static const float POST_DZ_DPS_Z    = 0.0f;     // deadzone disabled
static const float ALPHA_GYRO       = 0.25f;    // slower filter for smoother motion
static const float ALPHA_TILT       = 0.08f;
static const float BASE_SENS        = 0.002832f; // pow(2, -9) * 1.45 from reference impl
static const float SENS_MULT        = 0.3f;      // overall speed multiplier (more subtle)
static const float HSSF_T           = 1.0f;      // near-zero softening threshold
static const float HSSF_K           = 0.5f;      // near-zero softening strength
static const float BLEND_Z          = 0.2f;      // blend window for dual-IMU fusion
static const int   LOOP_DELAY_MS    = 2;

// Stationary bias adaptation
static const float STILL_DPS      = 1.0f;
static const uint32_t STILL_MS    = 500;
static const float BIAS_BETA      = 0.002f;

// Bias per-IMU (deg/s)
float bxA = 0, byA = 0, bzA = 0;
float bxB = 0, byB = 0, bzB = 0;

// Filtered tilt (rad)
float roll_f  = 0.0f;
float pitch_f = 0.0f;

// Filtered rates after tilt-compensation (deg/s)
float fgy = 0.0f;   // Y for up/down
float fgz = 0.0f;   // Z for left/right

static inline float rad2deg(float r) { return r * 57.2957795f; }
static inline float clamp01(float v) { return fminf(fmaxf(v, 0.0f), 1.0f); }
static inline float deadzone(float v, float dz) { return (fabsf(v) < dz) ? 0.0f : v; }
static inline float rampMid(float x, float z) {
  if (x < z) return 0.0f;
  if (x > 1.0f - z) return 1.0f;
  return (x - z) * (1.0f / (1.0f - 2.0f * z));
}
static inline float hssnf(float t, float k, float x) {
  float a = x - (x * k);
  float b = 1.0f - (x * k * (1.0f / t));
  return (b != 0.0f) ? (a / b) : 0.0f;
}
static inline float blendHighRange(float magDps, float start, float end) {
  float t = (magDps - start) / (end - start);
  return clamp01(t);
}
static inline float softenSmallMotion(float v) {
  float s = (v >= 0.0f) ? 1.0f : -1.0f;
  float mag = fabsf(v);
  if (mag >= HSSF_T) return v;
  return s * hssnf(HSSF_T, HSSF_K, mag);
}

void calibrateGyroBias(Adafruit_LSM6DS3 &imu, float &bx, float &by, float &bz,
                       const char* name, int samples = 800, int sampleDelayMs = 2) {
  Serial.printf("Calibrating %s gyro bias... keep STILL.\n", name);
  float sx = 0, sy = 0, sz = 0;
  sensors_event_t accel, gyro, temp;

  for (int i = 0; i < samples; i++) {
    imu.getEvent(&accel, &gyro, &temp);
    sx += rad2deg(gyro.gyro.x);
    sy += rad2deg(gyro.gyro.y);
    sz += rad2deg(gyro.gyro.z);
    delay(sampleDelayMs);
  }
  bx = sx / samples;
  by = sy / samples;
  bz = sz / samples;

  Serial.printf("%s bias (dps): bx=%.3f by=%.3f bz=%.3f\n", name, bx, by, bz);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);

  bool okA = imuA.begin_I2C(IMU_ADDR_A, &Wire);
  bool okB = imuB.begin_I2C(IMU_ADDR_B, &Wire);

  Serial.printf("IMU A (0x%02X): %s\n", IMU_ADDR_A, okA ? "OK" : "FAIL");
  Serial.printf("IMU B (0x%02X): %s\n", IMU_ADDR_B, okB ? "OK" : "FAIL");

  if (!okA || !okB) {
    Serial.println("Fix wiring: SA0/SDO must be different, CS must be 3V3, power lines solid.");
    while (1) delay(100);
  }

  // Configure BOTH sensors
  imuA.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  imuA.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imuA.setGyroDataRate(LSM6DS_RATE_208_HZ);
  imuA.setAccelDataRate(LSM6DS_RATE_208_HZ);

#ifdef LSM6DS_GYRO_RANGE_125_DPS
  imuB.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);   // low-range, low-noise leg
#else
  imuB.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);   // fallback when 125 DPS is unavailable
#endif
  imuB.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imuB.setGyroDataRate(LSM6DS_RATE_208_HZ);
  imuB.setAccelDataRate(LSM6DS_RATE_208_HZ);

  // Calibrate bias for BOTH
  calibrateGyroBias(imuA, bxA, byA, bzA, "IMU A");
  calibrateGyroBias(imuB, bxB, byB, bzB, "IMU B");

  bleMouse.begin();
  Serial.println("BLE mouse started. Pair 'AIR' on your PC.");
}

void loop() {
  // --- connection state tracking ---
  bool connected = bleMouse.isConnected();
  uint32_t nowMs = millis();

  if (connected && !wasConnected) {
    connectedSinceMs = nowMs;
    accDx = 0; accDy = 0;
  }

  if (!connected) {
    wasConnected = false;
    accDx = 0; accDy = 0;
    delay(IDLE_MS);
    return;
  }
  wasConnected = true;

  if (nowMs - connectedSinceMs < WARMUP_MS) {
    delay(2);
    return;
  }

  // --- read BOTH IMUs ---
  sensors_event_t accelA, gyroA, tempA;
  sensors_event_t accelB, gyroB, tempB;
  imuA.getEvent(&accelA, &gyroA, &tempA);
  imuB.getEvent(&accelB, &gyroB, &tempB);

  // ========== Fuse accel (average) for tilt ==========
  float ax = 0.5f * (accelA.acceleration.x + accelB.acceleration.x);
  float ay = 0.5f * (accelA.acceleration.y + accelB.acceleration.y);
  float az = 0.5f * (accelA.acceleration.z + accelB.acceleration.z);

  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

  roll_f  = ALPHA_TILT * roll  + (1.0f - ALPHA_TILT) * roll_f;
  pitch_f = ALPHA_TILT * pitch + (1.0f - ALPHA_TILT) * pitch_f;

  // ========== Gyro raw (deg/s) per IMU ==========
  float gxA_raw = rad2deg(gyroA.gyro.x);
  float gyA_raw = rad2deg(gyroA.gyro.y);
  float gzA_raw = rad2deg(gyroA.gyro.z);

  float gxB_raw = rad2deg(gyroB.gyro.x);
  float gyB_raw = rad2deg(gyroB.gyro.y);
  float gzB_raw = rad2deg(gyroB.gyro.z);

  // Remove per-IMU bias.
  float gxA = gxA_raw - bxA;
  float gyA = gyA_raw - byA;
  float gzA = gzA_raw - bzA;
  float gxB = gxB_raw - bxB;
  float gyB = gyB_raw - byB;
  float gzB = gzB_raw - bzB;

  // Dual-IMU fusion inspired by reference library: trust the low-range sensor
  // for subtle motion and only introduce the high-range leg as B approaches
  // its limits.
  float magB = fmaxf(fabsf(gxB_raw), fmaxf(fabsf(gyB_raw), fabsf(gzB_raw)));
  float blendStart = 0.45f * IMU_B_RANGE_DPS;   // keep B dominant below ~45% of range
  float blendEnd   = 0.80f * IMU_B_RANGE_DPS;   // start sliding to A beyond this
  float weightA = blendHighRange(magB, blendStart, blendEnd); // high-range blend factor
  float weightB = 1.0f - weightA;                               // low-range blend factor
  // Keep some low-range contribution even when moving fast so sensitivity stays up, but soften the floor.
  const float weightBMin = 0.25f;
  if (weightB < weightBMin) {
    weightB = weightBMin;
    weightA = 1.0f - weightB;
  }

  float gx = (gxA * weightA) + (gxB * weightB);
  float gy = (gyA * weightA) + (gyB * weightB);
  float gz = (gzA * weightA) + (gzB * weightB);
  float countsPerDps = (32768.0f / 500.0f) * weightA + (32768.0f / IMU_B_RANGE_DPS) * weightB;

  // ========== Auto-bias learning when still (update BOTH biases) ==========
  static uint32_t stillStart = 0;
  bool still = (fabsf(gx) < STILL_DPS && fabsf(gy) < STILL_DPS && fabsf(gz) < STILL_DPS);

  if (still) {
    if (stillStart == 0) stillStart = nowMs;
    if (nowMs - stillStart > STILL_MS) {
      // learn bias toward each sensor’s raw reading
      bxA = (1.0f - BIAS_BETA) * bxA + BIAS_BETA * gxA_raw;
      byA = (1.0f - BIAS_BETA) * byA + BIAS_BETA * gyA_raw;
      bzA = (1.0f - BIAS_BETA) * bzA + BIAS_BETA * gzA_raw;

      bxB = (1.0f - BIAS_BETA) * bxB + BIAS_BETA * gxB_raw;
      byB = (1.0f - BIAS_BETA) * byB + BIAS_BETA * gyB_raw;
      bzB = (1.0f - BIAS_BETA) * bzB + BIAS_BETA * gzB_raw;
    }
  } else {
    stillStart = 0;
  }

  // ========== Tilt-compensate fused gyro (rotate by roll/pitch) ==========
  float sr = sinf(roll_f),  cr = cosf(roll_f);
  float sp = sinf(pitch_f), cp = cosf(pitch_f);

  // roll about X
  float gx2 = gx;
  float gy2 = gy * cr - gz * sr;
  float gz2 = gy * sr + gz * cr;

  // pitch about Y
  float gx3 = gx2 * cp + gz2 * sp;
  float gy3 = gy2;
  float gz3 = -gx2 * sp + gz2 * cp;

  // low-pass the rates we use for cursor
  fgy = ALPHA_GYRO * gy3 + (1.0f - ALPHA_GYRO) * fgy;
  fgz = ALPHA_GYRO * gz3 + (1.0f - ALPHA_GYRO) * fgz;

  // ========== Map to cursor ==========
  float dx_f =  fgz * BASE_SENS * SENS_MULT * countsPerDps;
  float dy_f = -fgy * BASE_SENS * SENS_MULT * countsPerDps;

  dx_f = softenSmallMotion(dx_f);
  dy_f = softenSmallMotion(dy_f);

  accDx += dx_f;
  accDy += dy_f;

  // Notify at safe cadence
  if (nowMs - lastReportMs >= REPORT_MS) {
    lastReportMs = nowMs;

    int dx = (int)lroundf(accDx);
    int dy = (int)lroundf(accDy);

    accDx -= dx;
    accDy -= dy;

    if (dx == 0 && dy == 0) {
      delay(LOOP_DELAY_MS);
      return;
    }

    dx = constrain(dx, -127, 127);
    dy = constrain(dy, -127, 127);

    if (!bleMouse.isConnected()) {
      accDx = accDy = 0;
      return;
    }

    bleMouse.move(-dx, -dy);
  }

  delay(LOOP_DELAY_MS);
}
