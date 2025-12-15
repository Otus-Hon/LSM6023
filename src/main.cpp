#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_Sensor.h>
#include <BleMouse.h>
#include <math.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define IMU_ADDR 0x6A

Adafruit_LSM6DS3 imu;
BleMouse bleMouse("AIR", "ESP32", 100);

// ================== BLE flood protection ==================
static const uint32_t REPORT_MS  = 20;   // slow to ~50 Hz to avoid notify spam errors
static const uint32_t WARMUP_MS  = 600;  // wait after connect for CCC/ready
static const uint32_t IDLE_MS    = 5;    // small sleep when not connected

uint32_t lastReportMs = 0;
uint32_t connectedSinceMs = 0;
bool wasConnected = false;

// Accumulate fractional motion; only send at REPORT_MS cadence
float accDx = 0.0f;
float accDy = 0.0f;

// ================== IMU + cursor tunables ==================
static const float DEADZONE_DPS     = 1.6f;   // pre-tilt deadzone (raw gyro after bias)
static const float POST_DZ_DPS_Y    = 3.2f;   // post-tilt deadzone for Y (up/down) to kill gravity creep
static const float POST_DZ_DPS_Z    = 2.2f;   // post-tilt deadzone for Z (left/right)
static const float ALPHA_GYRO       = 0.35f;  // low-pass for gyro rate
static const float ALPHA_TILT       = 0.08f;  // low-pass for accel tilt (gravity)
static const float GAIN_Y           = 0.09f;  // per-axis gains: smaller Y to resist unwanted upward drift
static const float GAIN_Z           = 0.12f;
static const int   LOOP_DELAY_MS = 2;      // sensor loop pace (does NOT equal report rate)

// Stationary bias adaptation (temperature drift cancellation)
static const float STILL_DPS      = 1.0f;   // threshold for "not rotating"
static const uint32_t STILL_MS    = 500;    // must be still this long to start learning bias
static const float BIAS_BETA      = 0.002f; // slow learning rate; smaller = safer

// Bias (deg/s)
float bx = 0, by = 0, bz = 0;

// Filtered tilt (rad)
float roll_f  = 0.0f;
float pitch_f = 0.0f;

// Filtered rates after tilt-compensation (deg/s)
float fgy = 0.0f;   // Y for up/down
float fgz = 0.0f;   // Z for left/right

static inline float rad2deg(float r) { return r * 57.2957795f; }

static inline float deadzone(float v, float dz) {
  return (fabsf(v) < dz) ? 0.0f : v;
}

void calibrateGyroBias(int samples = 800, int sampleDelayMs = 2) {
  Serial.println("Calibrating gyro bias... keep the sensor still.");
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

  Serial.printf("Bias (dps): bx=%.3f by=%.3f bz=%.3f\n", bx, by, bz);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!imu.begin_I2C(IMU_ADDR, &Wire)) {
    Serial.println("LSM6DS3 not found at 0x6A");
    while (1) delay(100);
  }
  Serial.println("LSM6DS3 OK");

  imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imu.setGyroDataRate(LSM6DS_RATE_208_HZ);
  imu.setAccelDataRate(LSM6DS_RATE_208_HZ);

  calibrateGyroBias();

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

  // Don’t notify too early; host may not have enabled notifications yet
  if (nowMs - connectedSinceMs < WARMUP_MS) {
    delay(2);
    return;
  }

  // --- read IMU ---
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);

  // ========== Accelerometer tilt (roll/pitch) ==========
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

  roll_f  = ALPHA_TILT * roll  + (1.0f - ALPHA_TILT) * roll_f;
  pitch_f = ALPHA_TILT * pitch + (1.0f - ALPHA_TILT) * pitch_f;

  // ========== Raw gyro in deg/s ==========
  float gx_raw = rad2deg(gyro.gyro.x);
  float gy_raw = rad2deg(gyro.gyro.y);
  float gz_raw = rad2deg(gyro.gyro.z);

  // Remove current bias estimate
  float gx = gx_raw - bx;
  float gy = gy_raw - by;
  float gz = gz_raw - bz;

  // Pre-tilt deadzone
  gx = deadzone(gx, DEADZONE_DPS);
  gy = deadzone(gy, DEADZONE_DPS);
  gz = deadzone(gz, DEADZONE_DPS);

  // ========== Auto-bias learning when still ==========
  static uint32_t stillStart = 0;
  bool still = (fabsf(gx) < STILL_DPS && fabsf(gy) < STILL_DPS && fabsf(gz) < STILL_DPS);

  if (still) {
    if (stillStart == 0) stillStart = nowMs;
    if (nowMs - stillStart > STILL_MS) {
      bx = (1.0f - BIAS_BETA) * bx + BIAS_BETA * gx_raw;
      by = (1.0f - BIAS_BETA) * by + BIAS_BETA * gy_raw;
      bz = (1.0f - BIAS_BETA) * bz + BIAS_BETA * gz_raw;
    }
  } else {
    stillStart = 0;
  }

  // ========== Tilt-compensate gyro (rotate by roll/pitch) ==========
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

  // Post-compensation deadzone (key for “keeps moving upward”)
  if (fabsf(fgy) < POST_DZ_DPS_Y) fgy = 0.0f;
  if (fabsf(fgz) < POST_DZ_DPS_Z) fgz = 0.0f;

  // ========== Map to cursor (your requested axes) ==========
  // Z -> left/right, Y -> up/down
  float dx_f =  (fgz) * GAIN_Z;
  float dy_f = -(fgy) * GAIN_Y;

  // Anti-noise accumulator gate (prevents integrating tiny residuals)
  if (fabsf(dx_f) < 0.05f) dx_f = 0.0f;
  if (fabsf(dy_f) < 0.05f) dy_f = 0.0f;

  accDx += dx_f;
  accDy += dy_f;

  // ...but only notify at a fixed safe cadence
  if (nowMs - lastReportMs >= REPORT_MS) {
    lastReportMs = nowMs;

    int dx = (int)lroundf(accDx);
    int dy = (int)lroundf(accDy);

    // keep fractional remainder for smoothness
    accDx -= dx;
    accDy -= dy;

    // absolute gate: don’t send empty reports
    if (dx == 0 && dy == 0) {
      delay(LOOP_DELAY_MS);
      return;
    }

    dx = constrain(dx, -127, 127);
    dy = constrain(dy, -127, 127);

    // If the host disconnected mid-frame, skip sending to avoid notify errors.
    if (!bleMouse.isConnected()) {
      accDx = accDy = 0;
      return;
    }

    // If direction feels inverted, flip signs here (ONLY here).
    bleMouse.move(-dx, -dy);
  }

  delay(LOOP_DELAY_MS);
}
