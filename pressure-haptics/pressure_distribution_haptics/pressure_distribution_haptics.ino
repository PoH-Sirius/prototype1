#include <Arduino.h>
#include <M5Unified.h>
#include <ctype.h>
#include <math.h>
#include "driver/i2s_tdm.h"
#include "CheckedTactileSensor.h"

// ============================================================
// Pressure-distribution haptics for a 6 x 6 tactile array
//
// Changes from the original sketch:
// - Uses the complete pressure distribution instead of one maximum per quadrant.
// - Bilinearly distributes each taxel to the four actuators.
// - Uses low-pass pressure and a short-time pressure difference as the input.
// - Adds selectable frequency and delay presets inspired by Hirose & Inami,
//   "Haptic Representation Method for Material Properties utilizing
//   Pseudo-weight Shifting", CHI 2026.
// - Uses a zero-mean asymmetric waveform instead of a pure sine wave.
//
// Serial commands: c = direct contact, w = water, y = yogurt, r = rubber,
//                  d = toggle directional pseudo-shift,
//                  s = toggle NORMAL / LIGHT-GRIP sensitivity,
//                  i = cycle IMU axis mapping
// ============================================================

// ============================================================
// I2S / TDM pins
// ============================================================

#define I2S_BCLK_PIN 5
#define I2S_LRCK_PIN 4
#define I2S_DOUT_PIN 3

#define SAMPLE_RATE 44100
#define TDM_SLOTS 8

i2s_chan_handle_t tx_chan;

// ============================================================
// Tactile sensor
// ============================================================

#define DFROBOT_TACTILE_ARRAY_SIZE 36
#define ADDRESS 1

CheckedTactileSensor tactile(ADDRESS, &Serial1);

// ============================================================
// Installation-dependent mapping
// ============================================================

// Finger coordinate system after the sensor was moved to the index-finger
// side. Logical order:
// 0 = fingertip-left, 1 = fingertip-right,
// 2 = joint-left,     3 = joint-right.
// Physical sensor corners become:
// fingertip-left=sensor upper-right, fingertip-right=sensor lower-right,
// joint-left=sensor upper-left, joint-right=sensor lower-left.
constexpr bool SWAP_SENSOR_XY = true;
constexpr bool FLIP_SENSOR_X = false;
constexpr bool FLIP_SENSOR_Y = true;

// Actuator grid:
//   fingertip row (right-hand view): thumb SLOT2, index SLOT0
//   joint row:                       thumb SLOT3, index SLOT1
// Values are in the logical pressure order shown above.
constexpr uint8_t SLOT_FOR_CORNER[4] = {2, 0, 3, 1};

// Change a sign if an actuator is mounted in the opposite direction.
constexpr float ACTUATOR_POLARITY[4] = {1.0f, 1.0f, 1.0f, 1.0f};

// ============================================================
// Pressure processing
// ============================================================

constexpr uint32_t SENSOR_PERIOD_MS = 50;  // tactile sensor is configured to 20 Hz
constexpr uint32_t SENSOR_TIMEOUT_MS = 300;

constexpr float PRESSURE_LPF_ALPHA = 0.32f;

constexpr float COP_LPF_ALPHA = 0.42f;
// Fast-minus-slow filtering isolates small fingertip deformation from the
// participant's steady grip. At 20 Hz, the slow baseline adapts in ~2 s.
constexpr float TAXEL_FAST_ALPHA = 0.55f;
constexpr float TAXEL_BASELINE_ALPHA = 0.025f;
// A dynamic left/right or up/down pressure moment of 3.5% of total grip
// pressure reaches full directional input.
constexpr float DEFORMATION_RATIO_FULL = 0.035f;
// Change to -1.0f if the perceived pull is opposite to the CoP movement.
constexpr float DIRECTION_SIGN = 1.0f;

// 100 ms difference window at 20 Hz. This is the pressure equivalent of the
// paper's low-pass + finite-difference acceleration processing.
constexpr int DIFF_FRAMES = 2;
constexpr int HISTORY_FRAMES = 32;  // 1.6 s at 20 Hz

// Original code used a peak amplitude of 10000. Keep the same electrical limit.
constexpr float MAX_OUTPUT_AMPLITUDE = 10000.0f;
constexpr float MASTER_GAIN = 0.85f;
// Vibr-eau measured liquid/container impacts around 90.75 ms and used 80 ms
// vibrotactile pulses. Keep this transient separate from the continuous flow.
constexpr uint32_t FLUID_BURST_MS = 80;
constexpr uint32_t FLUID_BURST_COOLDOWN_MS = 75;
// Four actuators firing together feel much stronger than one actuator at the
// same numerical amplitude. Scale only the short liquid-impact burst while
// preserving the tilt/flow cue and the 80 ms research-informed envelope.
constexpr float WATER_BURST_GAIN = 0.52f;
constexpr float YOGURT_BURST_GAIN = 0.52f;
constexpr int FLUID_BURST_SAMPLES =
    (SAMPLE_RATE * FLUID_BURST_MS) / 1000;
constexpr int FLUID_BURST_ATTACK_SAMPLES =
    (SAMPLE_RATE * 6) / 1000;

float filteredPressure[4] = {0, 0, 0, 0};
float pressureHistory[4][HISTORY_FRAMES] = {};
float outputHistory[4][HISTORY_FRAMES] = {};
float targetAmplitude[4] = {0, 0, 0, 0};
float currentAmplitude[4] = {0, 0, 0, 0};

float phase[4] = {0, 0, 0, 0};
float copXHistory[HISTORY_FRAMES] = {};
float copYHistory[HISTORY_FRAMES] = {};
float sloshImpactXHistory[HISTORY_FRAMES] = {};
float sloshImpactYHistory[HISTORY_FRAMES] = {};
float taxelFast[6][6] = {};
float taxelBaseline[6][6] = {};
float filteredCopX = 0.5f;
float filteredCopY = 0.5f;
bool copInitialized = false;
bool deformationInitialized = false;
bool sloshInitialized = false;
bool directionalEnabled = true;
float liquidPositionX = 0.0f;
float liquidPositionY = 0.0f;
float liquidVelocityX = 0.0f;
float liquidVelocityY = 0.0f;
int8_t liquidWallX = 0;
int8_t liquidWallY = 0;
float fluidCarrierHz = 110.0f;
int fluidBurstAgeSamples = -1;
float fluidBurstStrength[4] = {0, 0, 0, 0};
float fluidBurstFrequencyHz = 160.0f;
float fluidBurstPhase = 0.0f;
float fluidResonatorY1 = 0.0f;
float fluidResonatorY2 = 0.0f;
uint32_t fluidNoiseState = 0x6D2B79F5u;
uint32_t lastFluidBurstMs = 0;
bool imuAvailable = false;
bool imuBaselineInitialized = false;
uint8_t imuMappingIndex = 3;  // XY with both signs reversed for this mounting.
float imuNeutralX = 0.0f;
float imuNeutralY = 0.0f;
float imuGravityX = 0.0f;
float imuGravityY = 0.0f;
float imuMotionBaseline[3] = {0.0f, 0.0f, 0.0f};
float imuTiltX = 0.0f;
float imuTiltY = 0.0f;
float imuShakeX = 0.0f;
float imuShakeY = 0.0f;
float previousPressureDirectionX = 0.0f;
float previousPressureDirectionY = 0.0f;
bool pressureMotionInitialized = false;
int sensitivityIndex = 1;  // Start in LIGHT-GRIP mode.
int historyHead = 0;
uint32_t lastSensorReadMs = 0;
uint32_t lastGoodSensorMs = 0;
uint32_t lastDebugMs = 0;

struct MaterialPreset {
  const char *name;
  float frequencyHz;
  uint16_t delayMs;
  float steadyMix;
  float changeMix;
  float harmonicMix;
  float directionScale;
  float amplitudeFollow;
};

struct SensitivityPreset {
  const char *name;
  uint16_t sensorThreshold;
  float pressureOn;
  float pressureFull;
  float pressureChangeFull;
  float copMinTotalPressure;
  float copMovementFull;
  float directionalGain;
  float minimumDirectionalGate;
};

// NORMAL is the previous stable setting. LIGHT-GRIP is intended for a
// participant holding the device gently while moving it. The electrical
// maximum remains unchanged in both modes.
const SensitivityPreset SENSITIVITY[] = {
    {"NORMAL", 20, 45.0f, 2300.0f, 750.0f, 800.0f, 0.18f, 0.58f, 0.0f},
    {"LIGHT-GRIP", 8, 12.0f, 1350.0f, 320.0f, 220.0f, 0.08f, 0.72f, 0.25f},
};

// Water / yogurt / rubber use the frequency and delay examples shown in the
// paper's Figure 1. Mix values are safe starting points for this pressure input
// and should be tuned experimentally with the actual enclosure and actuators.
const MaterialPreset MATERIALS[] = {
    // name, Hz, delay, steady, change, asymmetry, direction, response speed
    {"CONTACT", 120.0f, 0, 0.50f, 0.50f, 0.18f, 0.70f, 0.0020f},
    {"WATER", 160.0f, 0, 0.00f, 0.00f, 0.22f, 1.05f, 0.0048f},
    {"YOGURT", 80.0f, 100, 0.00f, 0.00f, 0.30f, 0.90f, 0.0012f},
    {"RUBBER", 40.0f, 200, 0.58f, 0.42f, 0.34f, 0.55f, 0.0008f},
};

constexpr int MATERIAL_COUNT = sizeof(MATERIALS) / sizeof(MATERIALS[0]);
constexpr int WATER_MATERIAL_INDEX = 1;
constexpr int YOGURT_MATERIAL_INDEX = 2;
// Start in WATER so reopening the USB serial port cannot fall back to the
// continuous CONTACT buzz if the board is reset by the host application.
int materialIndex = WATER_MATERIAL_INDEX;

// ============================================================
// Helpers
// ============================================================

static float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static float clampSigned(float x) {
  if (x < -1.0f) return -1.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static float normalizedWithDeadband(float x, float deadband, float fullScale) {
  const float magnitude = fabsf(x);
  if (magnitude <= deadband) return 0.0f;
  const float normalized =
      clamp01((magnitude - deadband) / (fullScale - deadband));
  return x < 0.0f ? -normalized : normalized;
}

static void triggerFluidBurst(float strength0, float strength1,
                              float strength2, float strength3,
                              float frequencyHz, uint32_t now) {
  fluidBurstStrength[0] = clamp01(strength0);
  fluidBurstStrength[1] = clamp01(strength1);
  fluidBurstStrength[2] = clamp01(strength2);
  fluidBurstStrength[3] = clamp01(strength3);
  fluidBurstFrequencyHz = frequencyHz;
  fluidBurstAgeSamples = 0;
  fluidBurstPhase = 0.0f;
  fluidResonatorY1 = 0.0f;
  fluidResonatorY2 = 0.0f;
  lastFluidBurstMs = now;
}

static void mapImuAxes(float ax, float ay, float az,
                       float &mappedX, float &mappedY) {
  const float raw[3] = {ax, ay, az};
  // Ordered pairs: XY, YX, XZ, ZX, YZ, ZY. Four sign combinations per pair.
  constexpr uint8_t PAIRS[6][2] = {
      {0, 1}, {1, 0}, {0, 2}, {2, 0}, {1, 2}, {2, 1}};
  const uint8_t pairIndex = (imuMappingIndex / 4) % 6;
  const uint8_t signIndex = imuMappingIndex % 4;
  mappedX = raw[PAIRS[pairIndex][0]];
  mappedY = raw[PAIRS[pairIndex][1]];
  if (signIndex & 1) mappedX = -mappedX;
  if (signIndex & 2) mappedY = -mappedY;
}

static uint8_t unusedImuAxis() {
  // Missing raw axis for XY, YX, XZ, ZX, YZ, ZY respectively.
  constexpr uint8_t UNUSED_AXIS[6] = {2, 2, 1, 1, 0, 0};
  return UNUSED_AXIS[(imuMappingIndex / 4) % 6];
}

static void printImuMapping() {
  constexpr const char *PAIR_NAMES[6] = {"XY", "YX", "XZ", "ZX", "YZ", "ZY"};
  const uint8_t pairIndex = (imuMappingIndex / 4) % 6;
  const uint8_t signIndex = imuMappingIndex % 4;
  Serial.printf("IMU mapping=%s Xsign=%c Ysign=%c (%u/24)\n",
                PAIR_NAMES[pairIndex],
                (signIndex & 1) ? '-' : '+',
                (signIndex & 2) ? '-' : '+',
                imuMappingIndex + 1);
}

static int wrappedIndex(int index) {
  while (index < 0) index += HISTORY_FRAMES;
  while (index >= HISTORY_FRAMES) index -= HISTORY_FRAMES;
  return index;
}

static int delayFramesFor(const MaterialPreset &material) {
  const int frames = (material.delayMs + SENSOR_PERIOD_MS / 2) / SENSOR_PERIOD_MS;
  return constrain(frames, 0, HISTORY_FRAMES - 1);
}

// Two-harmonic approximation of an asymmetric waveform. It has zero DC but
// asymmetric peaks. The exact perceived force depends strongly on the
// actuator and enclosure.
static float asymmetricWave(float p, float harmonicMix) {
  const float a = 1.0f - harmonicMix;
  const float wave = a * sinf(p) + harmonicMix * sinf(2.0f * p - 0.5f * PI);
  return constrain(wave, -1.0f, 1.0f);
}

static void printMaterial() {
  const MaterialPreset &m = MATERIALS[materialIndex];
  Serial.printf("Material=%s  frequency=%.1fHz  delay=%ums  direction=%.2f\n",
                m.name, m.frequencyHz, m.delayMs, m.directionScale);
}

static void printDirectionalState() {
  Serial.printf("Directional pseudo-shift=%s\n",
                directionalEnabled ? "ON" : "OFF");
}

static void printSensitivity() {
  const SensitivityPreset &s = SENSITIVITY[sensitivityIndex];
  Serial.printf("Sensitivity=%s  sensor-threshold=%u\n",
                s.name, s.sensorThreshold);
}

static void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char command = (char)tolower((unsigned char)Serial.read());
    if (command == 'd') {
      directionalEnabled = !directionalEnabled;
      printDirectionalState();
      continue;
    }
    if (command == 's') {
      sensitivityIndex = 1 - sensitivityIndex;
      tactile.setThld(SENSITIVITY[sensitivityIndex].sensorThreshold);
      copInitialized = false;
      deformationInitialized = false;
      imuBaselineInitialized = false;
      pressureMotionInitialized = false;
      printSensitivity();
      continue;
    }
    if (command == 'i') {
      if (!imuAvailable) {
        Serial.println("IMU is not available; pressure-only fallback remains active");
        continue;
      }
      imuMappingIndex = (imuMappingIndex + 1) % 24;
      imuBaselineInitialized = false;
      pressureMotionInitialized = false;
      sloshInitialized = false;
      printImuMapping();
      continue;
    }
    int nextMaterial = materialIndex;
    if (command == 'c') nextMaterial = 0;
    else if (command == 'w') nextMaterial = 1;
    else if (command == 'y') nextMaterial = 2;
    else if (command == 'r') nextMaterial = 3;
    else continue;

    // Do not let samples generated by the previous preset leak through the
    // delay line of the newly selected material.
    materialIndex = nextMaterial;
    sloshInitialized = false;
    imuBaselineInitialized = false;
    pressureMotionInitialized = false;
    imuShakeX = 0.0f;
    imuShakeY = 0.0f;
    liquidPositionX = 0.0f;
    liquidPositionY = 0.0f;
    liquidVelocityX = 0.0f;
    liquidVelocityY = 0.0f;
    liquidWallX = 0;
    liquidWallY = 0;
    fluidCarrierHz = 110.0f;
    fluidBurstAgeSamples = -1;
    for (int corner = 0; corner < 4; ++corner) {
      fluidBurstStrength[corner] = 0.0f;
    }
    for (int corner = 0; corner < 4; ++corner) {
      for (int frame = 0; frame < HISTORY_FRAMES; ++frame) {
        outputHistory[corner][frame] = 0.0f;
      }
      targetAmplitude[corner] = 0.0f;
    }
    for (int frame = 0; frame < HISTORY_FRAMES; ++frame) {
      sloshImpactXHistory[frame] = 0.0f;
      sloshImpactYHistory[frame] = 0.0f;
    }
    printMaterial();
  }
}

// ============================================================
// I2S / TDM initialization (kept compatible with the original sketch)
// ============================================================

void setupTDM() {
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));

  i2s_tdm_config_t tdm_cfg = {
      .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
      .slot_cfg = I2S_TDM_PCM_SHORT_SLOT_DEFAULT_CONFIG(
          I2S_DATA_BIT_WIDTH_16BIT,
          I2S_SLOT_MODE_STEREO,
          (i2s_tdm_slot_mask_t)(
              I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3 |
              I2S_TDM_SLOT4 | I2S_TDM_SLOT5 | I2S_TDM_SLOT6 | I2S_TDM_SLOT7)),
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED,
          .bclk = (gpio_num_t)I2S_BCLK_PIN,
          .ws = (gpio_num_t)I2S_LRCK_PIN,
          .dout = (gpio_num_t)I2S_DOUT_PIN,
          .din = I2S_GPIO_UNUSED,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false,
          },
      },
  };

  tdm_cfg.slot_cfg.total_slot = 8;
  tdm_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;

  ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(tx_chan, &tdm_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));

  Serial.println("TDM Ready: 44.1 kHz / 16 bit / 8 slots");
}

// ============================================================
// Pressure distribution -> four actuator targets
// ============================================================

static void updatePressureTargets() {
  const uint32_t now = millis();
  if (now - lastSensorReadMs < SENSOR_PERIOD_MS) return;
  lastSensorReadMs = now;

  const sAdcDatas_t adcDatas = tactile.getDatasChecked();
  if (adcDatas.result != 0) {
    Serial.printf("Sensor ADC read failed: 0x%02X\n", adcDatas.result);
    if (now - lastGoodSensorMs > SENSOR_TIMEOUT_MS) {
      for (int corner = 0; corner < 4; ++corner) targetAmplitude[corner] = 0;
    }
    return;
  }
  lastGoodSensorMs = now;

  float weightedPressure[4] = {0, 0, 0, 0};
  float weightTotal[4] = {0, 0, 0, 0};
  float quadrantPeak[4] = {0, 0, 0, 0};
  float totalPressure = 0.0f;
  float copNumeratorX = 0.0f;
  float copNumeratorY = 0.0f;
  float deformationMomentX = 0.0f;
  float deformationMomentY = 0.0f;

  const int sizeX = tactileSensorArrayX;
  const int sizeY = tactileSensorArrayY;

  for (int rawY = 0; rawY < sizeY; ++rawY) {
    for (int rawX = 0; rawX < sizeX; ++rawX) {
      int sx = rawX;
      int sy = rawY;
      if (SWAP_SENSOR_XY) {
        const int temp = sx;
        sx = sy;
        sy = temp;
      }

      float x = (float)sx / (float)((SWAP_SENSOR_XY ? sizeY : sizeX) - 1);
      float y = (float)sy / (float)((SWAP_SENSOR_XY ? sizeX : sizeY) - 1);
      if (FLIP_SENSOR_X) x = 1.0f - x;
      if (FLIP_SENSOR_Y) y = 1.0f - y;

      float p = (float)adcDatas.adcval[rawY][rawX];
      if (p < 0) p = 0;
      if (p > 4095) p = 4095;

      totalPressure += p;
      copNumeratorX += p * x;
      copNumeratorY += p * y;

      if (!deformationInitialized) {
        taxelFast[rawY][rawX] = p;
        taxelBaseline[rawY][rawX] = p;
      } else {
        taxelFast[rawY][rawX] +=
            TAXEL_FAST_ALPHA * (p - taxelFast[rawY][rawX]);
        taxelBaseline[rawY][rawX] +=
            TAXEL_BASELINE_ALPHA * (p - taxelBaseline[rawY][rawX]);
        const float dynamicPressure =
            taxelFast[rawY][rawX] - taxelBaseline[rawY][rawX];
        deformationMomentX += dynamicPressure * (2.0f * x - 1.0f);
        deformationMomentY += dynamicPressure * (2.0f * y - 1.0f);
      }

      // Bilinear corner weights. Their sum is exactly one for every taxel.
      const float w[4] = {
          (1.0f - x) * (1.0f - y),
          x * (1.0f - y),
          (1.0f - x) * y,
          x * y,
      };

      for (int corner = 0; corner < 4; ++corner) {
        weightedPressure[corner] += p * w[corner];
        weightTotal[corner] += w[corner];
      }

      const int quadrant = (y >= 0.5f ? 2 : 0) + (x >= 0.5f ? 1 : 0);
      if (p > quadrantPeak[quadrant]) quadrantPeak[quadrant] = p;
    }
  }

  const MaterialPreset &material = MATERIALS[materialIndex];
  const SensitivityPreset &sensitivity = SENSITIVITY[sensitivityIndex];
  const int differenceIndex = wrappedIndex(historyHead - DIFF_FRAMES);
  const int outputDelayFrames = delayFramesFor(material);
  const int delayedOutputIndex = wrappedIndex(historyHead - outputDelayFrames);

  // Calibrate the current lightly held pose as neutral, then use gravity
  // projection as a reliable tilt signal. Pressure remains the contact gate
  // and contributes the fingertip-deformation component.
  imuTiltX = 0.0f;
  imuTiltY = 0.0f;
  imuShakeX = 0.0f;
  imuShakeY = 0.0f;
  if (imuAvailable) {
    M5.Imu.update();
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    if (M5.Imu.getAccel(&ax, &ay, &az)) {
      float mappedX = 0.0f;
      float mappedY = 0.0f;
      mapImuAxes(ax, ay, az, mappedX, mappedY);
      if (totalPressure >= sensitivity.copMinTotalPressure) {
        if (!imuBaselineInitialized) {
          imuNeutralX = mappedX;
          imuNeutralY = mappedY;
          imuGravityX = mappedX;
          imuGravityY = mappedY;
          imuMotionBaseline[0] = ax;
          imuMotionBaseline[1] = ay;
          imuMotionBaseline[2] = az;
          imuBaselineInitialized = true;
        } else {
          // Tilt and shake use separate time scales. Gravity follows a slow
          // tilt smoothly, while the motion baseline moves much more slowly
          // so repeated shaking keeps producing acceleration pulses.
          constexpr float IMU_GRAVITY_ALPHA = 0.10f;
          constexpr float IMU_MOTION_BASELINE_ALPHA = 0.04f;
          constexpr float IMU_TILT_FULL_G = 0.12f;
          imuGravityX += IMU_GRAVITY_ALPHA * (mappedX - imuGravityX);
          imuGravityY += IMU_GRAVITY_ALPHA * (mappedY - imuGravityY);
          imuTiltX = clampSigned(
              (imuGravityX - imuNeutralX) / IMU_TILT_FULL_G);
          imuTiltY = clampSigned(
              (imuGravityY - imuNeutralY) / IMU_TILT_FULL_G);
          const float rawMotion[3] = {
              ax - imuMotionBaseline[0],
              ay - imuMotionBaseline[1],
              az - imuMotionBaseline[2],
          };
          for (int axis = 0; axis < 3; ++axis) {
            imuMotionBaseline[axis] +=
                IMU_MOTION_BASELINE_ALPHA * rawMotion[axis];
          }

          float mappedMotionX = 0.0f;
          float mappedMotionY = 0.0f;
          mapImuAxes(rawMotion[0], rawMotion[1], rawMotion[2],
                     mappedMotionX, mappedMotionY);
          const float mappedShakeX = normalizedWithDeadband(
              mappedMotionX, 0.015f, 0.25f);
          const float mappedShakeY = normalizedWithDeadband(
              mappedMotionY, 0.015f, 0.25f);

          // Motion along the third, unmapped axis has no natural X/Y wall.
          // Convert it into an opposing diagonal pulse so every shake
          // direction can still produce a spatial splash instead of silence.
          const float depthShake = normalizedWithDeadband(
              rawMotion[unusedImuAxis()], 0.015f, 0.25f);
          imuShakeX = clampSigned(mappedShakeX + 0.55f * depthShake);
          imuShakeY = clampSigned(mappedShakeY - 0.55f * depthShake);
        }
      } else {
        imuBaselineInitialized = false;
      }
    }
  }

  // Estimate how the center of pressure moved over the same 100 ms window as
  // the local pressure difference. On first contact, initialize the complete
  // history to avoid creating a false direction pulse.
  float copMovementX = 0.0f;
  float copMovementY = 0.0f;
  float deformationX = 0.0f;
  float deformationY = 0.0f;
  if (totalPressure >= sensitivity.copMinTotalPressure) {
    const float rawCopX = copNumeratorX / totalPressure;
    const float rawCopY = copNumeratorY / totalPressure;

    if (!copInitialized) {
      filteredCopX = rawCopX;
      filteredCopY = rawCopY;
      for (int frame = 0; frame < HISTORY_FRAMES; ++frame) {
        copXHistory[frame] = filteredCopX;
        copYHistory[frame] = filteredCopY;
      }
      copInitialized = true;
    } else {
      filteredCopX += COP_LPF_ALPHA * (rawCopX - filteredCopX);
      filteredCopY += COP_LPF_ALPHA * (rawCopY - filteredCopY);
      copMovementX = clampSigned(
          (filteredCopX - copXHistory[differenceIndex]) /
          sensitivity.copMovementFull);
      copMovementY = clampSigned(
          (filteredCopY - copYHistory[differenceIndex]) /
          sensitivity.copMovementFull);
    }

    copXHistory[historyHead] = filteredCopX;
    copYHistory[historyHead] = filteredCopY;

    if (deformationInitialized) {
      const float normalization =
          totalPressure * DEFORMATION_RATIO_FULL;
      deformationX = clampSigned(deformationMomentX / normalization);
      deformationY = clampSigned(deformationMomentY / normalization);
    }
    deformationInitialized = true;
  } else {
    copInitialized = false;
    deformationInitialized = false;
    pressureMotionInitialized = false;
    filteredCopX = 0.5f;
    filteredCopY = 0.5f;
  }

  float baseIntensity[4] = {0, 0, 0, 0};
  float strongestSteady = 0.0f;

  for (int corner = 0; corner < 4; ++corner) {
    const float distributedMean =
        weightTotal[corner] > 0.0f ? weightedPressure[corner] / weightTotal[corner] : 0.0f;

    // Mean pressure gives smooth movement. A smaller peak component preserves
    // crisp response to a fingertip-sized contact.
    const float localPressure = 0.62f * distributedMean + 0.38f * quadrantPeak[corner];

    filteredPressure[corner] +=
        PRESSURE_LPF_ALPHA * (localPressure - filteredPressure[corner]);

    pressureHistory[corner][historyHead] = filteredPressure[corner];
    const float oldPressure = pressureHistory[corner][differenceIndex];
    const float signedChange = filteredPressure[corner] - oldPressure;

    float steady =
        (filteredPressure[corner] - sensitivity.pressureOn) /
        (sensitivity.pressureFull - sensitivity.pressureOn);
    steady = powf(clamp01(steady), 0.68f);

    // A moderate pressure change reaches full transient output. Keep its sign
    // so pressing and releasing reverse the asymmetric waveform. Releases are
    // intentionally weaker than presses.
    float change = clampSigned(
        signedChange / sensitivity.pressureChangeFull);
    if (change < 0.0f) change *= 0.38f;

    baseIntensity[corner] = clampSigned(
        material.steadyMix * steady + material.changeMix * change);

    if (steady > strongestSteady) strongestSteady = steady;
  }

  // Finger-space direction vectors: fingertip-left(-x,-y),
  // fingertip-right(+x,-y), joint-left(-x,+y), joint-right(+x,+y).
  // Opposing sides receive opposite waveform polarity.
  constexpr float CORNER_X[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
  constexpr float CORNER_Y[4] = {-1.0f, -1.0f, 1.0f, 1.0f};
  constexpr float INV_SQRT_2 = 0.70710678f;
  // CoP motion gives the gross movement; fast-minus-slow taxel deformation
  // emphasizes the tiny redistribution caused by the fingertip pressing a
  // side wall. The latter is deliberately weighted more heavily.
  const float directionInputX = clampSigned(
      0.55f * copMovementX + 0.95f * deformationX);
  const float directionInputY = clampSigned(
      0.55f * copMovementY + 0.95f * deformationY);

  // A quick change in pressure direction is another shake cue. This keeps the
  // jaba-jaba response available even if the IMU is unavailable.
  float pressureShakeX = 0.0f;
  float pressureShakeY = 0.0f;
  if (copInitialized) {
    if (!pressureMotionInitialized) {
      previousPressureDirectionX = directionInputX;
      previousPressureDirectionY = directionInputY;
      pressureMotionInitialized = true;
    } else {
      pressureShakeX = normalizedWithDeadband(
          directionInputX - previousPressureDirectionX, 0.06f, 0.35f);
      pressureShakeY = normalizedWithDeadband(
          directionInputY - previousPressureDirectionY, 0.06f, 0.35f);
      previousPressureDirectionX = directionInputX;
      previousPressureDirectionY = directionInputY;
    }
  }
  const float fluidDirectionInputX =
      (imuAvailable && imuBaselineInitialized)
          ? clampSigned(0.72f * imuTiltX + 0.70f * directionInputX)
          : directionInputX;
  const float fluidDirectionInputY =
      (imuAvailable && imuBaselineInitialized)
          ? clampSigned(0.72f * imuTiltY + 0.70f * directionInputY)
          : directionInputY;

  // Water and yogurt are pressure-driven virtual masses. Pressure deformation
  // supplies a tilt proxy, while each material has its own inertia, damping,
  // wall response, flow strength, and carrier frequency.
  float driveInputX = directionInputX;
  float driveInputY = directionInputY;
  float sloshX = 0.0f;
  float sloshY = 0.0f;
  float fluidFlowX = 0.0f;
  float fluidFlowY = 0.0f;
  float splashX = 0.0f;
  float splashY = 0.0f;
  const bool waterMode = materialIndex == WATER_MATERIAL_INDEX;
  const bool yogurtMode = materialIndex == YOGURT_MATERIAL_INDEX;
  const bool fluidMode = waterMode || yogurtMode;
  if (fluidMode) {
    const float liquidDrive = yogurtMode ? 1.80f : 2.10f;
    const float liquidSpring = yogurtMode ? 8.0f : 24.0f;
    const float liquidDamping = yogurtMode ? 10.0f : 7.0f;
    const float wallPosition = yogurtMode ? 0.45f : 0.58f;
    const float wallRelease = yogurtMode ? 0.18f : 0.22f;
    const float impactBase = yogurtMode ? 0.18f : 0.30f;
    const float impactVelocityGain = yogurtMode ? 0.75f : 0.85f;
    const int reboundFrames = yogurtMode ? 5 : 3;
    const int rippleFrames = yogurtMode ? 10 : 6;
    const float reboundGain = yogurtMode ? 0.30f : 0.55f;
    const float rippleGain = yogurtMode ? 0.12f : 0.24f;
    const float flowGain = yogurtMode ? 0.38f : 0.28f;
    const float flowVelocityFull = yogurtMode ? 1.20f : 2.50f;

    float impactX = 0.0f;
    float impactY = 0.0f;
    if (!sloshInitialized) {
      liquidPositionX = 0.0f;
      liquidPositionY = 0.0f;
      liquidVelocityX = 0.0f;
      liquidVelocityY = 0.0f;
      liquidWallX = 0;
      liquidWallY = 0;
      sloshInitialized = true;
    } else {
      constexpr float DT = SENSOR_PERIOD_MS * 0.001f;
      const float targetX = clampSigned(
          liquidDrive * fluidDirectionInputX);
      const float targetY = clampSigned(
          liquidDrive * fluidDirectionInputY);
      const float accelerationX =
          liquidSpring * (targetX - liquidPositionX) -
          liquidDamping * liquidVelocityX;
      const float accelerationY =
          liquidSpring * (targetY - liquidPositionY) -
          liquidDamping * liquidVelocityY;

      liquidVelocityX += accelerationX * DT;
      liquidVelocityY += accelerationY * DT;
      liquidPositionX = constrain(
          liquidPositionX + liquidVelocityX * DT, -1.0f, 1.0f);
      liquidPositionY = constrain(
          liquidPositionY + liquidVelocityY * DT, -1.0f, 1.0f);

      if (fabsf(liquidPositionX) < wallRelease) liquidWallX = 0;
      if (fabsf(liquidPositionY) < wallRelease) liquidWallY = 0;

      if (liquidPositionX >= wallPosition && liquidWallX != 1) {
        impactX = clamp01(
            impactBase + fabsf(liquidVelocityX) * impactVelocityGain);
        liquidWallX = 1;
      } else if (liquidPositionX <= -wallPosition && liquidWallX != -1) {
        impactX = -clamp01(
            impactBase + fabsf(liquidVelocityX) * impactVelocityGain);
        liquidWallX = -1;
      }

      if (liquidPositionY >= wallPosition && liquidWallY != 1) {
        impactY = clamp01(
            impactBase + fabsf(liquidVelocityY) * impactVelocityGain);
        liquidWallY = 1;
      } else if (liquidPositionY <= -wallPosition && liquidWallY != -1) {
        impactY = -clamp01(
            impactBase + fabsf(liquidVelocityY) * impactVelocityGain);
        liquidWallY = -1;
      }
    }

    sloshImpactXHistory[historyHead] = impactX;
    sloshImpactYHistory[historyHead] = impactY;
    const int reboundIndex = wrappedIndex(historyHead - reboundFrames);
    const int rippleIndex = wrappedIndex(historyHead - rippleFrames);
    sloshX = clampSigned(
        impactX - reboundGain * sloshImpactXHistory[reboundIndex] +
        rippleGain * sloshImpactXHistory[rippleIndex]);
    sloshY = clampSigned(
        impactY - reboundGain * sloshImpactYHistory[reboundIndex] +
        rippleGain * sloshImpactYHistory[rippleIndex]);

    // A small velocity-proportional cue represents liquid moving inside the
    // container before it reaches a wall. It disappears after the virtual
    // mass settles, unlike the old continuous pressure buzz.
    fluidFlowX = flowGain * normalizedWithDeadband(
        liquidVelocityX, 0.06f, flowVelocityFull);
    fluidFlowY = flowGain * normalizedWithDeadband(
        liquidVelocityY, 0.06f, flowVelocityFull);

    // Fast alternating acceleration makes the contents feel as if they break
    // into several impacts while the enclosure is being shaken. Yogurt keeps
    // this component softer because its apparent viscosity is higher.
    const float shakeInputX =
        (imuAvailable && imuBaselineInitialized)
            ? clampSigned(0.85f * imuShakeX + 0.35f * pressureShakeX)
            : pressureShakeX;
    const float shakeInputY =
        (imuAvailable && imuBaselineInitialized)
            ? clampSigned(0.85f * imuShakeY + 0.35f * pressureShakeY)
            : pressureShakeY;
    const float splashGain = yogurtMode ? 0.22f : 0.48f;
    splashX = splashGain * shakeInputX;
    splashY = splashGain * shakeInputY;

    driveInputX = clampSigned(sloshX + fluidFlowX + splashX);
    driveInputY = clampSigned(sloshY + fluidFlowY + splashY);

    // Flow uses a softer carrier; wall impacts become sharper and higher.
    const float collisionMagnitude = fmaxf(fabsf(sloshX), fabsf(sloshY));
    const float splashMagnitude = fmaxf(fabsf(splashX), fabsf(splashY));
    fluidCarrierHz = yogurtMode
        ? 65.0f + 20.0f * collisionMagnitude + 15.0f * splashMagnitude
        : 110.0f + 55.0f * collisionMagnitude + 40.0f * splashMagnitude;

    // Research-informed transient rendering:
    // - Vibr-eau found that shaking excites both sides with similar amplitude,
    //   so a fast shake triggers all four actuators together.
    // - For a slower wall collision, retain spatial asymmetry and emphasize
    //   the side reached by the virtual liquid.
    // - Cirio et al. relate impact intensity to velocity cubed; use a cubic
    //   curve above threshold while retaining a clear minimum tactile pulse.
    if (copInitialized && now - lastFluidBurstMs >= FLUID_BURST_COOLDOWN_MS) {
      const float shakeMagnitude =
          fmaxf(fabsf(shakeInputX), fabsf(shakeInputY));
      if (shakeMagnitude >= 0.10f) {
        const float normalizedShake =
            normalizedWithDeadband(shakeMagnitude, 0.10f, 0.80f);
        const float burstStrength = yogurtMode
            ? 0.30f + 0.40f * powf(normalizedShake, 3.0f)
            : 0.48f + 0.52f * powf(normalizedShake, 3.0f);
        triggerFluidBurst(burstStrength, burstStrength,
                          burstStrength, burstStrength,
                          yogurtMode ? 85.0f : 160.0f, now);
      } else if (collisionMagnitude >= 0.08f) {
        const float collisionStrength = yogurtMode
            ? 0.24f + 0.40f * powf(collisionMagnitude, 3.0f)
            : 0.38f + 0.58f * powf(collisionMagnitude, 3.0f);
        float cornerStrength[4] = {};
        for (int corner = 0; corner < 4; ++corner) {
          const float projection = clampSigned(INV_SQRT_2 *
              (CORNER_X[corner] * sloshX +
               CORNER_Y[corner] * sloshY));
          const float spatialWeight =
              0.25f + 0.75f * clamp01(0.5f + 0.5f * projection);
          cornerStrength[corner] = collisionStrength * spatialWeight;
        }
        triggerFluidBurst(cornerStrength[0], cornerStrength[1],
                          cornerStrength[2], cornerStrength[3],
                          yogurtMode ? 80.0f : 150.0f, now);
      }
    }
  } else {
    sloshInitialized = false;
    fluidBurstAgeSamples = -1;
    sloshImpactXHistory[historyHead] = 0.0f;
    sloshImpactYHistory[historyHead] = 0.0f;
  }

  const float contactGate = copInitialized
      ? fmaxf(sensitivity.minimumDirectionalGate,
              sqrtf(clamp01(strongestSteady)))
      : 0.0f;

  for (int corner = 0; corner < 4; ++corner) {
    float directional = 0.0f;
    if (directionalEnabled && copInitialized) {
      directional = INV_SQRT_2 *
          (CORNER_X[corner] * driveInputX +
           CORNER_Y[corner] * driveInputY);
      directional = DIRECTION_SIGN * clampSigned(directional) *
          contactGate * sensitivity.directionalGain *
          material.directionScale;
    }

    const float intensity =
        clampSigned(baseIntensity[corner] + directional);

    const float rawTarget =
        MAX_OUTPUT_AMPLITUDE * MASTER_GAIN * intensity;

    outputHistory[corner][historyHead] = rawTarget;
    targetAmplitude[corner] = outputHistory[corner][delayedOutputIndex];
  }

  historyHead = wrappedIndex(historyHead + 1);

  const bool fluidEvent =
      fabsf(sloshX) >= 0.05f || fabsf(sloshY) >= 0.05f ||
      fabsf(splashX) >= 0.05f || fabsf(splashY) >= 0.05f;
  if (now - lastDebugMs >= 200 || fluidEvent) {
    lastDebugMs = now;
    Serial.printf("%s T=%.0f P=[%.0f %.0f %.0f %.0f] C=[%.2f %.2f] d=[%+.2f %+.2f] e=[%+.2f %+.2f] g=[%+.2f %+.2f] k=[%+.2f %+.2f] u=[%+.2f %+.2f] f=[%+.2f %+.2f] j=[%+.2f %+.2f] q=[%+.2f %+.2f] b=[%.2f %.2f %.2f %.2f] A=[%.0f %.0f %.0f %.0f]\n",
                  material.name,
                  totalPressure,
                  filteredPressure[0], filteredPressure[1],
                  filteredPressure[2], filteredPressure[3],
                  filteredCopX, filteredCopY,
                  copMovementX, copMovementY,
                  deformationX, deformationY,
                  imuTiltX, imuTiltY,
                  imuShakeX, imuShakeY,
                  fluidDirectionInputX, fluidDirectionInputY,
                  fluidFlowX, fluidFlowY,
                  splashX, splashY,
                  sloshX, sloshY,
                  fluidBurstStrength[0], fluidBurstStrength[1],
                  fluidBurstStrength[2], fluidBurstStrength[3],
                  targetAmplitude[0], targetAmplitude[1],
                  targetAmplitude[2], targetAmplitude[3]);
  }

}

// ============================================================
// Setup
// ============================================================

void setup() {
  auto cfg = M5.config();
  cfg.internal_spk = false;
  cfg.internal_mic = false;
  cfg.internal_imu = true;
  M5.begin(cfg);
  // The tactile array is powered from the M5 external 5 V rail. Keep it on
  // after an ESP32 restart; otherwise every Modbus read times out with 0x09.
  M5.Power.setExtOutput(true);

  Serial.begin(115200);
  delay(500);
  imuAvailable = M5.Imu.isEnabled();

  Serial1.begin(115200, SERIAL_8N1, 1, 2);
  delay(300);

  uint16_t model = 0xffff;
  while (true) {
    // A USB restart can leave partial bytes in the sensor UART. Clear them
    // before each probe and recreate the UART after a failed response.
    while (Serial1.available() > 0) Serial1.read();
    const uint8_t status = tactile.beginChecked(model);
    if (status == 0 && model == 1) break;
    Serial.printf("Sensor init failed: status=0x%02X model=0x%04X; requires 6x6\n",
                  status, model);
    Serial1.end();
    delay(100);
    Serial1.begin(115200, SERIAL_8N1, 1, 2);
    delay(900);
  }

  tactile.setThld(SENSITIVITY[sensitivityIndex].sensorThreshold);
  tactile.setSampleRate(eSampleRate20Hz);

  setupTDM();
  lastGoodSensorMs = millis();

  Serial.println("Pressure-distribution haptics ready");
  Serial.println("Commands: c=contact, w=water, y=yogurt, r=rubber, d=direction on/off, s=sensitivity, i=IMU axes");
  Serial.printf("IMU=%s\n", imuAvailable ? "ON" : "NOT AVAILABLE (pressure fallback)");
  if (imuAvailable) printImuMapping();
  printMaterial();
  printDirectionalState();
  printSensitivity();
}

// ============================================================
// Main loop / audio generation
// ============================================================

void loop() {
  M5.update();
  handleSerialCommands();
  updatePressureTargets();

  constexpr int FRAMES = 512;
  static int16_t samples[FRAMES * TDM_SLOTS];
  constexpr float TWO_PI_F = 2.0f * PI;

  const MaterialPreset &material = MATERIALS[materialIndex];
  const float playbackFrequencyHz =
      (materialIndex == WATER_MATERIAL_INDEX ||
       materialIndex == YOGURT_MATERIAL_INDEX)
          ? fluidCarrierHz
          : material.frequencyHz;

  for (int i = 0; i < FRAMES; ++i) {
    int16_t wave[4] = {0, 0, 0, 0};

    float fluidBurstSample = 0.0f;
    if (fluidBurstAgeSamples >= 0) {
      if (fluidBurstAgeSamples >= FLUID_BURST_SAMPLES) {
        fluidBurstAgeSamples = -1;
        for (int corner = 0; corner < 4; ++corner) {
          fluidBurstStrength[corner] = 0.0f;
        }
      } else {
        float envelope = 0.0f;
        if (fluidBurstAgeSamples < FLUID_BURST_ATTACK_SAMPLES) {
          envelope = (float)fluidBurstAgeSamples /
              (float)FLUID_BURST_ATTACK_SAMPLES;
        } else {
          const float releaseProgress =
              (float)(fluidBurstAgeSamples - FLUID_BURST_ATTACK_SAMPLES) /
              (float)(FLUID_BURST_SAMPLES - FLUID_BURST_ATTACK_SAMPLES);
          envelope = expf(-5.0f * releaseProgress);
        }

        // Cirio et al. synthesize fluid impacts as short noise bursts through
        // a resonator. Mix that resonant noise with a damped cavity tone so
        // small actuators retain a clear fundamental instead of sounding like
        // a continuous motor buzz.
        fluidNoiseState ^= fluidNoiseState << 13;
        fluidNoiseState ^= fluidNoiseState >> 17;
        fluidNoiseState ^= fluidNoiseState << 5;
        const float whiteNoise =
            ((float)(fluidNoiseState & 0xffffu) / 32767.5f) - 1.0f;
        constexpr float RESONATOR_RADIUS = 0.9975f;
        const float resonanceAngle =
            TWO_PI_F * fluidBurstFrequencyHz / SAMPLE_RATE;
        const float resonator =
            2.0f * RESONATOR_RADIUS * cosf(resonanceAngle) *
                fluidResonatorY1 -
            RESONATOR_RADIUS * RESONATOR_RADIUS * fluidResonatorY2 +
            (1.0f - RESONATOR_RADIUS) * whiteNoise;
        fluidResonatorY2 = fluidResonatorY1;
        fluidResonatorY1 = resonator;

        const float cavityTone = sinf(fluidBurstPhase);
        const float resonantNoise = clampSigned(4.0f * resonator);
        fluidBurstSample = envelope * clampSigned(
            0.58f * cavityTone + 0.42f * resonantNoise);

        fluidBurstPhase +=
            TWO_PI_F * fluidBurstFrequencyHz / SAMPLE_RATE;
        if (fluidBurstPhase >= TWO_PI_F) fluidBurstPhase -= TWO_PI_F;
        ++fluidBurstAgeSamples;
      }
    }

    for (int corner = 0; corner < 4; ++corner) {
      // Material-specific amplitude following changes the perceived viscosity.
      currentAmplitude[corner] +=
          material.amplitudeFollow *
          (targetAmplitude[corner] - currentAmplitude[corner]);

      const float value = asymmetricWave(phase[corner], material.harmonicMix);
      const float directionalOutput = currentAmplitude[corner] * value;
      const float fluidBurstGain =
          materialIndex == WATER_MATERIAL_INDEX
              ? WATER_BURST_GAIN
              : YOGURT_BURST_GAIN;
      const float burstOutput =
          MAX_OUTPUT_AMPLITUDE * MASTER_GAIN * fluidBurstGain *
          fluidBurstStrength[corner] * fluidBurstSample;
      const float output = ACTUATOR_POLARITY[corner] *
          (directionalOutput + burstOutput);
      wave[corner] = (int16_t)constrain((int)output, -32767, 32767);

      phase[corner] += TWO_PI_F * playbackFrequencyHz / SAMPLE_RATE;
      if (phase[corner] >= TWO_PI_F) phase[corner] -= TWO_PI_F;
    }

    for (int slot = 0; slot < TDM_SLOTS; ++slot) {
      samples[i * TDM_SLOTS + slot] = 0;
    }
    for (int corner = 0; corner < 4; ++corner) {
      samples[i * TDM_SLOTS + SLOT_FOR_CORNER[corner]] = wave[corner];
    }
  }

  size_t bytesWritten = 0;
  const esp_err_t err = i2s_channel_write(
      tx_chan, samples, sizeof(samples), &bytesWritten, portMAX_DELAY);

  if (err != ESP_OK) {
    Serial.printf("I2S write error: %d\n", err);
  }
}

