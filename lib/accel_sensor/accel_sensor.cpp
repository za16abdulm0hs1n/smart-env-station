/**
 * @file accel_sensor.cpp
 * @brief Minimal I2C accelerometer implementation with auto-detect:
 *        - MPU-6050 (addr 0x68/0x69)
 *        - LIS3DH/LIS2DH family (addr 0x18/0x19)
 *
 * API stays the same as accel_sensor.h:
 *   - bool  begin(uint8_t addr)
 *   - float readG()   // returns |a| in g (NAN on failure)
 *
 * NOTE: We keep header unchanged and do device-type handling internally.
 */
#include "accel_sensor.h"
#include <math.h>

// --- Local helpers ---------------------------------------------------------
static bool i2c_write8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool i2c_readN(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n, bool autoInc = false) {
  // For parts that need auto-increment via MSB (e.g., LIS3DH), caller can pass reg|0x80
  Wire.beginTransmission(addr);
  Wire.write(autoInc ? (reg | 0x80) : reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  const int got = Wire.requestFrom((int)addr, (int)n);
  if (got != (int)n) return false;
  for (int i = 0; i < got; ++i) buf[i] = Wire.read();
  return true;
}

static bool i2c_read8(uint8_t addr, uint8_t reg, uint8_t& val) {
  if (!i2c_readN(addr, reg, &val, 1, /*autoInc=*/false)) return false;
  return true;
}

// --- Device IDs ------------------------------------------------------------
// MPU-6050 WHO_AM_I @0x75: 0x68 (AD0=0) or 0x69 (AD0=1)
// LIS3DH WHO_AM_I  @0x0F: 0x33
enum DevType : uint8_t { DEV_NONE=0, DEV_MPU6050=1, DEV_LIS3DH=2 };

// Single-instance state (project only uses one AccelSensor)
static DevType s_dev = DEV_NONE;
static uint8_t s_addr = 0x00;

// --- Public methods --------------------------------------------------------
bool AccelSensor::begin(uint8_t addr) {
  _addr = addr;
  s_addr = addr;

  // Probe basic presence
  Wire.beginTransmission(_addr);
  if (Wire.endTransmission() != 0) {
    _ok = false;
    s_dev = DEV_NONE;
    return false;
  }

  // Try detect MPU-6050
  uint8_t who = 0;
  if (i2c_read8(_addr, 0x75, who) && (who == 0x68 || who == 0x69)) {
    // Wake from sleep: PWR_MGMT_1 = 0
    if (!i2c_write8(_addr, 0x6B, 0x00)) { _ok = false; s_dev = DEV_NONE; return false; }
    // Set accel full-scale ±2g (ACCEL_CONFIG=0x1C -> 0x00)
    (void)i2c_write8(_addr, 0x1C, 0x00);
    _ok  = true;
    s_dev = DEV_MPU6050;
    return true;
  }

  // Try detect LIS3DH / LIS2DH family
  who = 0;
  if (i2c_read8(_addr, 0x0F, who) && who == 0x33) {
    // CTRL_REG1 (0x20): 0b0101 0111 = 0x57
    //  ODR=100 Hz, LPen=0, Zen/Yen/Xen = 1 (enable all axes)
    if (!i2c_write8(_addr, 0x20, 0x57)) { _ok = false; s_dev = DEV_NONE; return false; }
    // CTRL_REG4 (0x23): BDU=1 (bit7), HR=1 (bit3), FS=±2g (bits4-5=00) => 0b1000 1000 = 0x88
    (void)i2c_write8(_addr, 0x23, 0x88);
    _ok  = true;
    s_dev = DEV_LIS3DH;
    return true;
  }

  // Unknown device at this address
  _ok = false;
  s_dev = DEV_NONE;
  return false;
}

float AccelSensor::readG() {
  if (s_dev == DEV_NONE) { _ok = false; return NAN; }

  if (s_dev == DEV_MPU6050) {
    // Read 6 bytes starting at ACCEL_XOUT_H (0x3B): order is Hi,Lo per axis
    uint8_t d[6];
    if (!i2c_readN(s_addr, 0x3B, d, 6, /*autoInc=*/false)) { _ok = false; return NAN; }
    int16_t ax = (int16_t)((d[0] << 8) | d[1]);
    int16_t ay = (int16_t)((d[2] << 8) | d[3]);
    int16_t az = (int16_t)((d[4] << 8) | d[5]);
    _ok = true;
    const float axg = ax / 16384.0f;  // ±2g
    const float ayg = ay / 16384.0f;
    const float azg = az / 16384.0f;
    return sqrtf(axg*axg + ayg*ayg + azg*azg);
  }

  if (s_dev == DEV_LIS3DH) {
    // Read 6 bytes starting at OUT_X_L (0x28), with auto-increment (bit7=1)
    uint8_t d[6];
    if (!i2c_readN(s_addr, 0x28, d, 6, /*autoInc=*/true)) { _ok = false; return NAN; }
    // Data order: XL, XH, YL, YH, ZL, ZH (low first). In HR mode, left-justified 12 bits.
    int16_t x = (int16_t)((d[1] << 8) | d[0]);
    int16_t y = (int16_t)((d[3] << 8) | d[2]);
    int16_t z = (int16_t)((d[5] << 8) | d[4]);
    x >>= 4; y >>= 4; z >>= 4; // align to 12-bit signed

    // Sensitivity in HR ±2g ≈ 1 mg/LSB → 0.001 g/LSB
    const float axg = (float)x * 0.001f;
    const float ayg = (float)y * 0.001f;
    const float azg = (float)z * 0.001f;
    _ok = true;
    return sqrtf(axg*axg + ayg*ayg + azg*azg);
  }

  _ok = false;
  return NAN;
}