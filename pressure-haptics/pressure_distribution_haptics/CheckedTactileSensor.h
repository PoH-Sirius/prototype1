#pragma once
#include <DFRobot_TactileSensor.h>

// Keep the vendor transport, but never decode a failed read. The upstream
// getModel() checks 0xff although DFRobot_RTU returns 0x09 on receive errors.
class CheckedTactileSensor : public DFRobot_TactileSensor {
public:
  CheckedTactileSensor(uint8_t address, Stream* stream)
      : DFRobot_TactileSensor(address, stream) { _array = 0; }

  uint8_t beginChecked(uint16_t& model) {
    _array = 0;
    uint8_t bytes[2] = {};
    const uint8_t status = readReg(INPUTREG_MODEL, bytes, sizeof(bytes), INPUTREG);
    if (status != 0) return status;
    model = (uint16_t(bytes[0]) << 8) | bytes[1];
    if (model > 1) return 0xfe;  // Valid reply, unsupported sensor model.
    _array = model == 0 ? 32 : 36;
    tactileSensorArrayX = model == 0 ? 8 : 6;
    tactileSensorArrayY = model == 0 ? 4 : 6;
    return 0;
  }

  sAdcDatas_t getDatasChecked() {
    sAdcDatas_t result = {};
    if (_array != 32 && _array != 36) {
      result.result = 0xfe;
      return result;
    }
    uint8_t bytes[72] = {};
    result.result = readReg(INPUTREG_GETDATAS, bytes, _array * 2, INPUTREG);
    if (result.result != 0) return result;
    for (uint8_t y = 0; y < tactileSensorArrayY; ++y) {
      for (uint8_t x = 0; x < tactileSensorArrayX; ++x) {
        const unsigned i = ((tactileSensorArrayY - 1 - y) * tactileSensorArrayX + x) * 2;
        result.adcval[y][x] = (uint16_t(bytes[i]) << 8) | bytes[i + 1];
      }
    }
    return result;
  }
};
