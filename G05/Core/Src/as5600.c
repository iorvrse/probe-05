#include "as5600.h"

int32_t _position = 0, _lastPosition = 0;
extern I2C_HandleTypeDef hi2c2;

int16_t readReg2(uint8_t reg)
{
    uint8_t data[2];
    HAL_I2C_Mem_Read(&hi2c2, AS5600_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
    return (data[0] << 8) | data[1];
}

int32_t getCumulativePosition(void)
{
    int16_t value = readReg2(AS5600_ANGLE) & 0x0FFF;

    // Whole rotation CW?
    if ((_lastPosition > 2048) && (value < (_lastPosition - 2048))) {
        _position = _position + 4095 - _lastPosition + value;
    }
    // Whole rotation CCW?
    else if ((value > 2048) && (_lastPosition < (value - 2048))) {
        _position = _position - 4095 - _lastPosition + value;
    } else
        _position = _position - _lastPosition + value;
    _lastPosition = value;

    return _position;
}

int32_t resetPosition(int32_t position)
{
    int32_t old = _position;
    _position = position;
    return old;
}

int32_t resetCumulativePosition(int32_t position)
{
    _lastPosition = readReg2(AS5600_RAW_ANGLE) & 0x0FFF;
    int32_t old = _position;
    _position = position;
    return old;
}
