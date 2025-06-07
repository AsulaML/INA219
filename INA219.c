#include "ina219.h"

static int ina219_write_reg(ina219_t* dev, uint8_t reg, uint16_t value) {
    return dev->bus.i2c_write(dev->address, reg, value);
}

static int ina219_read_reg(ina219_t* dev, uint8_t reg, uint16_t* value) {
    return dev->bus.i2c_read(dev->address, reg, value);
}

int ina219_init(ina219_t* dev, uint8_t address, ina219_bus_t bus) {
    dev->address = address;
    dev->bus = bus;
    return 0;
}

int ina219_calibrate(ina219_t* dev, float shunt_resistance, float max_current) {
    // Calcul du Current_LSB
    dev->current_lsb = max_current / 32767.0f;

    // Calibration Register = 0.04096 / (Current_LSB * Rshunt)
    dev->calibration_value = (uint16_t)(0.04096f / (dev->current_lsb * shunt_resistance));
    dev->power_lsb = dev->current_lsb * 20.0f;

    return ina219_write_reg(dev, INA219_REG_CALIBRATION, dev->calibration_value);
}

int ina219_read_shunt_voltage(ina219_t* dev, float* voltage) {
    int16_t raw;
    if (ina219_read_reg(dev, INA219_REG_SHUNT_VOLTAGE, (uint16_t*)&raw) != 0)
        return -1;
    *voltage = raw * 0.00001f; // 10 µV/bit
    return 0;
}

int ina219_read_bus_voltage(ina219_t* dev, float* voltage) {
    uint16_t raw;
    if (ina219_read_reg(dev, INA219_REG_BUS_VOLTAGE, &raw) != 0)
        return -1;
    raw >>= 3; // 3 LSB inutiles
    *voltage = raw * 0.004f; // 4 mV/bit
    return 0;
}

int ina219_read_current(ina219_t* dev, float* current) {
    int16_t raw;
    if (ina219_read_reg(dev, INA219_REG_CURRENT, (uint16_t*)&raw) != 0)
        return -1;
    *current = raw * dev->current_lsb;
    return 0;
}

int ina219_read_power(ina219_t* dev, float* power) {
    uint16_t raw;
    if (ina219_read_reg(dev, INA219_REG_POWER, &raw) != 0)
        return -1;
    *power = raw * dev->power_lsb;
    return 0;
}
