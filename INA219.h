#ifndef INA219_H
#define INA219_H

#include <stdint.h>

// Adresse I2C par défaut de l'INA219 (configurable)
#define INA219_DEFAULT_ADDRESS 0x40

// Registres de l'INA219
typedef enum {
    INA219_REG_CONFIG        = 0x00,
    INA219_REG_SHUNT_VOLTAGE = 0x01,
    INA219_REG_BUS_VOLTAGE   = 0x02,
    INA219_REG_POWER         = 0x03,
    INA219_REG_CURRENT       = 0x04,
    INA219_REG_CALIBRATION   = 0x05
} ina219_reg_t;

// Structure pour contenir les fonctions bas niveau (abstraction HAL I2C)
typedef struct {
    int (*i2c_write)(uint8_t addr, uint8_t reg, uint16_t data);
    int (*i2c_read)(uint8_t addr, uint8_t reg, uint16_t* data);
} ina219_bus_t;

// Structure principale de l'INA219
typedef struct {
    uint8_t address;
    ina219_bus_t bus;
    float current_lsb;   // En A/bit
    float power_lsb;     // En W/bit
    uint16_t calibration_value;
} ina219_t;

// Fonctions publiques
int ina219_init(ina219_t* dev, uint8_t address, ina219_bus_t bus);
int ina219_calibrate(ina219_t* dev, float shunt_resistance, float max_expected_current);
int ina219_read_shunt_voltage(ina219_t* dev, float* voltage);
int ina219_read_bus_voltage(ina219_t* dev, float* voltage);
int ina219_read_current(ina219_t* dev, float* current);
int ina219_read_power(ina219_t* dev, float* power);

#endif // INA219_H
