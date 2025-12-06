#pragma once

#include <string>
#include <cstdint>

// From external/liquidcrystal-i2c
#include "LiquidCrystal_I2C.h"

class Lcd1602Display
{
public:
    // addr usually 0x27 or 0x3F
    Lcd1602Display(uint8_t addr = 0x27, uint8_t cols = 16, uint8_t rows = 2);

    bool init();
    void clear();
    void print_line(uint8_t row, const std::string &text);
    uint8_t cols() const { return cols_; }

private:
    LiquidCrystal_I2C lcd_;
    uint8_t cols_;
    uint8_t rows_;
};