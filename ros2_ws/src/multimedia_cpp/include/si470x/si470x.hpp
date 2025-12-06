#pragma once

#include <string>
#include <cstdint>

class Si470x
{
public:
    // i2c_dev: e.g. "/dev/i2c-1"
    // addr: usual Si470x address is 0x10
    // reset_gpio: BCM GPIO number for RST pin (e.g. 27)
    Si470x(const std::string &i2c_dev, int addr, int reset_gpio);
    ~Si470x();

    bool init();
    bool set_frequency_10k(int ch10);  // 995 => 99.5 MHz

private:
    int fd_;
    int addr_;
    int reset_gpio_;

    bool reset_chip();
    bool read_registers(uint16_t *regs, int count = 16);
    bool write_registers(uint16_t *regs, int count = 16);
};